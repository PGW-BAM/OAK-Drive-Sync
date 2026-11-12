"""In-process fake MQTT broker + client for the digital twin and tests.

``FakeMQTTClient`` implements the subset of ``src.shared.mqtt_client.MQTTClient`` used by
``DriveManager`` / ``DriftDetector`` / ``SimImuResponder``: ``publish``, ``subscribe``,
``run``, ``stop``, ``connected``, ``_connected``, ``parse_payload``.

Faithfulness notes (reproduces the real client on purpose)
----------------------------------------------------------
* ``dispatch="per_topic"`` (default): uses the same ``TopicDispatcher`` as
  ``MQTTClient.run`` (ordered per topic, concurrent across topics).
* ``dispatch="serial"``: the OLD real-client behaviour, kept to document the bug - the
  receive loop *awaits each handler inline*. A handler that waits for another incoming
  message on the SAME client (e.g. ``DriveManager.handle_move`` -> ``converge`` -> ``request_imu_check``) therefore
  blocks the loop and the awaited reply cannot be dispatched until the wait times out.
  ``dispatch="concurrent"`` runs every handler as its own task (the fix).
* Payloads are JSON round-tripped (``model_dump_json`` / ``json.loads``) like the real wire.
* Broker-side subscriptions are snapshotted when ``run()`` connects (``subscribe`` after
  connect registers a handler but does not subscribe at the broker), as in ``MQTTClient``.
* ``publish`` before ``run()`` has connected blocks until connected (real: same).
* Retained messages are replayed to a client right after it subscribes (on connect).
* Not simulated: QoS, LWT delivery, reconnects. ``stop()`` additionally ends ``run()``
  (the real one only sets a flag and keeps looping until the connection drops).
"""

from __future__ import annotations

import asyncio
import json
from typing import TYPE_CHECKING, Any, Literal, TypeVar

import structlog
from pydantic import BaseModel

from src.shared.mqtt_client import MessageHandler, TopicDispatcher, _topic_matches

if TYPE_CHECKING:
    from collections.abc import Iterator

logger = structlog.get_logger()

T = TypeVar("T", bound=BaseModel)

Dispatch = Literal["per_topic", "serial", "concurrent"]


class FakeBroker:
    """Routes published messages to the queues of all matching, connected clients."""

    def __init__(self) -> None:
        self.retained: dict[str, str] = {}
        self.published: list[tuple[str, dict[str, Any]]] = []  # every message, in order
        self._clients: list[FakeMQTTClient] = []

    def client(
        self, client_id: str, *, dispatch: Dispatch = "per_topic", **kwargs: Any
    ) -> FakeMQTTClient:
        c = FakeMQTTClient(self, client_id=client_id, dispatch=dispatch, **kwargs)
        self._clients.append(c)
        return c

    def messages(self, topic_filter: str) -> Iterator[dict[str, Any]]:
        """Payloads published so far on topics matching ``topic_filter`` (for assertions)."""
        return (p for t, p in self.published if _topic_matches(topic_filter, t))

    def _publish(self, topic: str, data: str, retain: bool) -> None:
        self.published.append((topic, json.loads(data)))
        if retain:
            if data:
                self.retained[topic] = data
            else:
                self.retained.pop(topic, None)
        for c in self._clients:
            if c._broker_filters and any(_topic_matches(f, topic) for f in c._broker_filters):
                c._queue.put_nowait((topic, data))

    def _retained_for(self, filters: set[str]) -> list[tuple[str, str]]:
        return [
            (t, d) for t, d in self.retained.items() if any(_topic_matches(f, t) for f in filters)
        ]


class FakeMQTTClient:
    """Drop-in for ``MQTTClient`` talking to a ``FakeBroker`` (see module docstring)."""

    def __init__(
        self,
        broker: FakeBroker,
        *,
        client_id: str,
        dispatch: Dispatch = "per_topic",
        broker_host: str = "fake",
        broker_port: int = 0,
        lwt_topic: str | None = None,
        lwt_payload: dict[str, Any] | None = None,
    ) -> None:
        if dispatch not in ("per_topic", "serial", "concurrent"):
            raise ValueError(f"invalid dispatch mode {dispatch!r}")
        self.broker = broker
        self.client_id = client_id
        self.dispatch = dispatch
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.lwt_topic = lwt_topic
        self.lwt_payload = lwt_payload

        self._subscriptions: dict[str, MessageHandler] = {}
        self._broker_filters: set[str] = set()
        self._queue: asyncio.Queue[tuple[str, str] | None] = asyncio.Queue()
        self._connected = asyncio.Event()
        self._stop = asyncio.Event()
        self._tasks: set[asyncio.Task[None]] = set()
        self._dispatcher = TopicDispatcher()

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    async def publish(
        self,
        topic: str,
        payload: BaseModel | dict[str, Any],
        qos: int = 1,
        retain: bool = False,
    ) -> None:
        if not self._connected.is_set():
            logger.warning("fake_mqtt.publish_while_disconnected", topic=topic)
            await self._connected.wait()
        data = payload.model_dump_json() if isinstance(payload, BaseModel) else json.dumps(payload)
        self.broker._publish(topic, data, retain)

    def subscribe(self, topic_filter: str, handler: MessageHandler) -> None:
        """Register a handler. Like the real client: call before ``run()``."""
        self._subscriptions[topic_filter] = handler

    async def run(self) -> None:
        """Connect, subscribe, dispatch (serial or concurrent). Returns after ``stop()``."""
        self._broker_filters = set(self._subscriptions)
        self._connected.set()
        for topic, data in self.broker._retained_for(self._broker_filters):
            self._queue.put_nowait((topic, data))
        try:
            while not self._stop.is_set():
                item = await self._queue.get()
                if item is None:
                    break
                topic_str, data = item
                try:
                    payload = json.loads(data)
                except (json.JSONDecodeError, TypeError):
                    logger.warning("fake_mqtt.invalid_json", topic=topic_str)
                    continue
                if self.dispatch == "per_topic":  # exactly what MQTTClient.run does
                    self._dispatcher.dispatch(self._subscriptions, topic_str, payload)
                    continue
                for topic_filter, handler in list(self._subscriptions.items()):
                    if not _topic_matches(topic_filter, topic_str):
                        continue
                    if self.dispatch == "serial":
                        await self._call(handler, topic_str, payload)  # inline, like the real one
                    else:
                        task = asyncio.create_task(self._call(handler, topic_str, payload))
                        self._tasks.add(task)
                        task.add_done_callback(self._tasks.discard)
        except asyncio.CancelledError:
            pass
        finally:
            for task in list(self._tasks):
                task.cancel()
            await self._dispatcher.close()
            self._connected.clear()
            self._broker_filters = set()

    @staticmethod
    async def _call(handler: MessageHandler, topic_str: str, payload: dict[str, Any]) -> None:
        try:
            await handler(topic_str, payload)
        except Exception:
            logger.exception("fake_mqtt.handler_error", topic=topic_str)

    async def stop(self) -> None:
        self._stop.set()
        self._queue.put_nowait(None)

    @staticmethod
    def parse_payload(payload: dict[str, Any], model: type[T]) -> T:
        return model.model_validate(payload)
