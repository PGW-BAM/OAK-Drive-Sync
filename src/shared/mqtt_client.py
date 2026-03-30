"""
Reusable async MQTT client wrapper.

Provides:
- Automatic reconnection with exponential backoff
- Last Will and Testament configuration
- Pydantic model serialization/deserialization
- Structured logging via structlog
"""

from __future__ import annotations

import asyncio
import json
from typing import Any, Callable, Coroutine, TypeVar

import aiomqtt
import structlog
from pydantic import BaseModel

logger = structlog.get_logger()

T = TypeVar("T", bound=BaseModel)

MessageHandler = Callable[[str, dict[str, Any]], Coroutine[Any, Any, None]]


class MQTTClient:
    """Async MQTT client with auto-reconnect and Pydantic integration."""

    def __init__(
        self,
        *,
        broker_host: str = "localhost",
        broker_port: int = 1883,
        client_id: str,
        lwt_topic: str | None = None,
        lwt_payload: dict[str, Any] | None = None,
        reconnect_min_s: float = 1.0,
        reconnect_max_s: float = 30.0,
    ) -> None:
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client_id = client_id
        self.lwt_topic = lwt_topic
        self.lwt_payload = lwt_payload
        self.reconnect_min_s = reconnect_min_s
        self.reconnect_max_s = reconnect_max_s

        self._subscriptions: dict[str, MessageHandler] = {}
        self._client: aiomqtt.Client | None = None
        self._connected = asyncio.Event()
        self._stop = asyncio.Event()

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
        """Publish a message. Payload can be a Pydantic model or dict."""
        if not self._connected.is_set():
            logger.warning("mqtt.publish_while_disconnected", topic=topic)
            await self._connected.wait()

        if isinstance(payload, BaseModel):
            data = payload.model_dump_json()
        else:
            data = json.dumps(payload)

        assert self._client is not None
        await self._client.publish(topic, data, qos=qos, retain=retain)
        logger.debug("mqtt.published", topic=topic, qos=qos, retain=retain)

    def subscribe(self, topic_filter: str, handler: MessageHandler) -> None:
        """Register a handler for a topic filter. Call before run()."""
        self._subscriptions[topic_filter] = handler
        logger.info("mqtt.handler_registered", topic_filter=topic_filter)

    async def run(self) -> None:
        """Main loop: connect, subscribe, dispatch messages. Auto-reconnects."""
        backoff = self.reconnect_min_s

        while not self._stop.is_set():
            try:
                will = None
                if self.lwt_topic and self.lwt_payload:
                    will = aiomqtt.Will(
                        topic=self.lwt_topic,
                        payload=json.dumps(self.lwt_payload),
                        qos=1,
                        retain=True,
                    )

                async with aiomqtt.Client(
                    hostname=self.broker_host,
                    port=self.broker_port,
                    identifier=self.client_id,
                    will=will,
                ) as client:
                    self._client = client
                    self._connected.set()
                    backoff = self.reconnect_min_s
                    logger.info(
                        "mqtt.connected",
                        broker=self.broker_host,
                        port=self.broker_port,
                    )

                    # Subscribe to all registered topic filters
                    for topic_filter in self._subscriptions:
                        await client.subscribe(topic_filter, qos=1)
                        logger.info("mqtt.subscribed", topic_filter=topic_filter)

                    # Message dispatch loop
                    async for message in client.messages:
                        topic_str = str(message.topic)
                        try:
                            payload = json.loads(message.payload)  # type: ignore[arg-type]
                        except (json.JSONDecodeError, TypeError):
                            logger.warning("mqtt.invalid_json", topic=topic_str)
                            continue

                        # Find matching handler (exact match or wildcard)
                        for topic_filter, handler in self._subscriptions.items():
                            if _topic_matches(topic_filter, topic_str):
                                try:
                                    await handler(topic_str, payload)
                                except Exception:
                                    logger.exception(
                                        "mqtt.handler_error",
                                        topic=topic_str,
                                        handler=handler.__name__,
                                    )

            except aiomqtt.MqttError as e:
                self._connected.clear()
                self._client = None
                logger.warning(
                    "mqtt.disconnected",
                    error=str(e),
                    reconnect_in=backoff,
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, self.reconnect_max_s)

            except asyncio.CancelledError:
                break

        self._connected.clear()
        self._client = None
        logger.info("mqtt.stopped")

    async def stop(self) -> None:
        """Signal the run loop to stop."""
        self._stop.set()

    @staticmethod
    def parse_payload(payload: dict[str, Any], model: type[T]) -> T:
        """Parse a raw dict payload into a Pydantic model."""
        return model.model_validate(payload)


def _topic_matches(topic_filter: str, topic: str) -> bool:
    """Check if a topic matches a filter (supports + and # wildcards)."""
    filter_parts = topic_filter.split("/")
    topic_parts = topic.split("/")

    for i, fp in enumerate(filter_parts):
        if fp == "#":
            return True
        if i >= len(topic_parts):
            return False
        if fp != "+" and fp != topic_parts[i]:
            return False

    return len(filter_parts) == len(topic_parts)
