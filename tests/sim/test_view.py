"""Light smoke tests for the 3D view helpers (skipped without NiceGUI)."""

from __future__ import annotations

import pytest

pytest.importorskip("nicegui")

from src.sim.view import _explain, _stl_bytes  # noqa: E402
from tests.sim.conftest import neutral_rig  # noqa: E402


def test_stl_bytes_layout() -> None:
    mesh = neutral_rig().specimen.mesh()
    data = _stl_bytes(mesh)
    assert len(data) == 84 + 50 * len(mesh.faces)


def test_explain_names_limit() -> None:
    rig = neutral_rig()
    res = rig.solve("cam1", 200.0)
    assert not res.reachable
    assert "Soft-Max" in _explain(rig, "cam1", res)
