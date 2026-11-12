"""Digital twin of the camera test rig (headless kinematics core).

See docs/SIMULATION.md. Pure Python (``math``), no hardware, no MQTT.
"""

from src.sim.rig_config import RigConfig, load_rig_config
from src.sim.rig_model import RigModel

__all__ = ["RigConfig", "RigModel", "load_rig_config"]
