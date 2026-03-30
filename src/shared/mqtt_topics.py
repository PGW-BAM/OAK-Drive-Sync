"""
Canonical MQTT topic definitions.

All topic strings used anywhere in the system MUST be imported from here.
Do not hardcode topic strings in any other module.
"""

# --- Commands (Windows → Pi) ---
CMD_DRIVE_MOVE = "cmd/drives/{cam_id}/move"
CMD_DRIVE_HOME = "cmd/drives/{cam_id}/home"
CMD_DRIVE_STOP = "cmd/drives/{cam_id}/stop"

# --- Status (Pi → Windows) ---
STATUS_DRIVE_POSITION = "status/drives/{cam_id}/position"
STATUS_CAMERA_STATE = "status/cameras/{cam_id}/state"

# --- Health Beacons ---
HEALTH_PI = "health/pi"
HEALTH_WIN_CONTROLLER = "health/win_controller"
HEALTH_CAMERA = "health/cameras/{cam_id}"

# --- Error Events ---
ERROR_DRIVE = "error/drives/{cam_id}"
ERROR_CAMERA = "error/cameras/{cam_id}"
ERROR_ORCHESTRATION = "error/orchestration/{event}"

# --- Monitoring ---
MONITORING_CONNECTIVITY = "monitoring/connectivity"

# --- Configuration ---
CONFIG_SEQUENCE_ACTIVE = "config/sequence/active"

# --- Wildcards for subscriptions ---
SUB_ALL_DRIVE_STATUS = "status/drives/+/position"
SUB_ALL_CAMERA_STATUS = "status/cameras/+/state"
SUB_ALL_HEALTH = "health/#"
SUB_ALL_ERRORS = "error/#"
SUB_ALL_DRIVE_CMDS = "cmd/drives/+/+"


def topic(template: str, **kwargs: str) -> str:
    """Format a topic template with the given parameters.

    Usage:
        topic(CMD_DRIVE_MOVE, cam_id="cam1")
        → "cmd/drives/cam1/move"
    """
    return template.format(**kwargs)
