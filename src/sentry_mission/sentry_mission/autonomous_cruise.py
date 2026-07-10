"""Small pure helpers for autonomous cruise state decisions."""


AUTO_STATES = {
    "PATROL",
    "STOPPED",
    "SCANNING",
    "ANALYZING",
    "ACTION",
    "RESUME",
}


def mission_state_to_mode(state: str) -> str:
    """Return the operator-facing mode for a mission state."""
    return "MANUAL" if state == "MANUAL" else "AUTO"


def should_send_patrol_goal(
    state: str,
    nav2_ready: bool,
    sending_goal: bool,
    current_wp_idx: int,
    waypoint_count: int,
) -> bool:
    """Whether mission control should dispatch the current patrol waypoint."""
    return (
        state == "PATROL"
        and nav2_ready
        and not sending_goal
        and current_wp_idx < waypoint_count
    )
