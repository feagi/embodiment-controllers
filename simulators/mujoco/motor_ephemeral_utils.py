"""
Ephemeral motor command helpers for MuJoCo controller.

Kept dependency-free so unit tests can import without numpy/mujoco/feagi.
"""


def motor_rx_is_new_packet(rx_seq: object, last_seen_rx_seq: int) -> bool:
    """
    True when the motor callback sequence advanced (new FEAGI motor packet).

    Ephemeral policy: drive actuators from get_angle/get_speed only on ticks
    where this is True; otherwise use neutral ctrl.
    """
    return isinstance(rx_seq, int) and rx_seq != int(last_seen_rx_seq)
