"""
Ephemeral motor command helpers for MuJoCo controller.

Kept dependency-free so unit tests can import without numpy/mujoco/feagi.
"""


def motor_rx_is_new_packet(rx_seq: object, last_seen_rx_seq: int) -> bool:
    """
    True when the motor callback sequence advanced (new FEAGI motor packet).

    The MuJoCo controller uses this with incremental / effort-absolute modes to
    choose between refreshing SDK values and holding the last applied ctrl at
    physics rate when FEAGI does not send a new packet every simulation step.
    """
    return isinstance(rx_seq, int) and rx_seq != int(last_seen_rx_seq)
