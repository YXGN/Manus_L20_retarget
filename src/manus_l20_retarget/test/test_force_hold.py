from manus_l20_retarget.retarget_pipeline import FINGER_CONTROL_SLOTS, TactileForceHold


OPEN = [255] * 20
CLOSED = [0] * 20


def test_force_hold_latches_contact_and_releases_for_opening():
    hold = TactileForceHold(
        neutral_command=OPEN,
        closed_command=CLOSED,
        force_threshold=5.0,
        contact_samples=2,
        release_delta=0.05,
        feedback_timeout_sec=0.1,
    )
    blocked = hold.apply(CLOSED, CLOSED, OPEN, now=0.0, hold_command=OPEN)
    for slots in FINGER_CONTROL_SLOTS:
        assert [blocked[slot] for slot in slots] == [255] * len(slots)
    hold.update_force([0.0, 8.0, 0.0, 0.0, 0.0], now=0.01)
    hold.update_force([0.0, 8.0, 0.0, 0.0, 0.0], now=0.02)
    measured = list(OPEN)
    for slot in FINGER_CONTROL_SLOTS[1]:
        measured[slot] = 180
    output = hold.apply(CLOSED, CLOSED, OPEN, now=0.02, hold_command=measured)
    assert [output[slot] for slot in FINGER_CONTROL_SLOTS[1]] == [180] * 3
    assert hold.apply(OPEN, OPEN, measured, now=0.03, hold_command=measured) == OPEN
