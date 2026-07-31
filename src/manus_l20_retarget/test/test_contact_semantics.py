from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from manus_l20_retarget.contact_semantics import (
    FINGERTIP_CONTACT_SLOTS,
    FingertipContactStateMachine,
    blend_fingertip_contact_command,
    parse_fingertip_contact_config,
    thumb_fingertip_distance_ratios,
)


def _config_data() -> dict:
    contacts = {}
    for finger, slots in FINGERTIP_CONTACT_SLOTS.items():
        contacts[finger] = {
            "human": {
                "natural_open_distance_p05_ratio": 0.70,
                "contact_distance_p95_ratio": 0.10,
            },
            "robot": {
                "override_slots": list(slots),
                "max_command_delta": 20,
                "contact_command": [10] * 20,
            },
        }
    return {
        "schema": "manus_l20.fingertip_contact_semantics.v2",
        "runtime": {
            "enabled": True,
            "min_hold_sec": 0.10,
            "release_hold_sec": 0.05,
            "candidate_gap_ratio": 0.02,
            "takeover_start_progress": 0.50,
            "activation_rise_sec": 0.10,
            "activation_release_sec": 0.10,
        },
        "contacts": contacts,
    }


def _finger(mcp, tip):
    return SimpleNamespace(mcp=np.asarray(mcp, dtype=np.float64), tip=np.asarray(tip, dtype=np.float64))


class FingertipContactSemanticsTest(unittest.TestCase):
    def test_tip_distances_are_normalized_by_palm_width(self) -> None:
        skeleton = SimpleNamespace(
            thumb=_finger((0.0, 0.0, 0.0), (0.25, 0.0, 0.0)),
            index=_finger((0.0, 0.0, 0.0), (0.50, 0.0, 0.0)),
            middle=_finger((0.3, 0.0, 0.0), (0.25, 0.5, 0.0)),
            ring=_finger((0.7, 0.0, 0.0), (0.25, 0.75, 0.0)),
            pinky=_finger((1.0, 0.0, 0.0), (0.25, 1.0, 0.0)),
        )

        ratios = thumb_fingertip_distance_ratios(skeleton)

        self.assertAlmostEqual(ratios["index"], 0.25)
        self.assertAlmostEqual(ratios["middle"], 0.50)
        self.assertAlmostEqual(ratios["ring"], 0.75)
        self.assertAlmostEqual(ratios["pinky"], 1.00)

    def test_state_machine_latches_then_releases_with_hysteresis(self) -> None:
        config = parse_fingertip_contact_config(_config_data())
        state = FingertipContactStateMachine(config)
        near_index = {"index": 0.10, "middle": 0.60, "ring": 0.70, "pinky": 0.80}

        self.assertIsNone(state.update(near_index, 0.00).pair)
        self.assertIsNone(state.update(near_index, 0.05).pair)
        active = state.update(near_index, 0.11)
        self.assertEqual(active.pair, "index")
        self.assertEqual(active.event, "activated:index")
        self.assertGreater(active.activation, 0.0)

        held = state.update({**near_index, "index": 0.30}, 0.20)
        self.assertEqual(held.pair, "index")
        self.assertIsNone(held.event)

        self.assertIsNone(state.update({**near_index, "index": 0.50}, 0.21).event)
        released = state.update({**near_index, "index": 0.50}, 0.27)
        self.assertEqual(released.event, "released:index")
        self.assertIsNone(released.pair)
        self.assertEqual(released.activation, 0.0)

    def test_blend_changes_only_selected_slots_and_respects_delta_limit(self) -> None:
        config = parse_fingertip_contact_config(_config_data())
        profile = config.profiles["index"]
        base = [100] * 20

        blended = blend_fingertip_contact_command(base, profile, 0.5)

        for slot in profile.override_slots:
            self.assertEqual(blended[slot], 90)
        for slot in set(range(20)) - set(profile.override_slots):
            self.assertEqual(blended[slot], 100)

    def test_distance_controls_full_pose_takeover(self) -> None:
        data = _config_data()
        data["runtime"].update(
            {
                "min_hold_sec": 0.0,
                "activation_rise_sec": 0.0,
                "activation_release_sec": 0.0,
            }
        )
        config = parse_fingertip_contact_config(data)
        state = FingertipContactStateMachine(config)
        ratios = {"index": 0.35, "middle": 0.70, "ring": 0.70, "pinky": 0.70}

        state.update(ratios, 0.00)
        partial = state.update(ratios, 0.01)
        self.assertEqual(partial.pair, "index")
        self.assertAlmostEqual(partial.activation, 1.0 / 6.0)

        full = state.update({**ratios, "index": 0.10}, 0.02)
        self.assertEqual(full.pair, "index")
        self.assertEqual(full.activation, 1.0)

    def test_full_takeover_reaches_contact_command_for_dynamic_slots(self) -> None:
        data = _config_data()
        for contact in data["contacts"].values():
            contact["robot"]["max_command_delta"] = 255
        profile = parse_fingertip_contact_config(data).profiles["index"]

        blended = blend_fingertip_contact_command([100] * 20, profile, 1.0)

        for slot in profile.override_slots:
            self.assertEqual(blended[slot], 10)
        for slot in set(range(20)) - set(profile.override_slots):
            self.assertEqual(blended[slot], 100)

    def test_ambiguous_two_finger_contact_does_not_latch(self) -> None:
        config = parse_fingertip_contact_config(_config_data())
        state = FingertipContactStateMachine(config)
        ambiguous = {"index": 0.10, "middle": 0.11, "ring": 0.70, "pinky": 0.80}

        self.assertIsNone(state.update(ambiguous, 0.00).pair)
        self.assertIsNone(state.update(ambiguous, 0.20).pair)


if __name__ == "__main__":
    unittest.main()
