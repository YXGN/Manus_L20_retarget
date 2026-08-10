from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from manus_l20_retarget.contact_semantics import (
    CONTACT_STATE_ACTIVE,
    CONTACT_STATE_HOLD,
    CONTACT_STATE_RECLOSE,
    CONTACT_STATE_RELEASE,
    ContactCommandSmoother,
    ContactSlotPlanner,
    FINGERTIP_CONTACT_SLOTS,
    FingertipContactController,
    FingertipContactFeatureExtractor,
    FingertipContactPhaseConfig,
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
                "max_command_delta": 255,
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
            "full_takeover_progress": 1.00,
            "firm_contact_enter_activation": 1.00,
            "release_start_progress_delta": 0.06,
            "reclose_start_progress_delta": 0.02,
            "activation_rise_sec": 0.10,
            "activation_release_sec": 0.10,
            "distance_filter_alpha": 1.0,
            "command_slew_per_cycle": 255,
            "phase_switch_sec": 0.08,
        },
        "contacts": contacts,
    }


def _finger(mcp, tip):
    return SimpleNamespace(mcp=np.asarray(mcp, dtype=np.float64), tip=np.asarray(tip, dtype=np.float64))


def _skeleton(index_tip: float):
    return SimpleNamespace(
        thumb=_finger((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        index=_finger((0.0, 0.0, 0.0), (index_tip, 0.0, 0.0)),
        middle=_finger((0.3, 0.0, 0.0), (0.25, 0.5, 0.0)),
        ring=_finger((0.7, 0.0, 0.0), (0.25, 0.75, 0.0)),
        pinky=_finger((1.0, 0.0, 0.0), (0.25, 1.0, 0.0)),
    )


class FingertipContactSemanticsTest(unittest.TestCase):
    def test_tip_distances_are_normalized_by_palm_width(self) -> None:
        ratios = thumb_fingertip_distance_ratios(_skeleton(0.25))

        self.assertAlmostEqual(ratios["index"], 0.25)
        self.assertNotIn("middle", ratios)
        self.assertNotIn("ring", ratios)
        self.assertNotIn("pinky", ratios)

    def test_config_parses_v2_filter_and_smoother_runtime(self) -> None:
        data = _config_data()
        data["runtime"]["distance_filter_alpha"] = 0.75
        data["runtime"]["command_slew_per_cycle"] = 32
        data["runtime"]["phase_switch_sec"] = 0.12

        config = parse_fingertip_contact_config(data)

        self.assertAlmostEqual(config.distance_filter_alpha, 0.75)
        self.assertEqual(config.command_slew_per_cycle, 32)
        self.assertAlmostEqual(config.phase_switch_sec, 0.12)

    def test_feature_extractor_filters_distance_before_progress_and_velocity(self) -> None:
        data = _config_data()
        data["runtime"].update({"distance_filter_alpha": 0.5})
        config = parse_fingertip_contact_config(data)
        extractor = FingertipContactFeatureExtractor(config)

        extractor.update({"index": 0.70}, 0.00)
        first = extractor.update({"index": 0.10}, 0.01)["index"]
        second = extractor.update({"index": 0.10}, 0.02)["index"]

        self.assertAlmostEqual(first.distance_filtered, 0.40)
        self.assertAlmostEqual(second.distance_filtered, 0.25)
        self.assertLess(first.velocity, 0.0)
        self.assertAlmostEqual(second.progress, (0.70 - 0.25) / (0.70 - 0.10))

    def test_state_machine_latches_then_releases_with_named_state(self) -> None:
        config = parse_fingertip_contact_config(_config_data())
        state = FingertipContactStateMachine(config)
        ratios = {"index": 0.10, "middle": 0.70, "ring": 0.70, "pinky": 0.70}

        self.assertIsNone(state.update(ratios, 0.00).pair)
        self.assertIsNone(state.update(ratios, 0.05).pair)
        active = state.update(ratios, 0.11)
        self.assertEqual(active.pair, "index")
        self.assertEqual(active.event, "activated:index")
        self.assertEqual(active.state, CONTACT_STATE_HOLD)
        self.assertEqual(active.phase, 0.0)
        self.assertGreater(active.activation, 0.0)

        releasing = state.update({**ratios, "index": 0.50}, 0.14)
        self.assertEqual(releasing.pair, "index")
        self.assertEqual(releasing.state, CONTACT_STATE_RELEASE)
        self.assertGreater(releasing.phase, 0.0)
        self.assertLess(releasing.phase, 1.0)

        handoff = state.update({**ratios, "index": 0.70}, 0.20)
        self.assertIsNone(handoff.event)
        self.assertEqual(handoff.pair, "index")

        released = state.update({**ratios, "index": 0.70}, 0.26)
        self.assertEqual(released.event, "released:index")
        self.assertIsNone(released.pair)

    def test_release_mode_allows_intentional_reclose_before_full_open(self) -> None:
        data = _config_data()
        data["runtime"].update(
            {
                "min_hold_sec": 0.0,
                "takeover_start_progress": 0.50,
                "full_takeover_progress": 0.75,
                "firm_contact_enter_activation": 0.90,
                "activation_rise_sec": 0.0,
                "activation_release_sec": 0.0,
            }
        )
        state = FingertipContactStateMachine(parse_fingertip_contact_config(data))
        ratios = {"index": 0.10, "middle": 0.70, "ring": 0.70, "pinky": 0.70}

        state.update(ratios, 0.00)
        full = state.update(ratios, 0.01)
        self.assertEqual(full.activation, 1.0)

        releasing = state.update({**ratios, "index": 0.50}, 0.02)
        self.assertEqual(releasing.state, CONTACT_STATE_RELEASE)
        self.assertLess(releasing.activation, 1.0)

        reclosing = state.update({**ratios, "index": 0.46}, 0.03)
        self.assertEqual(reclosing.state, CONTACT_STATE_RECLOSE)
        self.assertGreater(reclosing.activation, releasing.activation)
        self.assertLess(reclosing.phase_target, releasing.phase_target)

    def test_release_phase_target_is_small_near_closed_contact(self) -> None:
        data = _config_data()
        data["runtime"].update(
            {
                "min_hold_sec": 0.0,
                "takeover_start_progress": 0.50,
                "full_takeover_progress": 0.75,
                "activation_rise_sec": 0.0,
                "activation_release_sec": 0.0,
                "phase_switch_sec": 0.0,
            }
        )
        state = FingertipContactStateMachine(parse_fingertip_contact_config(data))
        ratios = {"index": 0.10, "middle": 0.70, "ring": 0.70, "pinky": 0.70}

        state.update(ratios, 0.00)
        state.update(ratios, 0.01)
        near_closed_release = state.update({**ratios, "index": 0.142}, 0.02)

        # progress is still about 0.93.  It may be classified as release
        # intent, but release phase must stay small instead of hard-switching
        # to 1.0 and dropping flexion slot gains.
        self.assertEqual(near_closed_release.state, CONTACT_STATE_RELEASE)
        self.assertGreater(near_closed_release.activation, 0.90)
        self.assertLess(near_closed_release.phase_target, 0.10)
        self.assertLess(near_closed_release.phase, 0.10)

    def test_non_index_contact_does_not_latch(self) -> None:
        config = parse_fingertip_contact_config(_config_data())
        state = FingertipContactStateMachine(config)
        middle_only = {"index": 0.70, "middle": 0.10, "ring": 0.70, "pinky": 0.80}

        self.assertIsNone(state.update(middle_only, 0.00).pair)
        self.assertIsNone(state.update(middle_only, 0.20).pair)

    def test_blend_changes_only_selected_slots_and_respects_delta_limit(self) -> None:
        data = _config_data()
        data["contacts"]["index"]["robot"]["max_command_delta"] = 20
        profile = parse_fingertip_contact_config(data).profiles["index"]

        blended = blend_fingertip_contact_command([100] * 20, profile, 0.5)

        for slot in profile.override_slots:
            self.assertEqual(blended[slot], 90)
        for slot in set(range(20)) - set(profile.override_slots):
            self.assertEqual(blended[slot], 100)

    def test_slot_planner_advances_orientation_before_flexion_when_closing(self) -> None:
        profile = parse_fingertip_contact_config(_config_data()).profiles["index"]
        planner = ContactSlotPlanner(
            FingertipContactPhaseConfig(close_orientation_completion=0.25, close_flexion_start=0.40),
            lambda slot: 255,
        )

        slot_activations, slot_targets = planner.plan(profile, 0.25, CONTACT_STATE_ACTIVE, 0.0)

        self.assertEqual(slot_targets, {})
        self.assertEqual(slot_activations[5], 1.0)
        self.assertEqual(slot_activations[10], 1.0)
        for slot in (0, 1, 15, 16):
            self.assertEqual(slot_activations[slot], 0.0)

    def test_slot_planner_phase_blends_close_and_release_curves(self) -> None:
        profile = parse_fingertip_contact_config(_config_data()).profiles["index"]
        planner = ContactSlotPlanner(
            FingertipContactPhaseConfig(
                release_flexion_open_completion=0.65,
                release_orientation_gamma=2.5,
            ),
            lambda slot: 255,
        )

        close_slots, close_targets = planner.plan(profile, 0.82, CONTACT_STATE_ACTIVE, 0.0)
        release_slots, release_targets = planner.plan(profile, 0.82, CONTACT_STATE_RELEASE, 1.0)
        blended_slots, blended_targets = planner.plan(profile, 0.82, CONTACT_STATE_RELEASE, 0.5)

        for slot in (0, 1, 15, 16):
            self.assertEqual(release_targets[slot], 255)
            self.assertEqual(blended_targets[slot], 255)
            self.assertGreater(blended_slots[slot], min(close_slots[slot], release_slots[slot]))
            self.assertLess(blended_slots[slot], max(close_slots[slot], release_slots[slot]))
        for slot in (5, 10):
            self.assertGreater(close_slots[slot], release_slots[slot])
            self.assertGreater(blended_slots[slot], release_slots[slot])
            self.assertLess(blended_slots[slot], close_slots[slot])

    def test_command_smoother_slews_each_semantic_slot(self) -> None:
        smoother = ContactCommandSmoother(16)
        base = [100] * 20
        desired = [100] * 20
        for slot in FINGERTIP_CONTACT_SLOTS["index"]:
            desired[slot] = 10

        first = smoother.apply(base, desired, FINGERTIP_CONTACT_SLOTS["index"])
        second = smoother.apply(base, desired, FINGERTIP_CONTACT_SLOTS["index"])

        for slot in FINGERTIP_CONTACT_SLOTS["index"]:
            self.assertEqual(first[slot], 84)
            self.assertEqual(second[slot], 68)

    def test_controller_applies_semantic_command_and_debug_fields(self) -> None:
        data = _config_data()
        data["runtime"].update(
            {
                "min_hold_sec": 0.0,
                "activation_rise_sec": 0.0,
                "activation_release_sec": 0.0,
                "command_slew_per_cycle": 255,
            }
        )
        config = parse_fingertip_contact_config(data)
        controller = FingertipContactController(
            config,
            FingertipContactPhaseConfig(close_orientation_completion=0.25, close_flexion_start=0.40),
            lambda slot: 255,
        )

        controller.apply([100] * 20, _skeleton(0.10), 0.00)
        output, decision = controller.apply([100] * 20, _skeleton(0.10), 0.01)

        self.assertEqual(decision.pair, "index")
        self.assertEqual(decision.state, CONTACT_STATE_HOLD)
        self.assertEqual(decision.phase, 0.0)
        self.assertTrue(decision.slot_activations)
        self.assertEqual(decision.command_slots[5], output[5])
        self.assertEqual(output[5], 10)
        self.assertEqual(output[10], 10)


if __name__ == "__main__":
    unittest.main()
