from pathlib import Path

import yaml

from manus_l20_retarget.calibration_capture import FullCalibrationSession


def _ergonomics(value: float) -> dict[str, float]:
    keys = (
        "IndexMCPStretch",
        "MiddleMCPStretch",
        "RingMCPStretch",
        "PinkyMCPStretch",
        "IndexPIPStretch",
        "IndexDIPStretch",
        "MiddlePIPStretch",
        "MiddleDIPStretch",
        "RingPIPStretch",
        "RingDIPStretch",
        "PinkyPIPStretch",
        "PinkyDIPStretch",
        "ThumbMCPStretch",
        "ThumbPIPStretch",
        "ThumbDIPStretch",
        "IndexSpread",
        "MiddleSpread",
        "RingSpread",
        "PinkySpread",
    )
    return {key: value + index * 0.01 for index, key in enumerate(keys)}


def test_full_session_saves_four_runtime_calibrations(tmp_path: Path) -> None:
    session = FullCalibrationSession(
        hand="right",
        glove_topic="/unused",
        finger_flexion_ergonomics_output=str(tmp_path / "finger_flexion.yaml"),
        finger_yaw_output=str(tmp_path / "finger_yaw.yaml"),
        thumb_flexion_ergonomics_output=str(tmp_path / "thumb_flexion.yaml"),
        thumb_frame_output=str(tmp_path / "thumb_frame.yaml"),
    )
    session.samples = {
        "natural_open": {"yaw": _ergonomics(0.1), "thumb_segment_vector": [1.0, 0.0, 0.0]},
        "four_finger_fist": {"yaw": _ergonomics(0.8), "thumb_segment_vector": [1.0, 0.0, 0.0]},
        "finger_close": {"yaw": _ergonomics(0.3), "thumb_segment_vector": [1.0, 0.0, 0.0]},
        "finger_spread": {"yaw": _ergonomics(-0.3), "thumb_segment_vector": [1.0, 0.0, 0.0]},
        "thumb_pinky_root_touch": {"yaw": _ergonomics(0.6), "thumb_segment_vector": [0.0, 1.0, 0.0]},
    }

    paths = session.save()

    assert len(paths) == 4
    assert all(path.exists() for path in paths)
    with paths[1].open("r", encoding="utf-8") as handle:
        yaw = yaml.safe_load(handle)
    assert yaw["schema"] == "manus_l20.finger_yaw_calibration.v1"
    assert yaw["ergonomics_keys"] == ["IndexSpread", "MiddleSpread", "RingSpread", "PinkySpread"]
