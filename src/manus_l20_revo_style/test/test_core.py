from pathlib import Path

from manus_l20_revo_style.core import COMMAND_LENGTH, RevoStyleL20Retarget, load_yaml
from manus_l20_revo_style.mujoco_sim import L20MujocoSim


def _config():
    root = Path(__file__).resolve().parents[1]
    return load_yaml(root / "config" / "revo_style_l20_right.yaml")


def _direct_config():
    root = Path(__file__).resolve().parents[1]
    return load_yaml(root / "config" / "four_finger_direct_l20_right.yaml")


def test_open_like_input_stays_near_neutral():
    retarget = RevoStyleL20Retarget(_config())
    _, command = retarget.retarget({}, smooth=False)
    assert len(command) == COMMAND_LENGTH
    assert all(0 <= value <= 255 for value in command)


def test_flexion_input_closes_index_slots():
    cfg = _config()
    retarget = RevoStyleL20Retarget(cfg)
    _, open_command = retarget.retarget({}, smooth=False)
    _, closed_command = retarget.retarget(
        {
            "IndexMCPStretch": 85.0,
            "IndexPIPStretch": 75.0,
            "IndexDIPStretch": 55.0,
            "MiddleSpread": 0.0,
            "IndexSpread": 0.0,
        },
        smooth=False,
    )
    assert closed_command[1] < open_command[1]
    assert closed_command[16] < open_command[16]
    assert closed_command[16] == cfg["l20_command"]["closed_command"][16]


def test_spread_uses_middle_relative_reference():
    retarget = RevoStyleL20Retarget(_config())
    _, command_a = retarget.retarget({"MiddleSpread": 0.0, "IndexSpread": 20.0}, smooth=False)
    _, command_b = retarget.retarget({"MiddleSpread": 20.0, "IndexSpread": 20.0}, smooth=False)
    assert command_a[6] != command_b[6]


def test_direct_ergonomics_range_maps_four_finger_tip():
    cfg = _config()
    retarget = RevoStyleL20Retarget(cfg)
    _, command = retarget.retarget({"IndexPIPStretch": 75.0}, smooth=False)
    assert command[16] == cfg["l20_command"]["closed_command"][16]
    _, command = retarget.retarget({"IndexPIPStretch": 0.0}, smooth=False)
    assert command[16] == cfg["l20_command"]["open_command"][16]


def test_mujoco_command_smoke():
    cfg = _config()
    root = Path(__file__).resolve().parents[3]
    sim = L20MujocoSim(str(root / "src" / "l20_thumb_ik" / "assets" / "mjcf" / "linkerhand_l20_right" / "model.xml"), cfg)
    retarget = RevoStyleL20Retarget(cfg)
    _, command = retarget.retarget({"IndexMCPStretch": 80.0, "IndexPIPStretch": 70.0}, smooth=False)
    sim.set_l20_command(command)
    sim.step(5)
    measured = sim.measured_joint_positions()
    assert "index_mcp_pitch" in measured
    assert measured["index_mcp_pitch"] >= 0.0


def test_four_finger_direct_config_maps_to_255_0():
    retarget = RevoStyleL20Retarget(_direct_config())
    _, command = retarget.retarget(
        {
            "IndexMCPStretch": 85.0,
            "IndexPIPStretch": 75.0,
        },
        smooth=False,
    )
    assert command[1] == 0
    assert command[16] == 0
    assert command[2] == 255
    assert command[17] == 255


def test_four_finger_direct_open_is_255():
    retarget = RevoStyleL20Retarget(_direct_config())
    _, command = retarget.retarget(
        {
            "IndexMCPStretch": 0.0,
            "IndexPIPStretch": 0.0,
            "MiddleMCPStretch": 0.0,
            "MiddlePIPStretch": 0.0,
        },
        smooth=False,
    )
    assert command[1] == 255
    assert command[2] == 255
    assert command[16] == 255
    assert command[17] == 255
