"""LinkerHand SDK controller backend."""

from __future__ import annotations

import importlib.util
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np

from somehand.domain import HandCommand, HandState
from somehand.infrastructure.controllers.adapters import LinkerHandModelAdapter
from somehand.infrastructure.controllers.l20_flexion_calibration import L20FlexionCommandCalibrator
from somehand.infrastructure.controllers.l20_joint_range_mapping import L20JointRangeCommandMapper
from somehand.paths import DEFAULT_LINKERHAND_SDK_PATH


@lru_cache(maxsize=4)
def _load_linkerhand_api_class(sdk_root: str):
    root = Path(sdk_root or DEFAULT_LINKERHAND_SDK_PATH).resolve()
    module_path = root / "LinkerHand" / "linker_hand_api.py"
    if not module_path.exists():
        raise FileNotFoundError(f"LinkerHand API module not found: {module_path}")
    spec = importlib.util.spec_from_file_location("somehand_linkerhand_api", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load LinkerHand API module from: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(spec.name, module)
    spec.loader.exec_module(module)
    return module.LinkerHandApi


class LinkerHandSdkController:
    """Sends retargeted pose targets to LinkerHand SDK and polls state back."""

    def __init__(
        self,
        adapter: LinkerHandModelAdapter,
        *,
        transport: str = "can",
        can_interface: str = "can0",
        modbus_port: str = "None",
        default_speed: list[int] | None = None,
        default_torque: list[int] | None = None,
        sdk_root: str = "",
        real_state_print_every: int = 0,
        real_command_print_every: int = 0,
        real_can_dump: bool = False,
        l20_flexion_calibration: str | None = None,
        l20_flexion_deadzone: float = 0.0,
        l20_flexion_base_follow_tip: float = 0.0,
        l20_flexion_tip_gamma: float = 1.0,
        l20_flexion_close_command_floor: int = 0,
        l20_invert_finger_yaw: bool = False,
        l20_joint_range_mapping: str | None = None,
    ):
        self._adapter = adapter
        self._transport = transport
        self._can_interface = can_interface
        self._modbus_port = modbus_port if transport == "modbus" else "None"
        self._default_speed = list(default_speed or adapter.default_speed)
        self._default_torque = list(default_torque or adapter.default_torque)
        self._sdk_root = sdk_root
        self._real_state_print_every = int(real_state_print_every)
        self._real_command_print_every = int(real_command_print_every)
        self._real_can_dump = bool(real_can_dump)
        self._command_send_count = 0
        self._state_read_count = 0
        self._last_command_pose: list[int] | None = None
        self._skip_l20_state_poll = adapter.family == "L20" and self._real_state_print_every <= 0
        self._l20_invert_finger_yaw = bool(l20_invert_finger_yaw)
        if self._l20_invert_finger_yaw and adapter.family != "L20":
            raise ValueError(f"L20 finger yaw inversion requires family L20, got {adapter.family}")
        self._l20_joint_range_mapper = None
        if l20_joint_range_mapping is not None:
            if adapter.family != "L20":
                raise ValueError(f"L20 joint range mapping requires family L20, got {adapter.family}")
            self._l20_joint_range_mapper = L20JointRangeCommandMapper(
                adapter.hand_model,
                l20_joint_range_mapping,
            )
        self._l20_flexion_calibrator = None
        if l20_flexion_calibration is not None:
            if adapter.family != "L20":
                raise ValueError(f"L20 flexion calibration requires family L20, got {adapter.family}")
            self._l20_flexion_calibrator = L20FlexionCommandCalibrator(
                adapter.hand_model,
                l20_flexion_calibration,
                deadzone=l20_flexion_deadzone,
                base_follow_tip=l20_flexion_base_follow_tip,
                tip_gamma=l20_flexion_tip_gamma,
                close_command_floor=l20_flexion_close_command_floor,
            )
        self._api = None
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        api_cls = _load_linkerhand_api_class(self._sdk_root)
        self._api = api_cls(
            hand_type=self._adapter.hand_side,
            hand_joint=self._adapter.family,
            modbus=self._modbus_port,
            can=self._can_interface,
        )
        sdk_hand = getattr(self._api, "hand", None)
        if self._real_can_dump and sdk_hand is not None:
            sdk_hand.debug_dump_can = True
        if self._default_speed:
            self._api.set_speed(self._default_speed)
        if self._default_torque:
            self._api.set_torque(self._default_torque)
        self._running = True

    def set_command(self, command: HandCommand) -> None:
        if self._api is None:
            raise RuntimeError("LinkerHandSdkController has not been started")
        pose = self._adapter.qpos_to_sdk_range(command.target_qpos_rad)
        if self._l20_joint_range_mapper is not None:
            pose = self._l20_joint_range_mapper.apply(command.target_qpos_rad, pose)
        if self._l20_flexion_calibrator is not None:
            pose = self._l20_flexion_calibrator.apply(command.target_qpos_rad, pose)
        if self._l20_invert_finger_yaw:
            pose = self._invert_l20_finger_yaw(pose)
        self._last_command_pose = list(pose)
        self._command_send_count += 1
        if self._real_command_print_every > 0 and self._command_send_count % self._real_command_print_every == 0:
            print(self._format_real_command_line(command.target_qpos_rad, list(pose)), flush=True)
        self._api.finger_move(pose=pose)

    def get_state(self) -> HandState:
        if self._api is None:
            raise RuntimeError("LinkerHandSdkController has not been started")
        if self._skip_l20_state_poll and self._last_command_pose is not None:
            return HandState(
                measured_qpos_rad=None,
                measured_qvel=None,
                applied_ctrl=np.asarray(self._last_command_pose, dtype=np.float64),
                sim_time=None,
                faults=[],
                contacts=None,
                backend="real",
            )
        pose = self._api.get_state()
        qpos = self._adapter.sdk_range_to_qpos(pose)
        faults = list(self._api.get_fault())
        self._state_read_count += 1
        if self._real_state_print_every > 0 and self._state_read_count % self._real_state_print_every == 0:
            print(self._format_real_state_line(self._last_command_pose, list(pose), faults), flush=True)
        return HandState(
            measured_qpos_rad=qpos,
            measured_qvel=None,
            applied_ctrl=np.asarray(pose, dtype=np.float64),
            sim_time=None,
            faults=faults,
            contacts=None,
            backend="real",
        )

    def _format_real_state_line(self, command: list[int] | None, state: list[int], faults: list[int]) -> str:
        if self._adapter.family == "L20" and command is not None and len(command) == 20 and len(state) == 20:
            return (
                "real returned state(0-255)\n"
                f"  command\n{self._format_l20_groups(command)}\n"
                f"  state\n{self._format_l20_groups(state)}\n"
                f"  faults={faults}"
            )
        return f"real command range={command} real state range={state} faults={faults}"

    def _format_real_command_line(self, target_qpos_rad: np.ndarray, command: list[int]) -> str:
        if self._adapter.family == "L20" and len(command) == 20:
            return (
                f"{self._adapter.format_l20_target_qpos(target_qpos_rad)}\n"
                "real outgoing command(0-255)\n"
                f"{self._format_l20_groups(command)}"
            )
        return f"real target qpos={np.asarray(target_qpos_rad, dtype=np.float64).tolist()} real command range={command}"

    @staticmethod
    def _invert_l20_finger_yaw(command: list[int]) -> list[int]:
        if len(command) != 20:
            raise ValueError(f"L20 finger yaw inversion requires 20 command values, got {len(command)}")
        values = list(command)
        for index in range(6, 10):
            values[index] = 255 - values[index]
        return values

    @staticmethod
    def _format_l20_groups(values: list[int]) -> str:
        return (
            f"  指根弯曲={values[0:5]}\n"
            f"  横摆={values[5:10]}\n"
            f"  横滚={values[10:15]}\n"
            f"  指尖弯曲={values[15:20]}"
        )

    def close(self) -> None:
        if self._api is not None:
            close_can = getattr(self._api, "close_can", None)
            if callable(close_can):
                close_can()
        self._running = False
