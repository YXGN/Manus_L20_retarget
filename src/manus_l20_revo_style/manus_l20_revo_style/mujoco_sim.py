from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import mujoco
import numpy as np

from .core import clamp, finite_or


def _joint_type_value(value) -> int:
    return int(value.value) if hasattr(value, "value") else int(value)


def _poly(polycoef: np.ndarray, x: float) -> float:
    return sum(float(coefficient) * (x ** power) for power, coefficient in enumerate(polycoef))


@dataclass(slots=True)
class MimicJoint:
    qpos_id: int
    source_qpos_id: int
    polycoef: np.ndarray


class L20MujocoSim:
    def __init__(self, mjcf_path: str, config: Mapping[str, object]):
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.config = config
        self.joint_to_qpos = self._joint_name_to_qpos_index()
        self.actuator_qpos = self._actuator_qpos_indices()
        self.mimic_joints = self._collect_mimic_joints()
        self._stabilize_model()
        self.target_qpos = self.data.qpos.copy()
        self.apply_mimic(self.target_qpos)
        self.set_qpos(self.target_qpos)

    def set_l20_command(self, command: list[int]) -> np.ndarray:
        target = self.command_to_qpos(command, self.data.qpos)
        self.set_target_qpos(target)
        return target.copy()

    def set_target_qpos(self, qpos: np.ndarray) -> None:
        self.target_qpos = np.asarray(qpos, dtype=np.float64).copy()
        self.apply_mimic(self.target_qpos)

    def command_to_qpos(self, command: list[int], base_qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(base_qpos, dtype=np.float64).copy()
        self._set(qpos, "thumb_cmc_pitch", self._slot_amount(command, 0, "thumb_mcp"), 0.0, 0.72)
        self._set(qpos, "thumb_cmc_roll", self._slot_amount(command, 5, "thumb_roll"), 0.0, 1.10)
        self._set(qpos, "thumb_cmc_yaw", self._slot_amount(command, 10, "thumb_yaw"), 0.0, 1.20)
        self._set(qpos, "thumb_mcp", self._slot_amount(command, 15, "thumb_tip"), 0.0, 1.05)

        self._set(qpos, "index_mcp_roll", self._slot_amount(command, 6, "index_spread"), -0.17, 0.17)
        self._set(qpos, "index_mcp_pitch", self._slot_amount(command, 1, "index_mcp"), 0.0, 1.35)
        self._set(qpos, "index_pip", self._slot_amount(command, 16, "index_tip"), 0.0, 1.45)

        self._set(qpos, "middle_mcp_roll", self._slot_amount(command, 7, "middle_spread"), -0.12, 0.12)
        self._set(qpos, "middle_mcp_pitch", self._slot_amount(command, 2, "middle_mcp"), 0.0, 1.35)
        self._set(qpos, "middle_pip", self._slot_amount(command, 17, "middle_tip"), 0.0, 1.45)

        self._set(qpos, "ring_mcp_roll", self._slot_amount(command, 8, "ring_spread"), -0.17, 0.17)
        self._set(qpos, "ring_mcp_pitch", self._slot_amount(command, 3, "ring_mcp"), 0.0, 1.35)
        self._set(qpos, "ring_pip", self._slot_amount(command, 18, "ring_tip"), 0.0, 1.45)

        self._set(qpos, "pinky_mcp_roll", self._slot_amount(command, 9, "pinky_spread"), -0.17, 0.17)
        self._set(qpos, "pinky_mcp_pitch", self._slot_amount(command, 4, "pinky_mcp"), 0.0, 1.35)
        self._set(qpos, "pinky_pip", self._slot_amount(command, 19, "pinky_tip"), 0.0, 1.45)
        self.apply_mimic(qpos)
        return qpos

    def step(self, steps: int = 1) -> None:
        ctrl = self.target_qpos[self.actuator_qpos].copy()
        np.clip(ctrl, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1], out=ctrl)
        self.data.ctrl[:] = ctrl
        for _ in range(max(1, int(steps))):
            mujoco.mj_step(self.model, self.data)

    def measured_joint_positions(self) -> dict[str, float]:
        return {name: float(self.data.qpos[index]) for name, index in self.joint_to_qpos.items()}

    def target_joint_positions(self) -> dict[str, float]:
        return {name: float(self.target_qpos[index]) for name, index in self.joint_to_qpos.items()}

    def set_qpos(self, qpos: np.ndarray) -> None:
        self.data.qpos[:] = np.asarray(qpos, dtype=np.float64)
        self.apply_mimic(self.data.qpos)
        mujoco.mj_forward(self.model, self.data)

    def apply_mimic(self, qpos: np.ndarray) -> np.ndarray:
        for mimic in self.mimic_joints:
            qpos[mimic.qpos_id] = _poly(mimic.polycoef, float(qpos[mimic.source_qpos_id]))
        return qpos

    def _slot_amount(self, command: list[int], slot: int, range_name: str) -> float:
        command_cfg = self.config.get("l20_command", {})
        open_command = command_cfg.get("open_command", [255] * 20)
        closed_command = command_cfg.get("closed_command", [0] * 20)
        if len(command) <= slot:
            return 0.0
        low = finite_or(open_command[slot], 255.0)
        high = finite_or(closed_command[slot], 0.0)
        denom = high - low
        if abs(denom) < 1e-9:
            return 0.0
        return clamp((finite_or(command[slot], low) - low) / denom, 0.0, 1.0)

    def _set(self, qpos: np.ndarray, joint: str, amount: float, low: float, high: float) -> None:
        index = self.joint_to_qpos.get(joint)
        if index is None:
            return
        qpos[index] = low + amount * (high - low)

    def _joint_name_to_qpos_index(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if name:
                out[name] = int(self.model.jnt_qposadr[joint_id])
        return out

    def _actuator_qpos_indices(self) -> np.ndarray:
        indices: list[int] = []
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])
            indices.append(int(self.model.jnt_qposadr[joint_id]))
        return np.asarray(indices, dtype=np.int32)

    def _actuator_dof_indices(self) -> np.ndarray:
        indices: list[int] = []
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id][0])
            indices.append(int(self.model.jnt_dofadr[joint_id]))
        return np.asarray(indices, dtype=np.int32)

    def _collect_mimic_joints(self) -> list[MimicJoint]:
        mimic_joints: list[MimicJoint] = []
        joint_eq_type = _joint_type_value(mujoco.mjtEq.mjEQ_JOINT)
        for equality_id in range(self.model.neq):
            if int(self.model.eq_type[equality_id]) != joint_eq_type:
                continue
            mimic_joint_id = int(self.model.eq_obj1id[equality_id])
            source_joint_id = int(self.model.eq_obj2id[equality_id])
            if mimic_joint_id < 0 or source_joint_id < 0:
                continue
            mimic_joints.append(
                MimicJoint(
                    qpos_id=int(self.model.jnt_qposadr[mimic_joint_id]),
                    source_qpos_id=int(self.model.jnt_qposadr[source_joint_id]),
                    polycoef=np.asarray(self.model.eq_data[equality_id][:5], dtype=np.float64),
                )
            )
        return mimic_joints

    def _stabilize_model(self) -> None:
        dofs = self._actuator_dof_indices()
        if dofs.size:
            self.model.dof_damping[dofs] = np.maximum(self.model.dof_damping[dofs], 0.75)
        mimic_dofs = np.asarray(
            [
                int(self.model.jnt_dofadr[int(self.model.eq_obj1id[equality_id])])
                for equality_id in range(self.model.neq)
                if int(self.model.eq_type[equality_id]) == _joint_type_value(mujoco.mjtEq.mjEQ_JOINT)
                and int(self.model.eq_obj1id[equality_id]) >= 0
            ],
            dtype=np.int32,
        )
        if mimic_dofs.size:
            self.model.dof_damping[mimic_dofs] = np.maximum(self.model.dof_damping[mimic_dofs], 0.35)
            self.model.dof_armature[mimic_dofs] = np.maximum(self.model.dof_armature[mimic_dofs], 0.01)
            self.model.dof_frictionloss[mimic_dofs] = np.maximum(self.model.dof_frictionloss[mimic_dofs], 0.01)
        kp = np.minimum(self.model.actuator_gainprm[:, 0], 4.0)
        self.model.actuator_gainprm[:, 0] = kp
        self.model.actuator_biasprm[:, 1] = -kp
        joint_eq_type = _joint_type_value(mujoco.mjtEq.mjEQ_JOINT)
        equality_ids = [
            equality_id
            for equality_id in range(self.model.neq)
            if int(self.model.eq_type[equality_id]) == joint_eq_type
        ]
        if equality_ids:
            self.model.eq_solref[equality_ids, :] = np.asarray([0.005, 1.5], dtype=np.float64)
            self.model.eq_solimp[equality_ids, :] = np.asarray([0.95, 0.99, 0.0005, 0.5, 2.0], dtype=np.float64)


class PassiveViewer:
    def __init__(self, sim: L20MujocoSim):
        import mujoco.viewer

        self._viewer = mujoco.viewer.launch_passive(sim.model, sim.data)

    @property
    def is_running(self) -> bool:
        return bool(self._viewer.is_running())

    def sync(self) -> None:
        self._viewer.sync()

    def close(self) -> None:
        self._viewer.close()
