"""Experimental fingertip-position IK for L20 retargeting."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from somehand.domain import preprocess_landmarks
from somehand.infrastructure.hand_model import HandModel

_MCP_LANDMARKS = np.asarray([5, 9, 13, 17], dtype=np.int32)
_TARGET_LANDMARKS = np.asarray([3, 4, 7, 8, 11, 12, 15, 16, 19, 20], dtype=np.int32)
_PALM_BODY_NAMES = (
    "index_metacarpals",
    "middle_metacarpals",
    "ring_metacarpals",
    "pinky_metacarpals",
)
_TARGET_POINTS = (
    ("body", "thumb_distal"),
    ("site", "thumb_distal_tip"),
    ("body", "index_distal"),
    ("site", "index_distal_tip"),
    ("body", "middle_distal"),
    ("site", "middle_distal_tip"),
    ("body", "ring_distal"),
    ("site", "ring_distal_tip"),
    ("body", "pinky_distal"),
    ("site", "pinky_distal_tip"),
)
_CONTACT_HUMAN_PAIRS = (
    (4, 8),
    (4, 12),
    (4, 16),
    (4, 20),
)
_CONTACT_ROBOT_SITES = (
    ("thumb_distal_tip", "index_distal_tip"),
    ("thumb_distal_tip", "middle_distal_tip"),
    ("thumb_distal_tip", "ring_distal_tip"),
    ("thumb_distal_tip", "pinky_distal_tip"),
)
_CONTACT_NAMES = ("index", "middle", "ring", "pinky")
_THUMB_ACTIVE_JOINTS = (
    "thumb_cmc_yaw",
    "thumb_cmc_roll",
    "thumb_mcp",
)
_THUMB_SOFT_PITCH_ACTIVE_JOINTS = (
    "thumb_cmc_yaw",
    "thumb_cmc_roll",
    "thumb_cmc_pitch",
    "thumb_mcp",
)
_THUMB_ALL_ACTIVE_JOINTS = (
    "thumb_cmc_yaw",
    "thumb_cmc_roll",
    "thumb_cmc_pitch",
    "thumb_mcp",
    "thumb_dip",
)


@dataclass(frozen=True)
class L20TipIKResult:
    qpos: np.ndarray
    target_positions: np.ndarray
    solved_positions: np.ndarray
    per_tip_error: np.ndarray
    mean_error: float
    max_error: float
    iterations: int


class L20TipIKRetargeter:
    """Retarget MediaPipe landmarks to L20 qpos by matching finger keypoint positions."""

    def __init__(
        self,
        hand_model: HandModel,
        *,
        hand_side: str = "right",
        damping: float = 1e-3,
        posture_weight: float = 2e-3,
        step_scale: float = 0.8,
        max_step: float = 0.18,
        max_iterations: int = 30,
        tolerance: float = 2e-3,
    ):
        self.hand_model = hand_model
        self.hand_side = hand_side
        self.model = hand_model.model
        self.data = hand_model.data
        self.damping = float(damping)
        self.posture_weight = float(posture_weight)
        self.step_scale = float(step_scale)
        self.max_step = float(max_step)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)

        self.target_points = self._resolve_points(_TARGET_POINTS)
        self.palm_body_ids = self._resolve_bodies(_PALM_BODY_NAMES)
        self.active_qpos_ids = hand_model.get_actuator_qpos_indices()
        self.active_dof_ids = self._qpos_ids_to_dof_ids(self.active_qpos_ids)
        self.lower, self.upper = self._active_joint_ranges(self.active_qpos_ids)
        self._last_qpos: np.ndarray | None = None
        self._open_qpos = hand_model.get_qpos()

    def solve(self, landmarks_3d: np.ndarray) -> L20TipIKResult:
        targets = self.build_tip_targets(landmarks_3d)
        qpos = self._initial_qpos()
        iterations = 0
        for iterations in range(1, self.max_iterations + 1):
            self.hand_model.set_qpos(qpos)
            solved = self._target_positions()
            residual = (solved - targets).reshape(-1)
            per_tip_error = np.linalg.norm(solved - targets, axis=1)
            if float(np.max(per_tip_error)) <= self.tolerance:
                break

            jacobian = self._target_jacobian()
            active = qpos[self.active_qpos_ids]
            reference = self._reference_qpos()[self.active_qpos_ids]
            lhs = jacobian.T @ jacobian
            rhs = -(jacobian.T @ residual)
            lhs += (self.damping + self.posture_weight) * np.eye(len(self.active_qpos_ids))
            rhs += -self.posture_weight * (active - reference)
            delta = np.linalg.solve(lhs, rhs)
            norm = float(np.linalg.norm(delta))
            if norm > self.max_step:
                delta *= self.max_step / max(norm, 1e-8)
            active = np.clip(active + self.step_scale * delta, self.lower, self.upper)
            qpos[self.active_qpos_ids] = active
            self.hand_model.apply_mimic_constraints(qpos)

        self.hand_model.set_qpos(qpos)
        solved = self._target_positions()
        errors = np.linalg.norm(solved - targets, axis=1)
        self._last_qpos = qpos.copy()
        return L20TipIKResult(
            qpos=qpos.copy(),
            target_positions=targets,
            solved_positions=solved,
            per_tip_error=errors,
            mean_error=float(np.mean(errors)),
            max_error=float(np.max(errors)),
            iterations=iterations,
        )

    def build_tip_targets(self, landmarks_3d: np.ndarray) -> np.ndarray:
        landmarks = np.asarray(landmarks_3d, dtype=np.float64)
        if landmarks.shape != (21, 3):
            raise ValueError(f"Expected MediaPipe landmarks with shape (21, 3), got {landmarks.shape}")
        processed = preprocess_landmarks(landmarks, hand_side=self.hand_side)
        human_palm = processed[_MCP_LANDMARKS]
        robot_palm = self._palm_positions()
        scale, rotation, translation = _fit_similarity(human_palm, robot_palm)
        return scale * (processed[_TARGET_LANDMARKS] @ rotation) + translation

    def _initial_qpos(self) -> np.ndarray:
        if self._last_qpos is None:
            return self.hand_model.get_qpos()
        return self._last_qpos.copy()

    def _reference_qpos(self) -> np.ndarray:
        if self._last_qpos is None:
            return self._open_qpos
        return self._last_qpos

    def _target_positions(self) -> np.ndarray:
        positions = []
        for obj_type, obj_id in self.target_points:
            if obj_type == "site":
                positions.append(self.data.site_xpos[obj_id].copy())
            else:
                positions.append(self.data.xpos[obj_id].copy())
        return np.asarray(positions)

    def _palm_positions(self) -> np.ndarray:
        return np.asarray([self.data.xpos[body_id].copy() for body_id in self.palm_body_ids])

    def _target_jacobian(self) -> np.ndarray:
        rows: list[np.ndarray] = []
        for obj_type, obj_id in self.target_points:
            jac = np.zeros((3, self.model.nv), dtype=np.float64)
            if obj_type == "site":
                mujoco.mj_jacSite(self.model, self.data, jac, None, int(obj_id))
            else:
                mujoco.mj_jacBody(self.model, self.data, jac, None, int(obj_id))
            rows.append(jac[:, self.active_dof_ids])
        return np.vstack(rows)

    def _resolve_points(self, points: tuple[tuple[str, str], ...]) -> tuple[tuple[str, int], ...]:
        resolved = []
        for obj_type, name in points:
            if obj_type == "site":
                obj_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
            elif obj_type == "body":
                obj_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            else:
                raise ValueError(f"Unsupported L20 IK target type: {obj_type}")
            if obj_id < 0:
                raise ValueError(f"L20 IK {obj_type} target not found: {name}")
            resolved.append((obj_type, int(obj_id)))
        return tuple(resolved)

    def _resolve_sites(self, names: tuple[str, ...]) -> np.ndarray:
        ids = []
        for name in names:
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
            if site_id < 0:
                raise ValueError(f"L20 IK site not found: {name}")
            ids.append(site_id)
        return np.asarray(ids, dtype=np.int32)

    def _resolve_bodies(self, names: tuple[str, ...]) -> np.ndarray:
        ids = []
        for name in names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id < 0:
                raise ValueError(f"L20 IK body not found: {name}")
            ids.append(body_id)
        return np.asarray(ids, dtype=np.int32)

    def _qpos_ids_to_dof_ids(self, qpos_ids: np.ndarray) -> np.ndarray:
        qpos_to_dof: dict[int, int] = {}
        for joint_id in range(self.model.njnt):
            qpos_to_dof[int(self.model.jnt_qposadr[joint_id])] = int(self.model.jnt_dofadr[joint_id])
        return np.asarray([qpos_to_dof[int(qpos_id)] for qpos_id in qpos_ids], dtype=np.int32)

    def _active_joint_ranges(self, qpos_ids: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        qpos_to_range: dict[int, tuple[float, float]] = {}
        for joint_id in range(self.model.njnt):
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            low, high = self.model.jnt_range[joint_id]
            qpos_to_range[qpos_id] = (float(low), float(high))
        lower = np.asarray([qpos_to_range[int(qpos_id)][0] for qpos_id in qpos_ids], dtype=np.float64)
        upper = np.asarray([qpos_to_range[int(qpos_id)][1] for qpos_id in qpos_ids], dtype=np.float64)
        return lower, upper


def _fit_similarity(source: np.ndarray, target: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    covariance = source_zero.T @ target_zero
    u, singular_values, vh = np.linalg.svd(covariance)
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    scale = float(np.sum(singular_values) / max(np.sum(source_zero * source_zero), 1e-8))
    translation = target_center - scale * (source_center @ rotation)
    return scale, rotation, translation


@dataclass(frozen=True)
class L20ContactIKResult:
    qpos: np.ndarray
    base_qpos: np.ndarray
    target_distances: np.ndarray
    solved_distances: np.ndarray
    distance_error: np.ndarray
    mean_error: float
    max_error: float
    iterations: int


class L20ContactIKRefiner:
    """Refine an existing L20 qpos by matching thumb-to-fingertip distances."""

    def __init__(
        self,
        hand_model: HandModel,
        *,
        hand_side: str = "right",
        damping: float = 5e-3,
        posture_weight: float = 5e-2,
        step_scale: float = 0.6,
        max_step: float = 0.08,
        max_iterations: int = 12,
        tolerance: float = 3e-3,
        active_joints: str = "thumb",
        max_delta_from_base: float = 0.12,
        contact_weights: tuple[float, float, float, float] | None = None,
        contact_target_scales: tuple[float, float, float, float] | None = None,
        contact_min_target_distances: tuple[float, float, float, float] | None = None,
        active_contacts: tuple[str, ...] | None = None,
        objective: str = "distance",
        soft_pitch_delta: float = 0.0,
    ):
        self.hand_model = hand_model
        self.hand_side = hand_side
        self.model = hand_model.model
        self.data = hand_model.data
        self.damping = float(damping)
        self.posture_weight = float(posture_weight)
        self.step_scale = float(step_scale)
        self.max_step = float(max_step)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        self.max_delta_from_base = max(0.0, float(max_delta_from_base))
        self.soft_pitch_delta = max(0.0, float(soft_pitch_delta))
        self.objective = self._resolve_objective(objective)
        self.site_pairs = tuple((self._resolve_site(a), self._resolve_site(b)) for a, b in _CONTACT_ROBOT_SITES)
        self.active_contact_indices = self._resolve_active_contacts(active_contacts)
        self.contact_weights = self._normalize_contact_weights(contact_weights)
        self._contact_weight_sqrt = np.sqrt(self.contact_weights)
        self.contact_target_scales = self._normalize_contact_target_scales(contact_target_scales)
        self.contact_min_target_distances = self._normalize_contact_min_target_distances(contact_min_target_distances)
        self.palm_body_ids = self._resolve_bodies(_PALM_BODY_NAMES)
        self.active_qpos_ids = self._select_active_qpos_ids(active_joints)
        self.active_dof_ids = self._qpos_ids_to_dof_ids(self.active_qpos_ids)
        self.lower, self.upper = self._active_joint_ranges(self.active_qpos_ids)
        self._robot_scale = self._open_robot_contact_scale()

    def refine(self, landmarks_3d: np.ndarray, base_qpos: np.ndarray) -> L20ContactIKResult:
        qpos = np.asarray(base_qpos, dtype=np.float64).copy()
        reference = qpos.copy()
        self.hand_model.set_qpos(qpos)
        if self.objective == "vector":
            vector_targets = self.build_contact_vector_targets(landmarks_3d)
            targets = np.linalg.norm(vector_targets, axis=1)
        else:
            vector_targets = None
            targets = self.build_contact_targets(landmarks_3d)
        iterations = 0
        for iterations in range(1, self.max_iterations + 1):
            self.hand_model.set_qpos(qpos)
            solve_rows = self.active_contact_indices
            if self.objective == "vector":
                solved_vectors = self._contact_vectors()
                residual_vectors = (solved_vectors - vector_targets)[:, :2]
                active_vector_error = np.linalg.norm(residual_vectors[solve_rows], axis=1)
                if float(np.max(active_vector_error)) <= self.tolerance:
                    break
                jacobian = self._contact_vector_jacobian()
                component_rows = np.concatenate([np.arange(3 * row, 3 * row + 2) for row in solve_rows])
                component_weights = np.repeat(self._contact_weight_sqrt[solve_rows], 2)
                weighted_jacobian = jacobian[component_rows] * component_weights[:, None]
                weighted_residual = residual_vectors[solve_rows].reshape(-1) * component_weights
            else:
                solved = self._contact_distances()
                residual = solved - targets
                if float(np.max(np.abs(residual[solve_rows]))) <= self.tolerance:
                    break
                jacobian = self._contact_jacobian()
                weighted_jacobian = jacobian[solve_rows] * self._contact_weight_sqrt[solve_rows, None]
                weighted_residual = residual[solve_rows] * self._contact_weight_sqrt[solve_rows]
            active = qpos[self.active_qpos_ids]
            reference_active = reference[self.active_qpos_ids]
            lhs = weighted_jacobian.T @ weighted_jacobian
            rhs = -(weighted_jacobian.T @ weighted_residual)
            lhs += (self.damping + self.posture_weight) * np.eye(len(self.active_qpos_ids))
            rhs += -self.posture_weight * (active - reference_active)
            delta = np.linalg.solve(lhs, rhs)
            norm = float(np.linalg.norm(delta))
            if norm > self.max_step:
                delta *= self.max_step / max(norm, 1e-8)
            max_delta = self._active_max_delta_from_base()
            delta_lower = np.maximum(self.lower, reference_active - max_delta)
            delta_upper = np.minimum(self.upper, reference_active + max_delta)
            active = np.clip(active + self.step_scale * delta, delta_lower, delta_upper)
            qpos[self.active_qpos_ids] = active
            self.hand_model.apply_mimic_constraints(qpos)

        self.hand_model.set_qpos(qpos)
        solved = self._contact_distances()
        error = solved - targets
        return L20ContactIKResult(
            qpos=qpos.copy(),
            base_qpos=np.asarray(base_qpos, dtype=np.float64).copy(),
            target_distances=targets,
            solved_distances=solved,
            distance_error=error,
            mean_error=float(np.mean(np.abs(error))),
            max_error=float(np.max(np.abs(error))),
            iterations=iterations,
        )

    def build_contact_vector_targets(self, landmarks_3d: np.ndarray) -> np.ndarray:
        landmarks = np.asarray(landmarks_3d, dtype=np.float64)
        if landmarks.shape != (21, 3):
            raise ValueError(f"Expected MediaPipe landmarks with shape (21, 3), got {landmarks.shape}")
        processed = preprocess_landmarks(landmarks, hand_side=self.hand_side)
        human_palm = processed[_MCP_LANDMARKS]
        robot_palm = self._palm_positions()
        scale, rotation, _translation = _fit_similarity(human_palm, robot_palm)
        human_vectors = np.asarray(
            [processed[a] - processed[b] for a, b in _CONTACT_HUMAN_PAIRS],
            dtype=np.float64,
        )
        targets = scale * (human_vectors @ rotation) * self.contact_target_scales[:, None]
        return self._apply_min_target_distance_to_vectors(targets)

    def build_contact_targets(self, landmarks_3d: np.ndarray) -> np.ndarray:
        landmarks = np.asarray(landmarks_3d, dtype=np.float64)
        if landmarks.shape != (21, 3):
            raise ValueError(f"Expected MediaPipe landmarks with shape (21, 3), got {landmarks.shape}")
        processed = preprocess_landmarks(landmarks, hand_side=self.hand_side)
        human_scale = max(_human_middle_finger_scale(processed), 1e-8)
        raw = np.asarray(
            [np.linalg.norm(processed[a] - processed[b]) for a, b in _CONTACT_HUMAN_PAIRS],
            dtype=np.float64,
        )
        targets = raw * (self._robot_scale / human_scale) * self.contact_target_scales
        return np.maximum(targets, self.contact_min_target_distances)

    def _apply_min_target_distance_to_vectors(self, targets: np.ndarray) -> np.ndarray:
        adjusted = targets.copy()
        norms = np.linalg.norm(adjusted, axis=1)
        for index, min_distance in enumerate(self.contact_min_target_distances):
            if min_distance <= 0.0 or norms[index] >= min_distance:
                continue
            if norms[index] < 1e-8:
                adjusted[index, 0] = min_distance
            else:
                adjusted[index] *= float(min_distance / norms[index])
        return adjusted

    def _palm_positions(self) -> np.ndarray:
        return np.asarray([self.data.xpos[body_id].copy() for body_id in self.palm_body_ids])

    def _contact_distances(self) -> np.ndarray:
        distances = []
        for site_a, site_b in self.site_pairs:
            distances.append(float(np.linalg.norm(self.data.site_xpos[site_a] - self.data.site_xpos[site_b])))
        return np.asarray(distances, dtype=np.float64)

    def _contact_vectors(self) -> np.ndarray:
        vectors = []
        for site_a, site_b in self.site_pairs:
            vectors.append(self.data.site_xpos[site_a].copy() - self.data.site_xpos[site_b].copy())
        return np.asarray(vectors, dtype=np.float64)

    def _contact_jacobian(self) -> np.ndarray:
        rows = []
        for site_a, site_b in self.site_pairs:
            pos_a = self.data.site_xpos[site_a]
            pos_b = self.data.site_xpos[site_b]
            diff = pos_a - pos_b
            dist = float(np.linalg.norm(diff))
            if dist < 1e-8:
                rows.append(np.zeros(len(self.active_dof_ids), dtype=np.float64))
                continue
            jac_a = np.zeros((3, self.model.nv), dtype=np.float64)
            jac_b = np.zeros((3, self.model.nv), dtype=np.float64)
            mujoco.mj_jacSite(self.model, self.data, jac_a, None, int(site_a))
            mujoco.mj_jacSite(self.model, self.data, jac_b, None, int(site_b))
            rows.append((diff / dist) @ (jac_a[:, self.active_dof_ids] - jac_b[:, self.active_dof_ids]))
        return np.vstack(rows)

    def _contact_vector_jacobian(self) -> np.ndarray:
        rows = []
        for site_a, site_b in self.site_pairs:
            jac_a = np.zeros((3, self.model.nv), dtype=np.float64)
            jac_b = np.zeros((3, self.model.nv), dtype=np.float64)
            mujoco.mj_jacSite(self.model, self.data, jac_a, None, int(site_a))
            mujoco.mj_jacSite(self.model, self.data, jac_b, None, int(site_b))
            rows.append(jac_a[:, self.active_dof_ids] - jac_b[:, self.active_dof_ids])
        return np.vstack(rows)

    def _open_robot_contact_scale(self) -> float:
        self.hand_model.reset()
        positions = [
            self.data.site_xpos[self._resolve_site("middle_distal_tip")],
            self.data.xpos[self._resolve_body("middle_middle")],
            self.data.xpos[self._resolve_body("middle_proximal")],
            self.data.xpos[self._resolve_body("middle_metacarpals")],
        ]
        return float(
            sum(np.linalg.norm(b - a) for a, b in zip(positions[1:], positions[:-1]))
        )

    def _resolve_site(self, name: str) -> int:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
        if site_id < 0:
            raise ValueError(f"L20 contact IK site not found: {name}")
        return int(site_id)

    def _resolve_body(self, name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"L20 contact IK body not found: {name}")
        return int(body_id)

    def _resolve_bodies(self, names: tuple[str, ...]) -> np.ndarray:
        return np.asarray([self._resolve_body(name) for name in names], dtype=np.int32)

    def _resolve_objective(self, objective: str) -> str:
        normalized = objective.strip().lower()
        if normalized not in {"distance", "vector"}:
            raise ValueError(f"Unsupported L20 contact IK objective: {objective}")
        return normalized

    def _select_active_qpos_ids(self, active_joints: str) -> np.ndarray:
        if active_joints == "all":
            return self.hand_model.get_actuator_qpos_indices()
        if active_joints == "thumb":
            active_joint_names = _THUMB_ACTIVE_JOINTS
        elif active_joints == "thumb-soft-pitch":
            active_joint_names = _THUMB_SOFT_PITCH_ACTIVE_JOINTS
        elif active_joints == "thumb-all":
            active_joint_names = _THUMB_ALL_ACTIVE_JOINTS
        else:
            raise ValueError(f"Unsupported L20 contact IK active_joints: {active_joints}")
        joint_index = self.hand_model.get_joint_name_to_qpos_index()
        qpos_ids = []
        for name in active_joint_names:
            if name in joint_index:
                qpos_ids.append(joint_index[name])
        if not qpos_ids:
            raise ValueError("L20 contact IK could not resolve any thumb active joints")
        return np.asarray(qpos_ids, dtype=np.int32)

    def _active_max_delta_from_base(self) -> np.ndarray:
        max_delta = np.full(len(self.active_qpos_ids), self.max_delta_from_base, dtype=np.float64)
        if self.soft_pitch_delta >= self.max_delta_from_base:
            return max_delta
        joint_index = self.hand_model.get_joint_name_to_qpos_index()
        pitch_qpos_id = joint_index.get("thumb_cmc_pitch")
        if pitch_qpos_id is None:
            return max_delta
        for index, qpos_id in enumerate(self.active_qpos_ids):
            if int(qpos_id) == int(pitch_qpos_id):
                max_delta[index] = self.soft_pitch_delta
        return max_delta

    def _resolve_active_contacts(self, active_contacts: tuple[str, ...] | None) -> np.ndarray:
        if active_contacts is None:
            return np.arange(len(self.site_pairs), dtype=np.int32)
        names = []
        for name in active_contacts:
            normalized = name.strip().lower()
            if normalized == "all":
                return np.arange(len(self.site_pairs), dtype=np.int32)
            if normalized not in _CONTACT_NAMES:
                raise ValueError(f"Unsupported L20 contact IK active contact: {name}")
            if normalized not in names:
                names.append(normalized)
        if not names:
            raise ValueError("L20 contact IK active_contacts cannot be empty")
        return np.asarray([_CONTACT_NAMES.index(name) for name in names], dtype=np.int32)

    def _normalize_contact_weights(
        self,
        contact_weights: tuple[float, float, float, float] | None,
    ) -> np.ndarray:
        if contact_weights is None:
            return np.ones(len(self.site_pairs), dtype=np.float64)
        weights = np.asarray(contact_weights, dtype=np.float64)
        if weights.shape != (len(self.site_pairs),):
            raise ValueError(
                f"Expected {len(self.site_pairs)} L20 contact IK weights, got {weights.shape[0]}"
            )
        if np.any(weights <= 0.0):
            raise ValueError("L20 contact IK weights must all be positive")
        return weights

    def _normalize_contact_target_scales(
        self,
        contact_target_scales: tuple[float, float, float, float] | None,
    ) -> np.ndarray:
        if contact_target_scales is None:
            return np.ones(len(self.site_pairs), dtype=np.float64)
        scales = np.asarray(contact_target_scales, dtype=np.float64)
        if scales.shape != (len(self.site_pairs),):
            raise ValueError(
                f"Expected {len(self.site_pairs)} L20 contact target scales, got {scales.shape[0]}"
            )
        if np.any(scales <= 0.0):
            raise ValueError("L20 contact IK target scales must all be positive")
        return scales

    def _normalize_contact_min_target_distances(
        self,
        contact_min_target_distances: tuple[float, float, float, float] | None,
    ) -> np.ndarray:
        if contact_min_target_distances is None:
            return np.zeros(len(self.site_pairs), dtype=np.float64)
        distances = np.asarray(contact_min_target_distances, dtype=np.float64)
        if distances.shape != (len(self.site_pairs),):
            raise ValueError(
                f"Expected {len(self.site_pairs)} L20 contact min target distances, got {distances.shape[0]}"
            )
        if np.any(distances < 0.0):
            raise ValueError("L20 contact IK min target distances must be >= 0")
        return distances

    def _qpos_ids_to_dof_ids(self, qpos_ids: np.ndarray) -> np.ndarray:
        qpos_to_dof: dict[int, int] = {}
        for joint_id in range(self.model.njnt):
            qpos_to_dof[int(self.model.jnt_qposadr[joint_id])] = int(self.model.jnt_dofadr[joint_id])
        return np.asarray([qpos_to_dof[int(qpos_id)] for qpos_id in qpos_ids], dtype=np.int32)

    def _active_joint_ranges(self, qpos_ids: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        qpos_to_range: dict[int, tuple[float, float]] = {}
        for joint_id in range(self.model.njnt):
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            low, high = self.model.jnt_range[joint_id]
            qpos_to_range[qpos_id] = (float(low), float(high))
        lower = np.asarray([qpos_to_range[int(qpos_id)][0] for qpos_id in qpos_ids], dtype=np.float64)
        upper = np.asarray([qpos_to_range[int(qpos_id)][1] for qpos_id in qpos_ids], dtype=np.float64)
        return lower, upper


def _human_middle_finger_scale(processed_landmarks: np.ndarray) -> float:
    chain = (9, 10, 11, 12)
    return float(
        sum(
            np.linalg.norm(processed_landmarks[b] - processed_landmarks[a])
            for a, b in zip(chain, chain[1:])
        )
    )
