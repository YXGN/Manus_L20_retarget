from __future__ import annotations

import math

import numpy as np
from manus_ros2_msgs.msg import ManusRawNode

FINGER_CHAINS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
JOINT_ORDER = ("MCP", "PIP", "IP", "DIP", "TIP")
FINGER_SLOTS = {
    "Thumb": (1, 2, 3, 4),
    "Index": (5, 6, 7, 8),
    "Middle": (9, 10, 11, 12),
    "Ring": (13, 14, 15, 16),
    "Pinky": (17, 18, 19, 20),
}

TRANSFORMS: dict[str, np.ndarray] = {
    "identity": np.eye(3, dtype=np.float64),
    # Same transform used by somehand's PICO adapter. MANUS and PICO are not the
    # same device, but this is a useful candidate because both are tracking-space
    # hand skeletons entering a MediaPipe-style retargeting pipeline.
    "pico_native_to_rh": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    "manus_y_up_to_rh": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    ),
}


def manus_raw_nodes_to_mediapipe_landmarks(
    raw_nodes: list[ManusRawNode],
    *,
    transform: str = "pico_native_to_rh",
    wrist_mode: str = "estimate",
    distal_mode: str = "dip",
) -> np.ndarray:
    if distal_mode not in ("dip", "ip"):
        raise ValueError(f"unknown MANUS distal_mode: {distal_mode}")
    chains = {_chain: _ordered_chain(raw_nodes, _chain) for _chain in FINGER_CHAINS}
    missing = [chain for chain, points in chains.items() if len(points) < 3]
    if missing:
        raise ValueError(f"MANUS raw_nodes missing usable finger chains: {missing}")

    landmarks = np.zeros((21, 3), dtype=np.float64)
    for chain, slots in FINGER_SLOTS.items():
        points = chains[chain]
        selected = _select_four_landmarks(points, chain=chain, distal_mode=distal_mode)
        for slot, point in zip(slots, selected):
            landmarks[slot] = point

    landmarks[0] = _estimate_wrist(landmarks, mode=wrist_mode)
    matrix = TRANSFORMS.get(transform)
    if matrix is None:
        raise ValueError(f"unknown MANUS landmark transform: {transform}")
    return landmarks @ matrix.T


def summarize_landmarks(landmarks: np.ndarray) -> dict[str, object]:
    values = np.asarray(landmarks, dtype=np.float64)
    finger_lengths: dict[str, float] = {}
    root_flexion_rad: dict[str, float] = {}
    tip_flexion_rad: dict[str, float] = {}
    finger_yaw_rad = _finger_yaw_rad(values, source="tip")
    thumb_pose = _thumb_pose_features(values)
    for chain, slots in FINGER_SLOTS.items():
        indices = [0, *slots] if chain != "Thumb" else list(slots)
        length = 0.0
        for a, b in zip(indices[:-1], indices[1:]):
            length += float(np.linalg.norm(values[b] - values[a]))
        finger_lengths[chain] = round(length, 5)
        root_flexion_rad[chain] = round(_joint_flexion_rad(values[slots[0]], values[slots[1]], values[slots[2]]), 5)
        tip_flexion_rad[chain] = round(_joint_flexion_rad(values[slots[1]], values[slots[2]], values[slots[3]]), 5)

    palm_width = float(np.linalg.norm(values[5] - values[17]))
    middle_length = float(np.linalg.norm(values[9] - values[12]))
    return {
        "palm_width": round(palm_width, 5),
        "middle_mcp_to_tip": round(middle_length, 5),
        "finger_lengths": finger_lengths,
        "root_flexion_rad": root_flexion_rad,
        "tip_flexion_rad": tip_flexion_rad,
        "finger_yaw_rad": {
            chain: round(float(angle), 5)
            for chain, angle in zip(("Index", "Middle", "Ring", "Pinky"), finger_yaw_rad)
        },
        "thumb_pose": {
            key: round(float(value), 5)
            for key, value in thumb_pose.items()
        },
        "landmarks": np.round(values, 5).tolist(),
    }


def _ordered_chain(raw_nodes: list[ManusRawNode], chain: str) -> list[np.ndarray]:
    by_joint: dict[str, ManusRawNode] = {}
    for node in raw_nodes:
        if str(node.chain_type) == chain:
            by_joint.setdefault(str(node.joint_type), node)
    ordered_nodes = [by_joint[joint] for joint in JOINT_ORDER if joint in by_joint]
    return [_node_position(node) for node in ordered_nodes]


def _select_four_landmarks(points: list[np.ndarray], *, chain: str, distal_mode: str) -> list[np.ndarray]:
    # MANUS usually provides MCP, PIP, IP, DIP, TIP. MediaPipe wants four points
    # per finger. Test whether MANUS IP or DIP is a better MediaPipe DIP proxy.
    if len(points) >= 5:
        if chain != "Thumb" and distal_mode == "ip":
            return [points[0], points[1], points[2], points[4]]
        return [points[0], points[1], points[3], points[4]]
    if len(points) == 4:
        return [points[0], points[1], points[2], points[3]]
    return [points[0], points[1], points[-1], points[-1]]


def _node_position(node: ManusRawNode) -> np.ndarray:
    position = node.pose.position
    return np.asarray([position.x, position.y, position.z], dtype=np.float64)


def _estimate_wrist(landmarks: np.ndarray, *, mode: str) -> np.ndarray:
    mcp_indices = [5, 9, 13, 17]
    palm_center = np.mean(landmarks[mcp_indices], axis=0)
    if mode == "palm_center":
        return palm_center

    # Keep the wrist/palm frame stable across finger flexion. Using MCP->TIP
    # makes the estimated wrist drift during a fist because the fingertips fold
    # back toward the palm. The proximal MCP->PIP directions remain much more
    # stable and are closer to the palm-forward direction needed by somehand's
    # MediaPipe-style preprocessing.
    root_pairs = [(5, 6), (9, 10), (13, 14), (17, 18)]
    root_dirs = []
    for mcp_index, pip_index in root_pairs:
        vector = landmarks[pip_index] - landmarks[mcp_index]
        norm = float(np.linalg.norm(vector))
        if norm > 1e-8:
            root_dirs.append(vector / norm)
    if root_dirs:
        finger_dir = np.mean(root_dirs, axis=0)
        norm = float(np.linalg.norm(finger_dir))
        if norm > 1e-8:
            finger_dir = finger_dir / norm
        else:
            finger_dir = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        finger_dir = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    palm_width = float(np.linalg.norm(landmarks[5] - landmarks[17]))
    wrist_offset = max(palm_width * 1.15, 0.045)
    return palm_center - finger_dir * wrist_offset


def _joint_flexion_rad(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    first = a - b
    second = c - b
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-8:
        return 0.0
    cosine = float(np.dot(first, second) / denominator)
    interior_angle = math.acos(max(-1.0, min(1.0, cosine)))
    return math.pi - interior_angle


def _finger_yaw_rad(landmarks: np.ndarray, *, source: str = "tip") -> np.ndarray:
    source_pairs_by_name = {
        "pip": ((5, 6), (9, 10), (13, 14), (17, 18)),
        "dip": ((5, 7), (9, 11), (13, 15), (17, 19)),
        "tip": ((5, 8), (9, 12), (13, 16), (17, 20)),
    }
    source_pairs = source_pairs_by_name.get(source)
    if source_pairs is None:
        raise ValueError(f"unknown finger yaw source: {source}")
    frame = _palm_frame(landmarks)
    if frame is None:
        return np.zeros(4, dtype=np.float64)
    lateral, forward, _ = frame

    angles = []
    for mcp_index, target_index in source_pairs:
        vector = landmarks[target_index] - landmarks[mcp_index]
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            angles.append(0.0)
            continue
        direction = vector / norm
        angles.append(math.atan2(float(np.dot(direction, lateral)), float(np.dot(direction, forward))))
    while len(angles) < 4:
        angles.append(0.0)
    return np.asarray(angles[:4], dtype=np.float64)


def _finger_mcp_orientation_yaw_rad(raw_nodes: list[ManusRawNode]) -> np.ndarray:
    return _finger_joint_orientation_yaw_rad(raw_nodes, joint_type="MCP")


def _finger_joint_orientation_yaw_rad(raw_nodes: list[ManusRawNode], *, joint_type: str) -> np.ndarray:
    hand_orientation = None
    node_by_chain: dict[str, ManusRawNode] = {}
    joint_type = str(joint_type)
    for node in raw_nodes:
        chain = str(node.chain_type)
        joint = str(node.joint_type)
        if chain == "Hand":
            hand_orientation = _node_quaternion(node)
        elif chain in ("Index", "Middle", "Ring", "Pinky") and joint == joint_type:
            node_by_chain.setdefault(chain, node)

    hand_inverse = _quaternion_inverse(hand_orientation) if hand_orientation is not None else None
    angles: list[float] = []
    for chain in ("Index", "Middle", "Ring", "Pinky"):
        node = node_by_chain.get(chain)
        if node is None:
            angles.append(0.0)
            continue
        orientation = _node_quaternion(node)
        relative = _quaternion_multiply(hand_inverse, orientation) if hand_inverse is not None else orientation
        matrix = _quaternion_to_matrix(relative)
        angles.append(math.atan2(float(matrix[0, 2]), float(matrix[2, 2])))
    return np.asarray(angles, dtype=np.float64)


def _node_quaternion(node: ManusRawNode) -> np.ndarray:
    orientation = node.pose.orientation
    quat = np.asarray([orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-8:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def _quaternion_inverse(quat: np.ndarray) -> np.ndarray:
    return np.asarray([-quat[0], -quat[1], -quat[2], quat[3]], dtype=np.float64)


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def _quaternion_to_matrix(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _thumb_pose_features(landmarks: np.ndarray) -> dict[str, float]:
    frame = _palm_frame(landmarks)
    if frame is None:
        return {
            "yaw_rad": 0.0,
            "roll_rad": 0.0,
            "thumb_index_tip_distance": float(np.linalg.norm(landmarks[4] - landmarks[8])),
        }
    lateral, forward, normal = frame
    vector = landmarks[4] - landmarks[1]
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-8:
        yaw = 0.0
        roll = 0.0
    else:
        direction = vector / norm
        lateral_component = float(np.dot(direction, lateral))
        forward_component = float(np.dot(direction, forward))
        normal_component = float(np.dot(direction, normal))
        yaw = math.atan2(lateral_component, forward_component)
        roll = math.atan2(normal_component, math.hypot(lateral_component, forward_component))
    return {
        "yaw_rad": yaw,
        "roll_rad": roll,
        "thumb_index_tip_distance": float(np.linalg.norm(landmarks[4] - landmarks[8])),
    }


def _palm_frame(landmarks: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    lateral = landmarks[17] - landmarks[5]
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm <= 1e-8:
        return None
    lateral = lateral / lateral_norm

    roots = []
    for mcp_index, pip_index in ((5, 6), (9, 10), (13, 14), (17, 18)):
        vector = landmarks[pip_index] - landmarks[mcp_index]
        norm = float(np.linalg.norm(vector))
        if norm > 1e-8:
            roots.append(vector / norm)
    if not roots:
        return None

    forward = np.mean(roots, axis=0)
    forward = forward - np.dot(forward, lateral) * lateral
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1e-8:
        return None
    forward = forward / forward_norm

    normal = np.cross(lateral, forward)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1e-8:
        return None
    normal = normal / normal_norm
    return lateral, forward, normal
