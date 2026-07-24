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
    # Right MANUS glove coordinates mapped into the canonical right-hand
    # retargeting space used by the L20 mapping and thumb IK code.
    "right_glove_to_right_retarget": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    # Mirror a physical left MANUS glove into the same canonical space used by
    # the verified right-hand retarget path. This keeps the right-hand
    # calibration/IK command semantics while feeding them right-handed geometry.
    "left_glove_to_right_retarget": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    ),
}


def manus_raw_nodes_to_mediapipe_landmarks(
    raw_nodes: list[ManusRawNode],
    *,
    transform: str = "right_glove_to_right_retarget",
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


def ordered_chain_debug(raw_nodes: list[ManusRawNode], chain: str) -> list[dict[str, object]]:
    by_joint: dict[str, ManusRawNode] = {}
    for node in raw_nodes:
        if str(node.chain_type) == chain:
            by_joint.setdefault(str(node.joint_type), node)
    items: list[dict[str, object]] = []
    for joint in JOINT_ORDER:
        node = by_joint.get(joint)
        if node is None:
            continue
        position = _node_position(node)
        orientation = _node_quaternion(node)
        items.append(
            {
                "joint": joint,
                "position": np.round(position, 6).tolist(),
                "orientation_xyzw": np.round(orientation, 6).tolist(),
            }
        )
    return items


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
    # stable and are closer to the palm-forward direction needed by l20_ik_core's
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


def _node_quaternion(node: ManusRawNode) -> np.ndarray:
    orientation = node.pose.orientation
    quat = np.asarray([orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-8:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


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
