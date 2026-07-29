from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from manus_ros2_msgs.msg import ManusRawNode

LANDMARK_LAYOUT = {
    "Thumb": (("MCP", "PIP", "DIP", "TIP"), (1, 2, 3, 4)),
    "Index": (("MCP", "PIP", "DIP", "TIP"), (5, 6, 7, 8)),
    "Middle": (("MCP", "PIP", "DIP", "TIP"), (9, 10, 11, 12)),
    "Ring": (("MCP", "PIP", "DIP", "TIP"), (13, 14, 15, 16)),
    "Pinky": (("MCP", "PIP", "DIP", "TIP"), (17, 18, 19, 20)),
}


@dataclass(slots=True)
class ManusFingerSkeleton:
    """Named MANUS raw-skeleton positions for one finger."""

    mcp: np.ndarray
    pip: np.ndarray
    dip: np.ndarray
    tip: np.ndarray


@dataclass(slots=True)
class ManusRawPose:
    """Untransformed raw-node pose retained for native-pose work."""

    position: np.ndarray
    orientation_xyzw: np.ndarray


@dataclass(slots=True)
class ManusHandSkeleton:
    """MANUS hand skeleton with named joints instead of landmark indexes."""

    thumb: ManusFingerSkeleton
    index: ManusFingerSkeleton
    middle: ManusFingerSkeleton
    ring: ManusFingerSkeleton
    pinky: ManusFingerSkeleton
    raw_poses: dict[tuple[str, str], ManusRawPose]

TRANSFORMS: dict[str, np.ndarray] = {
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
    transform: str,
) -> np.ndarray:
    """Legacy 21-point adapter retained for the current geometry/IK path."""
    return manus_hand_skeleton_to_mediapipe_landmarks(
        manus_raw_nodes_to_hand_skeleton(raw_nodes, transform=transform)
    )


def manus_raw_nodes_to_hand_skeleton(
    raw_nodes: list[ManusRawNode],
    *,
    transform: str,
) -> ManusHandSkeleton:
    points: dict[str, dict[str, np.ndarray]] = {chain: {} for chain in LANDMARK_LAYOUT}
    raw_poses: dict[tuple[str, str], ManusRawPose] = {}
    matrix = TRANSFORMS.get(transform)
    if matrix is None:
        raise ValueError(f"unknown MANUS landmark transform: {transform}")

    for node in raw_nodes:
        chain = str(node.chain_type)
        joint = str(node.joint_type)
        raw_poses[(chain, joint)] = _node_pose(node)
        if chain not in points:
            continue
        required_joints, _ = LANDMARK_LAYOUT[chain]
        if joint not in required_joints:
            continue
        if joint in points[chain]:
            raise ValueError(f"MANUS raw_nodes contains duplicate joint: {chain}.{joint}")
        points[chain][joint] = _node_position(node) @ matrix.T

    missing = [
        f"{chain}.{joint}"
        for chain, (required_joints, _) in LANDMARK_LAYOUT.items()
        for joint in required_joints
        if joint not in points[chain]
    ]
    if missing:
        raise ValueError(f"MANUS raw_nodes missing required joints: {missing}")

    def finger(chain: str) -> ManusFingerSkeleton:
        return ManusFingerSkeleton(
            mcp=points[chain]["MCP"],
            pip=points[chain]["PIP"],
            dip=points[chain]["DIP"],
            tip=points[chain]["TIP"],
        )

    return ManusHandSkeleton(
        thumb=finger("Thumb"),
        index=finger("Index"),
        middle=finger("Middle"),
        ring=finger("Ring"),
        pinky=finger("Pinky"),
        raw_poses=raw_poses,
    )


def manus_hand_skeleton_to_mediapipe_landmarks(skeleton: ManusHandSkeleton) -> np.ndarray:
    """Convert named MANUS joints to the legacy MediaPipe-compatible layout."""
    landmarks = np.zeros((21, 3), dtype=np.float64)
    fingers = {
        "Thumb": skeleton.thumb,
        "Index": skeleton.index,
        "Middle": skeleton.middle,
        "Ring": skeleton.ring,
        "Pinky": skeleton.pinky,
    }
    for chain, (required_joints, slots) in LANDMARK_LAYOUT.items():
        finger = fingers[chain]
        landmarks[list(slots)] = [finger.mcp, finger.pip, finger.dip, finger.tip]

    landmarks[0] = _estimate_wrist(landmarks)
    return landmarks

def _node_position(node: ManusRawNode) -> np.ndarray:
    position = node.pose.position
    return np.asarray([position.x, position.y, position.z], dtype=np.float64)


def _node_pose(node: ManusRawNode) -> ManusRawPose:
    orientation = node.pose.orientation
    return ManusRawPose(
        position=_node_position(node),
        orientation_xyzw=np.asarray(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            dtype=np.float64,
        ),
    )


def _estimate_wrist(landmarks: np.ndarray) -> np.ndarray:
    mcp_indices = [5, 9, 13, 17]
    palm_center = np.mean(landmarks[mcp_indices], axis=0)

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
