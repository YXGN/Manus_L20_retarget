"""Argument parsing for the l20_ik_core CLI."""

from __future__ import annotations

import argparse

from l20_ik_core.domain import normalize_hand_side
from l20_ik_core.paths import DEFAULT_BIHAND_CONFIG_PATH, DEFAULT_CONFIG_PATH, DEFAULT_HC_MOCAP_REFERENCE_BVH


class _L20IkArgumentParser(argparse.ArgumentParser):
    def parse_known_args(self, args=None, namespace=None):
        parsed_args, extras = super().parse_known_args(args, namespace)
        normalize_both_hand_args(parsed_args)
        return parsed_args, extras


def parse_hand_selector(value: str) -> str:
    normalized = value.strip().lower()
    if normalized == "both":
        return "both"
    return normalize_hand_side(value)


def parse_l20_contact_float4(value: str) -> tuple[float, float, float, float]:
    try:
        values = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected four comma-separated numbers") from exc
    if len(values) != 4:
        raise argparse.ArgumentTypeError("expected exactly four values: index,middle,ring,pinky")
    return values


def parse_l20_contact_positive_float4(value: str) -> tuple[float, float, float, float]:
    values = parse_l20_contact_float4(value)
    if any(item <= 0.0 for item in values):
        raise argparse.ArgumentTypeError("all four values must be positive")
    return values


def parse_l20_contact_nonnegative_float4(value: str) -> tuple[float, float, float, float]:
    values = parse_l20_contact_float4(value)
    if any(item < 0.0 for item in values):
        raise argparse.ArgumentTypeError("all four values must be >= 0")
    return values


def parse_l20_contact_active_contacts(value: str) -> tuple[str, ...]:
    contacts = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    allowed = {"all", "index", "middle", "ring", "pinky"}
    if not contacts:
        raise argparse.ArgumentTypeError("active contacts cannot be empty")
    unknown = [contact for contact in contacts if contact not in allowed]
    if unknown:
        raise argparse.ArgumentTypeError("contacts must be from all,index,middle,ring,pinky")
    if "all" in contacts and len(contacts) > 1:
        raise argparse.ArgumentTypeError("'all' cannot be combined with other contacts")
    return contacts


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-c",
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to retargeting config YAML",
    )
    parser.add_argument(
        "-H",
        "--hand",
        type=parse_hand_selector,
        choices=["left", "right", "both"],
        default="right",
        help="Hand side for the current channel, or 'both' for two-hand mode",
    )
    parser.add_argument(
        "--record-output",
        default=None,
        help="Output pickle file for recorded hand-tracking frames",
    )
    parser.add_argument(
        "--backend",
        choices=["viewer", "sim", "real"],
        default="viewer",
        help="Execution backend for robot-hand output",
    )
    parser.add_argument("--control-rate", type=int, default=100, help="Controller update rate in Hz")
    parser.add_argument("--sim-rate", type=int, default=500, help="MuJoCo simulation rate in Hz")
    parser.add_argument("--transport", choices=["can", "modbus"], default="can", help="Real-hand transport mode")
    parser.add_argument("--can-interface", default="can0", help="CAN interface name for real-hand mode")
    parser.add_argument("--modbus-port", default="None", help="MODBUS serial port for real-hand mode")
    parser.add_argument("--sdk-root", default=None, help="Optional LinkerHand SDK root directory")
    parser.add_argument("--model-family", default=None, help="Optional LinkerHand SDK model family override")
    parser.add_argument(
        "--real-state-print-every",
        type=int,
        default=0,
        help="Print real-hand returned state every N reads; 0 disables printing",
    )
    parser.add_argument(
        "--real-command-print-every",
        type=int,
        default=0,
        help="Print real-hand target joint angles and outgoing command every N sends; 0 disables printing",
    )
    parser.add_argument(
        "--mediapipe-angle-print-every",
        type=int,
        default=0,
        help="Print MediaPipe 3D middle/tip flexion angles every N detected frames; 0 disables printing",
    )
    parser.add_argument(
        "--l20-flexion-calibration",
        default=None,
        help="Path to L20 flexion calibration JSON applied after retargeting and before real-hand commands",
    )
    parser.add_argument(
        "--l20-flexion-deadzone",
        type=float,
        default=0.0,
        help="Open-palm deadzone for normalized L20 flexion calibration; 0 disables deadzone",
    )
    parser.add_argument(
        "--l20-flexion-base-follow-tip",
        type=float,
        default=0.0,
        help="How much normalized fingertip flexion pulls base flexion; 0 disables coupling",
    )
    parser.add_argument(
        "--l20-flexion-tip-gamma",
        type=float,
        default=1.0,
        help="Gamma curve for fingertip flexion; values > 1 delay fingertip closing",
    )
    parser.add_argument(
        "--l20-flexion-close-command-floor",
        type=int,
        default=0,
        help="Minimum 0-255 flexion command after calibration; larger values make closing softer",
    )
    parser.add_argument(
        "--l20-invert-finger-yaw",
        action="store_true",
        help="Invert L20 four-finger yaw command slots 6:10 without changing thumb yaw or roll",
    )
    parser.add_argument(
        "--l20-joint-range-mapping",
        default=None,
        help="Path to L20 joint range mapping YAML applied after retargeting and before flexion calibration",
    )
    parser.add_argument(
        "--l20-contact-ik",
        action="store_true",
        help="Enable experimental L20 contact IK refinement before real/sim controller commands",
    )
    parser.add_argument(
        "--l20-contact-active-joints",
        choices=("thumb", "thumb-soft-pitch", "thumb-all", "all"),
        default="thumb",
        help="Active joints for L20 contact IK; 'thumb' avoids thumb root pitch changes",
    )
    parser.add_argument(
        "--l20-contact-active-contacts",
        type=parse_l20_contact_active_contacts,
        default=("pinky",),
        help="Comma-separated contacts solved by L20 contact IK: all,index,middle,ring,pinky",
    )
    parser.add_argument(
        "--l20-contact-objective",
        choices=("distance", "vector"),
        default="vector",
        help="L20 contact IK objective; vector keeps planar thumb-finger direction",
    )
    parser.add_argument(
        "--l20-contact-max-delta",
        type=float,
        default=0.12,
        help="Maximum radians each active L20 contact IK joint may move from base retarget qpos",
    )
    parser.add_argument(
        "--l20-contact-weights",
        type=parse_l20_contact_positive_float4,
        default=(1.0, 1.0, 1.0, 1.8),
        help="Comma-separated index,middle,ring,pinky L20 contact IK weights",
    )
    parser.add_argument(
        "--l20-contact-target-scales",
        type=parse_l20_contact_positive_float4,
        default=(1.0, 1.0, 1.0, 0.92),
        help="Comma-separated index,middle,ring,pinky L20 contact target distance scales",
    )
    parser.add_argument(
        "--l20-contact-min-target-distances",
        type=parse_l20_contact_nonnegative_float4,
        default=(0.0, 0.0, 0.0, 0.08),
        help="Comma-separated index,middle,ring,pinky minimum L20 contact target distances in meters",
    )
    parser.add_argument(
        "--l20-contact-soft-pitch-delta",
        type=float,
        default=0.0,
        help="Maximum thumb root pitch delta for thumb-soft-pitch mode; keep 0 unless explicitly testing pitch",
    )
    parser.add_argument(
        "--l20-contact-ik-print-every",
        type=int,
        default=0,
        help="Print L20 contact IK errors every N detected frames; 0 disables printing",
    )
    parser.add_argument(
        "--real-can-dump",
        action="store_true",
        help="Print raw LinkerHand CAN response frames in real-hand mode",
    )


def add_live_sampling_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--signal-fps",
        type=int,
        default=None,
        help="Fixed output sampling rate for live mocap input; defaults to the source nominal fps",
    )


def add_mediapipe_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--mediapipe-detection-confidence",
        type=float,
        default=0.7,
        help="Minimum MediaPipe hand detection confidence",
    )
    parser.add_argument(
        "--mediapipe-tracking-confidence",
        type=float,
        default=0.5,
        help="Minimum MediaPipe hand tracking confidence",
    )
    parser.add_argument(
        "--tracking-hold-frames",
        type=int,
        default=0,
        help="Hold the last valid webcam/video detection for N bad or missing frames",
    )
    parser.add_argument(
        "--tracking-jump-threshold",
        type=float,
        default=0.0,
        help="Reject landmark jumps larger than this MediaPipe world-coordinate distance; 0 disables",
    )


def add_dump_video_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-c",
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to retargeting config YAML",
    )
    parser.add_argument(
        "-H",
        "--hand",
        type=parse_hand_selector,
        choices=["left", "right", "both"],
        default="right",
        help="Hand side for the current channel, or 'both' for two-hand mode",
    )
    parser.add_argument("--recording", required=True, help="Path to a saved hand-tracking recording")
    parser.add_argument("--output", required=True, help="Output MP4 path for the rendered replay video")


def normalize_both_hand_args(args: argparse.Namespace) -> None:
    if getattr(args, "hand", None) != "both":
        return
    if getattr(args, "config", None) == str(DEFAULT_CONFIG_PATH):
        args.config = str(DEFAULT_BIHAND_CONFIG_PATH)


def build_parser() -> argparse.ArgumentParser:
    parser = _L20IkArgumentParser(prog="l20_ik_core", description="Unified dex hand retargeting CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    webcam = subparsers.add_parser("webcam", help="Retarget from a live webcam stream")
    add_common_args(webcam)
    add_mediapipe_args(webcam)
    webcam.add_argument("--camera", type=int, default=0, help="Webcam device index")
    webcam.add_argument("--camera-width", type=int, default=None, help="Requested webcam capture width")
    webcam.add_argument("--camera-height", type=int, default=None, help="Requested webcam capture height")
    webcam.add_argument("--camera-fps", type=int, default=None, help="Requested webcam capture FPS")
    webcam.add_argument(
        "--swap-hands",
        action="store_true",
        help="Swap MediaPipe Left/Right labels if capture reports the opposite hand",
    )
    webcam.add_argument(
        "--save-webcam-video",
        action="store_true",
        help="Save raw webcam input frames to recordings/webcam_<timestamp>.mp4",
    )
    webcam.add_argument(
        "--webcam-video-output-dir",
        default="recordings",
        help="Directory for --save-webcam-video timestamped MP4 files",
    )

    video = subparsers.add_parser("video", help="Retarget from a video file")
    add_common_args(video)
    add_mediapipe_args(video)
    video.add_argument("--video", required=True, help="Path to input video file")
    video.add_argument(
        "--swap-hands",
        action="store_true",
        help="Swap MediaPipe Left/Right labels if this video reports the opposite hand",
    )

    realsense = subparsers.add_parser(
        "realsense",
        help="Retarget from an Intel RealSense color+depth stream with MediaPipe 2D landmarks",
    )
    add_common_args(realsense)
    add_mediapipe_args(realsense)
    realsense.add_argument(
        "--swap-hands",
        action="store_true",
        help="Swap MediaPipe Left/Right labels if RealSense color reports the opposite hand",
    )
    realsense.add_argument("--realsense-color-width", type=int, default=1280, help="RealSense color stream width")
    realsense.add_argument("--realsense-color-height", type=int, default=720, help="RealSense color stream height")
    realsense.add_argument("--realsense-depth-width", type=int, default=1280, help="RealSense depth stream width")
    realsense.add_argument("--realsense-depth-height", type=int, default=720, help="RealSense depth stream height")
    realsense.add_argument("--realsense-fps", type=int, default=30, help="RealSense stream FPS")
    realsense.add_argument(
        "--realsense-depth-patch-radius",
        type=int,
        default=2,
        help="Median depth patch radius around each 2D hand landmark; 0 samples only the landmark pixel",
    )
    realsense.add_argument(
        "--realsense-depth-scale",
        type=float,
        default=1.0,
        help="Scale applied to deprojected RealSense 3D landmarks before retargeting",
    )

    replay = subparsers.add_parser("replay", help="Replay a saved hand-tracking recording")
    add_common_args(replay)
    replay.add_argument("--recording", required=True, help="Path to a saved hand-tracking recording")
    replay.add_argument("--loop", action="store_true", help="Loop the saved recording indefinitely")

    dump_video = subparsers.add_parser("dump-video", help="Render a replay recording to MP4 as fast as possible")
    add_dump_video_args(dump_video)

    pico = subparsers.add_parser("pico", help="Retarget from live PICO hand tracking via PICO Bridge")
    add_common_args(pico)
    add_live_sampling_args(pico)
    pico.add_argument("--pico-host", default="0.0.0.0", help="PICO Bridge receiver bind host")
    pico.add_argument("--pico-port", type=int, default=63901, help="PICO Bridge receiver TCP port")
    pico.add_argument(
        "--pico-advertise-ip",
        default=None,
        help="Optional PC IPv4 address advertised to the headset",
    )
    pico.add_argument(
        "--no-pico-discovery",
        action="store_true",
        help="Disable PICO Bridge UDP discovery broadcasts",
    )
    pico.add_argument(
        "--pico-timeout",
        type=float,
        default=60.0,
        help="Timeout in seconds while waiting for PICO Bridge hand-tracking frames",
    )

    hc_mocap = subparsers.add_parser("hc-mocap", help="Retarget from a live hc_mocap UDP stream")
    add_common_args(hc_mocap)
    add_live_sampling_args(hc_mocap)
    hc_mocap.add_argument(
        "--reference-bvh",
        default=str(DEFAULT_HC_MOCAP_REFERENCE_BVH),
        help="Optional custom hc_mocap BVH override; default uses built-in joint ordering",
    )
    hc_mocap.add_argument("--udp-host", default="", help="UDP bind host for hc_mocap input")
    hc_mocap.add_argument("--udp-port", type=int, default=1118, help="UDP port for hc_mocap input")
    hc_mocap.add_argument("--udp-timeout", type=float, default=30.0, help="UDP startup timeout in seconds")
    hc_mocap.add_argument(
        "--udp-stats-every",
        type=int,
        default=120,
        help="Print UDP receive statistics every N processed frames (0 disables)",
    )

    return parser


__all__ = [
    "_L20IkArgumentParser",
    "add_common_args",
    "add_dump_video_args",
    "add_live_sampling_args",
    "add_mediapipe_args",
    "build_parser",
    "normalize_both_hand_args",
    "parse_hand_selector",
]
