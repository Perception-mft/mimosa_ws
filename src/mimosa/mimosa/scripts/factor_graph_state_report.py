#!/usr/bin/env python3
"""Compare Mimosa pre-GTSAM IMU/LiDAR states with GPS and AWSIM ground truth."""

from __future__ import annotations

import argparse
import html
import math
import shutil
import sqlite3
import subprocess
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

EXPECTED_TYPE = "mimosa_msgs/msg/FactorGraphState"
GPS_TYPE = "sensor_msgs/msg/NavSatFix"
ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
POSE_STAMPED_TYPE = "geometry_msgs/msg/PoseStamped"
DEFAULT_IMU_TOPIC = "/debug/imu/state"
DEFAULT_LIDAR_TOPIC = "/debug/lidar/state"
DEFAULT_GPS_TOPIC = "/sensing/gnss/nav_sat_fix"
DEFAULT_ODOMETRY_TOPIC = "/mimosa_node/graph/odometry"
DEFAULT_AWSIM_TOPIC = "/control/state/pose"
M113_CONFIG_RELATIVE_PATH = Path("config/m113/params.yaml")

go = None
pio = None
make_subplots = None


def git_revision() -> Tuple[str, str]:
    """Return the branch and commit for the checkout that generated the report."""
    for candidate in (Path(__file__).resolve().parent, Path.cwd()):
        try:
            repository = subprocess.run(
                ["git", "rev-parse", "--show-toplevel"],
                cwd=candidate,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=repository,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            branch_result = subprocess.run(
                ["git", "branch", "--show-current"],
                cwd=repository,
                check=True,
                capture_output=True,
                text=True,
            )
            branch = branch_result.stdout.strip()
            if not branch:
                remote_branches = subprocess.run(
                    [
                        "git",
                        "branch",
                        "--remotes",
                        "--points-at",
                        "HEAD",
                        "--format=%(refname:short)",
                    ],
                    cwd=repository,
                    check=True,
                    capture_output=True,
                    text=True,
                ).stdout.splitlines()
                branch = next(
                    (name for name in remote_branches if not name.endswith("/HEAD")),
                    "detached HEAD",
                )
            return branch, commit
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    return "unavailable", "unavailable"


def load_plotly() -> None:
    global go, pio, make_subplots
    try:
        import plotly.graph_objects as plotly_go
        import plotly.io as plotly_io
        from plotly.subplots import make_subplots as plotly_make_subplots
    except ImportError as exc:
        raise SystemExit(
            "Plotly is required. Install the package dependency with rosdep or run "
            "'sudo apt install python3-plotly'."
        ) from exc
    go = plotly_go
    pio = plotly_io
    make_subplots = plotly_make_subplots


@dataclass
class StateSeries:
    time: np.ndarray
    state_index: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray
    rpy_deg: np.ndarray
    velocity: np.ndarray
    accelerometer_bias: np.ndarray
    gyroscope_bias: np.ndarray
    gravity: np.ndarray

    def __len__(self) -> int:
        return len(self.time)


@dataclass
class DecodedState:
    timestamp: float
    state_index: int
    position: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray
    accelerometer_bias: np.ndarray
    gyroscope_bias: np.ndarray
    gravity: np.ndarray


@dataclass
class DecodedGps:
    timestamp: float
    status: int
    latitude: float
    longitude: float
    altitude: float
    covariance: np.ndarray


@dataclass
class DecodedPose:
    timestamp: float
    position: np.ndarray
    quaternion: np.ndarray


@dataclass
class GpsSeries:
    time: np.ndarray
    position_enu: np.ndarray
    covariance: np.ndarray

    def __len__(self) -> int:
        return len(self.time)


@dataclass
class PoseSeries:
    time: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray
    rpy_deg: np.ndarray

    def __len__(self) -> int:
        return len(self.time)


class CdrReader:
    """Minimal CDR decoder for the two message types used by this report."""

    def __init__(self, payload: bytes):
        if len(payload) < 4:
            raise ValueError("CDR payload is shorter than its encapsulation header")
        encapsulation = int.from_bytes(payload[:2], byteorder="big")
        if encapsulation not in (0, 1):
            raise ValueError(f"Unsupported CDR encapsulation: 0x{encapsulation:04x}")
        self.payload = payload
        self.endian = "<" if encapsulation == 1 else ">"
        self.offset = 4
        self.alignment_origin = 4

    def align(self, size: int) -> None:
        relative_offset = self.offset - self.alignment_origin
        self.offset += (-relative_offset) % size

    def unpack(self, code: str, size: int):
        self.align(size)
        if self.offset + size > len(self.payload):
            raise ValueError("Unexpected end of CDR payload")
        value = struct.unpack_from(self.endian + code, self.payload, self.offset)[0]
        self.offset += size
        return value

    def int32(self) -> int:
        return self.unpack("i", 4)

    def int8(self) -> int:
        return self.unpack("b", 1)

    def uint8(self) -> int:
        return self.unpack("B", 1)

    def uint16(self) -> int:
        return self.unpack("H", 2)

    def uint32(self) -> int:
        return self.unpack("I", 4)

    def uint64(self) -> int:
        return self.unpack("Q", 8)

    def float64(self) -> float:
        return self.unpack("d", 8)

    def string(self) -> str:
        size = self.uint32()
        if size == 0:
            return ""
        if self.offset + size > len(self.payload):
            raise ValueError("Unexpected end of CDR string")
        raw = self.payload[self.offset : self.offset + size]
        self.offset += size
        if raw.endswith(b"\0"):
            raw = raw[:-1]
        return raw.decode("utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read Mimosa pre-optimization IMU/LiDAR states, optimized odometry, "
            "NavSatFix GPS, and AWSIM ground truth from a ROS 2 bag, then create an "
            "interactive Plotly HTML report."
        )
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory, .db3 file, or .mcap file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output HTML path (default: factor_graph_report.html inside a bag directory)",
    )
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC)
    parser.add_argument("--gps-topic", default=DEFAULT_GPS_TOPIC)
    parser.add_argument("--odometry-topic", default=DEFAULT_ODOMETRY_TOPIC)
    parser.add_argument("--awsim-topic", default=DEFAULT_AWSIM_TOPIC)
    parser.add_argument(
        "--max-time-difference",
        type=float,
        default=0.05,
        help="Maximum IMU/LiDAR/output timestamp difference for orientation errors, in seconds",
    )
    parser.add_argument(
        "--gps-max-time-difference",
        type=float,
        default=0.2,
        help="Maximum state-to-GPS timestamp difference, in seconds",
    )
    parser.add_argument(
        "--gps-alignment",
        choices=("se3", "translation", "none"),
        default="se3",
        help="Alignment applied to the common Mimosa map frame before GPS comparison",
    )
    parser.add_argument(
        "--plotlyjs",
        choices=("inline", "cdn"),
        default="inline",
        help="Embed Plotly JS for an offline report or load it from the CDN",
    )
    return parser.parse_args()


def quaternion_to_rpy_deg(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return np.rad2deg([roll, pitch, yaw])


def decode_state(payload: bytes, bag_timestamp_ns: int) -> DecodedState:
    reader = CdrReader(payload)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    reader.string()  # header.frame_id is not needed for plotting.
    state_index = reader.uint64()
    values = np.asarray([reader.float64() for _ in range(19)])
    timestamp = (
        bag_timestamp_ns * 1e-9
        if seconds == 0 and nanoseconds == 0
        else float(seconds) + float(nanoseconds) * 1e-9
    )
    return DecodedState(
        timestamp=timestamp,
        state_index=state_index,
        position=values[0:3],
        quaternion=values[3:7],
        velocity=values[7:10],
        accelerometer_bias=values[10:13],
        gyroscope_bias=values[13:16],
        gravity=values[16:19],
    )


def decode_gps(payload: bytes, bag_timestamp_ns: int) -> DecodedGps:
    reader = CdrReader(payload)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    reader.string()  # header.frame_id
    status = reader.int8()
    reader.uint16()  # NavSatStatus.service
    latitude = reader.float64()
    longitude = reader.float64()
    altitude = reader.float64()
    covariance = np.asarray([reader.float64() for _ in range(9)]).reshape(3, 3)
    reader.uint8()  # position_covariance_type
    timestamp = (
        bag_timestamp_ns * 1e-9
        if seconds == 0 and nanoseconds == 0
        else float(seconds) + float(nanoseconds) * 1e-9
    )
    return DecodedGps(
        timestamp=timestamp,
        status=status,
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        covariance=covariance,
    )


def decode_odometry(payload: bytes, bag_timestamp_ns: int) -> DecodedPose:
    reader = CdrReader(payload)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    reader.string()  # header.frame_id
    reader.string()  # child_frame_id
    position = np.asarray([reader.float64() for _ in range(3)])
    quaternion = np.asarray([reader.float64() for _ in range(4)])
    timestamp = (
        bag_timestamp_ns * 1e-9
        if seconds == 0 and nanoseconds == 0
        else float(seconds) + float(nanoseconds) * 1e-9
    )
    return DecodedPose(timestamp, position, quaternion)


def decode_pose_stamped(payload: bytes, bag_timestamp_ns: int) -> DecodedPose:
    reader = CdrReader(payload)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    reader.string()  # header.frame_id
    position = np.asarray([reader.float64() for _ in range(3)])
    quaternion = np.asarray([reader.float64() for _ in range(4)])
    timestamp = (
        bag_timestamp_ns * 1e-9
        if seconds == 0 and nanoseconds == 0
        else float(seconds) + float(nanoseconds) * 1e-9
    )
    return DecodedPose(timestamp, position, quaternion)


def empty_series() -> StateSeries:
    return StateSeries(
        time=np.empty(0),
        state_index=np.empty(0, dtype=np.uint64),
        position=np.empty((0, 3)),
        quaternion=np.empty((0, 4)),
        rpy_deg=np.empty((0, 3)),
        velocity=np.empty((0, 3)),
        accelerometer_bias=np.empty((0, 3)),
        gyroscope_bias=np.empty((0, 3)),
        gravity=np.empty((0, 3)),
    )


def records_to_series(records: List[DecodedState]) -> StateSeries:
    if not records:
        return empty_series()

    records.sort(key=lambda item: item.timestamp)
    quaternion_array = np.asarray([item.quaternion for item in records])
    rpy = np.asarray([quaternion_to_rpy_deg(value) for value in quaternion_array])
    # Avoid artificial +/-180 degree jumps while preserving each source independently.
    rpy = np.rad2deg(np.unwrap(np.deg2rad(rpy), axis=0))

    return StateSeries(
        time=np.asarray([item.timestamp for item in records]),
        state_index=np.asarray([item.state_index for item in records], dtype=np.uint64),
        position=np.asarray([item.position for item in records]),
        quaternion=quaternion_array,
        rpy_deg=rpy,
        velocity=np.asarray([item.velocity for item in records]),
        accelerometer_bias=np.asarray([item.accelerometer_bias for item in records]),
        gyroscope_bias=np.asarray([item.gyroscope_bias for item in records]),
        gravity=np.asarray([item.gravity for item in records]),
    )


def records_to_pose_series(records: List[DecodedPose]) -> PoseSeries:
    if not records:
        return PoseSeries(
            np.empty(0), np.empty((0, 3)), np.empty((0, 4)), np.empty((0, 3))
        )
    records.sort(key=lambda item: item.timestamp)
    quaternions = np.asarray([item.quaternion for item in records])
    rpy = np.asarray([quaternion_to_rpy_deg(value) for value in quaternions])
    rpy = np.rad2deg(np.unwrap(np.deg2rad(rpy), axis=0))
    return PoseSeries(
        time=np.asarray([item.timestamp for item in records]),
        position=np.asarray([item.position for item in records]),
        quaternion=quaternions,
        rpy_deg=rpy,
    )


def geodetic_to_ecef(latitude_deg: np.ndarray, longitude_deg: np.ndarray, altitude: np.ndarray) -> np.ndarray:
    """Convert WGS84 latitude, longitude, and ellipsoid altitude to ECEF."""
    semi_major_axis = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = flattening * (2.0 - flattening)
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    sin_latitude = np.sin(latitude)
    cos_latitude = np.cos(latitude)
    radius = semi_major_axis / np.sqrt(1.0 - eccentricity_squared * sin_latitude**2)
    return np.column_stack(
        (
            (radius + altitude) * cos_latitude * np.cos(longitude),
            (radius + altitude) * cos_latitude * np.sin(longitude),
            (radius * (1.0 - eccentricity_squared) + altitude) * sin_latitude,
        )
    )


def records_to_gps_series(records: List[DecodedGps]) -> GpsSeries:
    valid = [
        item
        for item in records
        if item.status >= 0
        and np.isfinite([item.latitude, item.longitude, item.altitude]).all()
        and -90.0 <= item.latitude <= 90.0
        and -180.0 <= item.longitude <= 180.0
    ]
    if not valid:
        return GpsSeries(np.empty(0), np.empty((0, 3)), np.empty((0, 3, 3)))
    valid.sort(key=lambda item: item.timestamp)
    latitude = np.asarray([item.latitude for item in valid])
    longitude = np.asarray([item.longitude for item in valid])
    altitude = np.asarray([item.altitude for item in valid])
    ecef = geodetic_to_ecef(latitude, longitude, altitude)

    latitude_origin = math.radians(latitude[0])
    longitude_origin = math.radians(longitude[0])
    sin_latitude, cos_latitude = math.sin(latitude_origin), math.cos(latitude_origin)
    sin_longitude, cos_longitude = math.sin(longitude_origin), math.cos(longitude_origin)
    ecef_to_enu = np.array(
        [
            [-sin_longitude, cos_longitude, 0.0],
            [-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude],
            [cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude],
        ]
    )
    position_enu = (ecef - ecef[0]) @ ecef_to_enu.T
    return GpsSeries(
        time=np.asarray([item.timestamp for item in valid]),
        position_enu=position_enu,
        covariance=np.asarray([item.covariance for item in valid]),
    )


def bag_storage_paths(bag_path: Path) -> Tuple[str, List[Path]]:
    if not bag_path.exists():
        raise FileNotFoundError(f"ROS 2 bag does not exist: {bag_path}")
    if bag_path.is_file() and bag_path.suffix == ".db3":
        return "sqlite3", [bag_path]
    if bag_path.is_file() and bag_path.suffix == ".mcap":
        return "mcap", [bag_path]
    if bag_path.is_dir():
        databases = sorted(bag_path.glob("*.db3"))
        if databases:
            return "sqlite3", databases
        mcap_files = sorted(bag_path.glob("*.mcap"))
        if mcap_files:
            return "mcap", mcap_files
    raise RuntimeError(f"No ROS 2 .db3 or .mcap files found in: {bag_path}")


def read_sqlite_files(
    databases: Sequence[Path], decoders: Dict[str, object]
) -> Tuple[Dict[str, List[object]], Dict[str, str]]:
    records: Dict[str, List[object]] = {topic: [] for topic in decoders}
    discovered_types: Dict[str, str] = {}

    for database in databases:
        connection = sqlite3.connect(f"file:{database.resolve()}?mode=ro", uri=True)
        try:
            for name, message_type in connection.execute("SELECT name, type FROM topics"):
                discovered_types[name] = message_type

            placeholders = ",".join("?" for _ in decoders)
            query = f"""
                SELECT topics.name, messages.timestamp, messages.data
                FROM messages
                JOIN topics ON topics.id = messages.topic_id
                WHERE topics.name IN ({placeholders})
                ORDER BY messages.timestamp
            """
            for topic, bag_timestamp_ns, payload in connection.execute(query, tuple(decoders)):
                try:
                    records[topic].append(decoders[topic](bytes(payload), bag_timestamp_ns))
                except (UnicodeDecodeError, ValueError, struct.error) as exc:
                    raise RuntimeError(
                        f"Failed to decode {topic} in {database} at {bag_timestamp_ns} ns: {exc}"
                    ) from exc
        except sqlite3.Error as exc:
            raise RuntimeError(f"Failed to read ROS 2 bag database '{database}': {exc}") from exc
        finally:
            connection.close()
    return records, discovered_types


def read_mcap_files(
    mcap_files: Sequence[Path], decoders: Dict[str, object]
) -> Tuple[Dict[str, List[object]], Dict[str, str]]:
    try:
        from mcap.reader import make_reader
    except ImportError as exc:
        raise RuntimeError(
            "Reading MCAP bags requires the standalone 'mcap' Python package. "
            "Install it with: python3 -m pip install mcap"
        ) from exc

    records: Dict[str, List[object]] = {topic: [] for topic in decoders}
    discovered_types: Dict[str, str] = {}
    for mcap_file in mcap_files:
        try:
            with mcap_file.open("rb") as stream:
                reader = make_reader(stream)
                for schema, channel, message in reader.iter_messages(topics=list(decoders)):
                    if schema is None:
                        raise RuntimeError(
                            f"MCAP channel '{channel.topic}' has no message schema"
                        )
                    discovered_types[channel.topic] = schema.name
                    records[channel.topic].append(
                        decoders[channel.topic](bytes(message.data), message.log_time)
                    )
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(f"Failed to read MCAP file '{mcap_file}': {exc}") from exc
    return records, discovered_types


def read_bag(
    bag_path: Path,
    imu_topic: str,
    lidar_topic: str,
    gps_topic: str,
    odometry_topic: str,
    awsim_topic: str,
) -> Tuple[Dict[str, StateSeries], GpsSeries, PoseSeries, PoseSeries]:
    decoders = {
        imu_topic: decode_state,
        lidar_topic: decode_state,
        gps_topic: decode_gps,
        odometry_topic: decode_odometry,
        awsim_topic: decode_pose_stamped,
    }
    storage_id, storage_files = bag_storage_paths(bag_path)
    if storage_id == "sqlite3":
        records, discovered_types = read_sqlite_files(storage_files, decoders)
    else:
        records, discovered_types = read_mcap_files(storage_files, decoders)

    missing = [topic for topic in decoders if topic not in discovered_types]
    if missing:
        available = "\n  ".join(sorted(discovered_types))
        raise RuntimeError(
            f"Required topic(s) not found: {', '.join(missing)}\nAvailable topics:\n  {available}"
        )

    expected_types = {
        imu_topic: EXPECTED_TYPE,
        lidar_topic: EXPECTED_TYPE,
        gps_topic: GPS_TYPE,
        odometry_topic: ODOMETRY_TYPE,
        awsim_topic: POSE_STAMPED_TYPE,
    }
    wrong_types = [
        f"{topic}: expected {expected_types[topic]}, found {discovered_types[topic]}"
        for topic in decoders
        if discovered_types[topic] != expected_types[topic]
    ]
    if wrong_types:
        raise RuntimeError("Unexpected topic type(s): " + ", ".join(wrong_types))

    states = {
        imu_topic: records_to_series(records[imu_topic]),
        lidar_topic: records_to_series(records[lidar_topic]),
    }
    return (
        states,
        records_to_gps_series(records[gps_topic]),
        records_to_pose_series(records[odometry_topic]),
        records_to_pose_series(records[awsim_topic]),
    )


def nearest_matches(
    reference_time: np.ndarray, query_time: np.ndarray, max_difference: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Return query indices and nearest reference indices within max_difference."""
    if len(reference_time) == 0 or len(query_time) == 0:
        return np.empty(0, dtype=int), np.empty(0, dtype=int)

    right = np.searchsorted(reference_time, query_time, side="left")
    right = np.clip(right, 0, len(reference_time) - 1)
    left = np.clip(right - 1, 0, len(reference_time) - 1)
    use_left = np.abs(query_time - reference_time[left]) <= np.abs(
        reference_time[right] - query_time
    )
    nearest = np.where(use_left, left, right)
    valid = np.abs(query_time - reference_time[nearest]) <= max_difference
    return np.flatnonzero(valid), nearest[valid]


def estimate_position_alignment(
    source: np.ndarray, target: np.ndarray, mode: str
) -> Tuple[np.ndarray, np.ndarray]:
    """Estimate target ~= R * source + t without changing scale."""
    rotation = np.eye(3)
    if len(source) == 0:
        raise RuntimeError("No time-aligned IMU/GPS samples are available for alignment")
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    if mode == "se3":
        if len(source) < 3:
            raise RuntimeError("SE(3) GPS alignment requires at least three matched samples")
        left, _, right_transpose = np.linalg.svd(
            (source - source_center).T @ (target - target_center)
        )
        rotation = right_transpose.T @ left.T
        if np.linalg.det(rotation) < 0.0:
            right_transpose[-1, :] *= -1.0
            rotation = right_transpose.T @ left.T
    if mode == "none":
        translation = np.zeros(3)
    else:
        translation = target_center - rotation @ source_center
    return rotation, translation


def transform_positions(
    positions: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    return positions @ rotation.T + translation


def quaternion_to_rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Cannot convert a zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def aligned_rpy_deg(quaternions: np.ndarray, frame_rotation: np.ndarray) -> np.ndarray:
    """Express orientations in a frame reached by left-multiplying frame_rotation."""
    rpy = []
    for quaternion in quaternions:
        matrix = frame_rotation @ quaternion_to_rotation_matrix(quaternion)
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        rpy.append([roll, pitch, yaw])
    return np.rad2deg(np.unwrap(np.asarray(rpy), axis=0))


def zero_at_first_position(positions: np.ndarray) -> np.ndarray:
    """Express a position series relative to its first sample."""
    return positions - positions[0]


def gps_component_figure(
    component: int,
    imu_time: np.ndarray,
    imu_position: np.ndarray,
    lidar_time: np.ndarray,
    lidar_position: np.ndarray,
    output_time: np.ndarray,
    output_position: np.ndarray,
    gps_time: np.ndarray,
    gps_position: np.ndarray,
    awsim_time: np.ndarray,
    awsim_position: np.ndarray,
    imu_error_time: np.ndarray,
    imu_error: np.ndarray,
    lidar_error_time: np.ndarray,
    lidar_error: np.ndarray,
    output_error_time: np.ndarray,
    output_error: np.ndarray,
    output_gt_error_time: np.ndarray,
    output_gt_error: np.ndarray,
) -> go.Figure:
    axis = ("x", "y", "z")[component]
    figure = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        row_heights=[0.68, 0.32],
        vertical_spacing=0.1,
        subplot_titles=(f"Position {axis}", f"Position {axis} errors"),
    )
    figure.add_trace(
        go.Scatter(
            x=imu_time,
            y=imu_position[:, component],
            name="IMU",
            mode="lines",
            visible="legendonly",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=lidar_time,
            y=lidar_position[:, component],
            name="LiDAR",
            mode="lines+markers",
            marker={"size": 4},
            visible="legendonly",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=gps_time,
            y=gps_position[:, component],
            name="GPS",
            mode="lines+markers",
            marker={"size": 5},
            visible="legendonly",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=awsim_time,
            y=awsim_position[:, component],
            name="AWSIM ground truth",
            mode="lines",
            line={"width": 3, "dash": "dot"},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_time,
            y=output_position[:, component],
            name="Mimosa output",
            mode="lines",
            line={"width": 3},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=imu_error_time,
            y=imu_error[:, component],
            name="IMU - GPS",
            mode="lines",
            visible="legendonly",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=lidar_error_time,
            y=lidar_error[:, component],
            name="LiDAR - GPS",
            mode="lines",
            visible="legendonly",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_error_time,
            y=output_error[:, component],
            name="GPS - Mimosa output",
            mode="lines",
            line={"width": 3},
            visible="legendonly",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_gt_error_time,
            y=output_gt_error[:, component],
            name="AWSIM GT - Mimosa output",
            mode="lines",
            line={"width": 3},
        ),
        row=2,
        col=1,
    )
    figure.update_yaxes(title_text=f"Relative {axis} [m]", row=1, col=1)
    figure.update_yaxes(title_text="Error [m]", row=2, col=1, zeroline=True)
    figure.update_xaxes(title_text="Time from first sample [s]", row=2, col=1)
    figure.update_layout(
        title=f"Position {axis}: Mimosa output and AWSIM ground truth",
        height=700,
        hovermode="x unified",
        template="plotly_white",
        legend={"orientation": "h"},
    )
    return figure


def orientation_component_figure(
    component: int,
    imu_time: np.ndarray,
    imu_orientation: np.ndarray,
    lidar_time: np.ndarray,
    lidar_orientation: np.ndarray,
    output_time: np.ndarray,
    output_orientation: np.ndarray,
    awsim_time: np.ndarray,
    awsim_orientation: np.ndarray,
    lidar_error_time: np.ndarray,
    lidar_orientation_error: np.ndarray,
    output_error_time: np.ndarray,
    output_orientation_error: np.ndarray,
    output_gt_error_time: np.ndarray,
    output_gt_orientation_error: np.ndarray,
) -> go.Figure:
    angle = ("Roll", "Pitch", "Yaw")[component]
    figure = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        row_heights=[0.68, 0.32],
        vertical_spacing=0.1,
        subplot_titles=(angle, f"{angle} differences"),
    )
    figure.add_trace(
        go.Scatter(
            x=imu_time,
            y=imu_orientation[:, component],
            name="IMU",
            mode="lines",
            visible="legendonly",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=lidar_time,
            y=lidar_orientation[:, component],
            name="LiDAR",
            mode="lines+markers",
            marker={"size": 4},
            visible="legendonly",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_time,
            y=output_orientation[:, component],
            name="Mimosa output",
            mode="lines",
            line={"width": 3},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=awsim_time,
            y=awsim_orientation[:, component],
            name="AWSIM ground truth",
            mode="lines",
            line={"width": 3, "dash": "dot"},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=lidar_error_time,
            y=lidar_orientation_error[:, component],
            name="LiDAR - IMU",
            mode="lines",
            visible="legendonly",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_error_time,
            y=output_orientation_error[:, component],
            name="Mimosa output - IMU",
            mode="lines",
            line={"width": 3},
            visible="legendonly",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=output_gt_error_time,
            y=output_gt_orientation_error[:, component],
            name="AWSIM GT - Mimosa output",
            mode="lines",
            line={"width": 3},
        ),
        row=2,
        col=1,
    )
    figure.update_yaxes(title_text="Angle [deg]", row=1, col=1)
    figure.update_yaxes(title_text="Difference [deg]", row=2, col=1, zeroline=True)
    figure.update_xaxes(title_text="Time from first sample [s]", row=2, col=1)
    figure.update_layout(
        title=f"Orientation {angle.lower()}: Mimosa output and AWSIM ground truth",
        height=700,
        hovermode="x unified",
        template="plotly_white",
        legend={"orientation": "h"},
    )
    return figure


def gps_trajectory_figure(
    imu_position: np.ndarray,
    lidar_position: np.ndarray,
    output_position: np.ndarray,
    gps_position: np.ndarray,
    awsim_position: np.ndarray,
) -> go.Figure:
    figure = go.Figure()
    for name, position in (
        ("IMU", imu_position),
        ("LiDAR", lidar_position),
        ("Mimosa output", output_position),
        ("GPS", gps_position),
        ("AWSIM ground truth", awsim_position),
    ):
        figure.add_trace(
            go.Scatter3d(
                x=position[:, 0],
                y=position[:, 1],
                z=position[:, 2],
                name=name,
                mode="lines",
                visible=(
                    True
                    if name in ("Mimosa output", "AWSIM ground truth")
                    else "legendonly"
                ),
            )
        )
    figure.update_layout(
        title="Aligned 3D trajectories",
        height=1000,
        template="plotly_white",
        scene={"xaxis_title": "East [m]", "yaxis_title": "North [m]", "zaxis_title": "Up [m]"},
    )
    return figure


def wrapped_angle_difference_deg(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    return (first - second + 180.0) % 360.0 - 180.0


def summary_html(
    bag_path: Path,
    git_branch: str,
    git_commit: str,
    imu: StateSeries,
    lidar: StateSeries,
    gps: GpsSeries,
    output: PoseSeries,
    awsim: PoseSeries,
    matched_count: int,
    output_matched_count: int,
    position_delta: np.ndarray,
    orientation_delta: np.ndarray,
    gps_alignment: str,
    imu_gps_error: np.ndarray,
    lidar_gps_error: np.ndarray,
    gps_output_error: np.ndarray,
    output_gt_position_error: np.ndarray,
    output_gt_orientation_error: np.ndarray,
) -> str:
    def metric(values: np.ndarray, operation, fallback: str = "n/a") -> str:
        return fallback if values.size == 0 else f"{operation(values):.6g}"

    def component_metrics(
        values: np.ndarray,
        operation,
        labels: Sequence[str],
        unit: str,
        fallback: str = "n/a",
    ) -> str:
        if values.size == 0:
            return fallback
        result = operation(values)
        return ", ".join(
            f"{label}={value:.6g} {unit}" for label, value in zip(labels, result)
        )

    rows = [
        ("Bag", str(bag_path)),
        ("Git branch", git_branch),
        ("Git commit", git_commit),
        ("IMU samples", str(len(imu))),
        ("LiDAR samples", str(len(lidar))),
        ("Mimosa output samples", str(len(output))),
        ("AWSIM ground truth samples", str(len(awsim))),
        ("Valid GPS samples", str(len(gps))),
        ("LiDAR/IMU matched samples", str(matched_count)),
        ("Mimosa output/IMU matched samples", str(output_matched_count)),
        ("Mean LiDAR/IMU position difference", metric(np.linalg.norm(position_delta, axis=1), np.mean) + " m"),
        ("Maximum LiDAR/IMU position difference", metric(np.linalg.norm(position_delta, axis=1), np.max) + " m"),
        (
            "Mean absolute LiDAR/IMU orientation difference",
            metric(np.abs(orientation_delta), np.mean) + " deg",
        ),
        (
            "Maximum absolute LiDAR/IMU orientation difference",
            metric(np.abs(orientation_delta), np.max) + " deg",
        ),
        ("GPS alignment", gps_alignment),
        ("IMU/GPS matched samples", str(len(imu_gps_error))),
        ("LiDAR/GPS matched samples", str(len(lidar_gps_error))),
        (
            "IMU position RMSE against GPS",
            metric(np.linalg.norm(imu_gps_error, axis=1), lambda value: np.sqrt(np.mean(value**2)))
            + " m",
        ),
        (
            "IMU mean position error against GPS",
            metric(np.linalg.norm(imu_gps_error, axis=1), np.mean) + " m",
        ),
        (
            "IMU maximum position error against GPS",
            metric(np.linalg.norm(imu_gps_error, axis=1), np.max) + " m",
        ),
        (
            "LiDAR position RMSE against GPS",
            metric(
                np.linalg.norm(lidar_gps_error, axis=1),
                lambda value: np.sqrt(np.mean(value**2)),
            )
            + " m",
        ),
        (
            "LiDAR mean position error against GPS",
            metric(np.linalg.norm(lidar_gps_error, axis=1), np.mean) + " m",
        ),
        (
            "LiDAR maximum position error against GPS",
            metric(np.linalg.norm(lidar_gps_error, axis=1), np.max) + " m",
        ),
        ("GPS/Mimosa output matched samples", str(len(gps_output_error))),
        (
            "Mimosa output position RMSE against GPS",
            metric(
                np.linalg.norm(gps_output_error, axis=1),
                lambda value: np.sqrt(np.mean(value**2)),
            )
            + " m",
        ),
        (
            "Mimosa output mean position error against GPS",
            metric(np.linalg.norm(gps_output_error, axis=1), np.mean) + " m",
        ),
        (
            "Mimosa output maximum position error against GPS",
            metric(np.linalg.norm(gps_output_error, axis=1), np.max) + " m",
        ),
        ("Mimosa output/AWSIM GT matched samples", str(len(output_gt_position_error))),
        (
            "AWSIM GT - Mimosa mean position error",
            component_metrics(
                output_gt_position_error,
                lambda value: np.mean(value, axis=0),
                ("x", "y", "z"),
                "m",
            ),
        ),
        (
            "AWSIM GT - Mimosa position RMSE by axis",
            component_metrics(
                output_gt_position_error,
                lambda value: np.sqrt(np.mean(value**2, axis=0)),
                ("x", "y", "z"),
                "m",
            ),
        ),
        (
            "AWSIM GT - Mimosa maximum absolute position error",
            component_metrics(
                output_gt_position_error,
                lambda value: np.max(np.abs(value), axis=0),
                ("x", "y", "z"),
                "m",
            ),
        ),
        (
            "Mimosa output position RMSE against AWSIM GT",
            metric(
                np.linalg.norm(output_gt_position_error, axis=1),
                lambda value: np.sqrt(np.mean(value**2)),
            )
            + " m",
        ),
        (
            "Mimosa output mean position error against AWSIM GT",
            metric(np.linalg.norm(output_gt_position_error, axis=1), np.mean) + " m",
        ),
        (
            "Mimosa output maximum position error against AWSIM GT",
            metric(np.linalg.norm(output_gt_position_error, axis=1), np.max) + " m",
        ),
        (
            "AWSIM GT - Mimosa mean orientation error",
            component_metrics(
                output_gt_orientation_error,
                lambda value: np.mean(value, axis=0),
                ("roll", "pitch", "yaw"),
                "deg",
            ),
        ),
        (
            "AWSIM GT - Mimosa orientation RMSE by angle",
            component_metrics(
                output_gt_orientation_error,
                lambda value: np.sqrt(np.mean(value**2, axis=0)),
                ("roll", "pitch", "yaw"),
                "deg",
            ),
        ),
        (
            "AWSIM GT - Mimosa maximum absolute orientation error",
            component_metrics(
                output_gt_orientation_error,
                lambda value: np.max(np.abs(value), axis=0),
                ("roll", "pitch", "yaw"),
                "deg",
            ),
        ),
        (
            "Mimosa output mean absolute orientation error against AWSIM GT",
            metric(np.abs(output_gt_orientation_error), np.mean) + " deg",
        ),
        (
            "Mimosa output maximum absolute orientation error against AWSIM GT",
            metric(np.abs(output_gt_orientation_error), np.max) + " deg",
        ),
    ]
    return "<table>" + "".join(
        f"<tr><th>{html.escape(label)}</th><td>{html.escape(value)}</td></tr>"
        for label, value in rows
    ) + "</table>"


def write_report(
    output: Path,
    bag_path: Path,
    imu: StateSeries,
    lidar: StateSeries,
    gps: GpsSeries,
    mimosa_output: PoseSeries,
    awsim: PoseSeries,
    max_time_difference: float,
    gps_max_time_difference: float,
    gps_alignment: str,
    plotlyjs: str,
) -> None:
    git_branch, git_commit = git_revision()
    start_time = min(
        imu.time[0], lidar.time[0], gps.time[0], mimosa_output.time[0], awsim.time[0]
    )
    imu_time = imu.time - start_time
    lidar_time = lidar.time - start_time
    gps_time = gps.time - start_time
    output_time = mimosa_output.time - start_time
    awsim_time = awsim.time - start_time

    lidar_matches, imu_matches = nearest_matches(imu.time, lidar.time, max_time_difference)
    matched_time = lidar.time[lidar_matches] - start_time
    position_delta = lidar.position[lidar_matches] - imu.position[imu_matches]
    output_matches, imu_output_matches = nearest_matches(
        imu.time, mimosa_output.time, max_time_difference
    )
    output_orientation_error_time = mimosa_output.time[output_matches] - start_time

    imu_for_alignment, gps_for_alignment = nearest_matches(
        gps.time, imu.time, gps_max_time_difference
    )
    rotation, translation = estimate_position_alignment(
        imu.position[imu_for_alignment],
        gps.position_enu[gps_for_alignment],
        gps_alignment,
    )
    imu_position_aligned = transform_positions(imu.position, rotation, translation)
    lidar_position_aligned = transform_positions(lidar.position, rotation, translation)
    output_position_aligned = transform_positions(
        mimosa_output.position, rotation, translation
    )
    awsim_for_alignment, gps_for_awsim_alignment = nearest_matches(
        gps.time, awsim.time, gps_max_time_difference
    )
    awsim_rotation, awsim_translation = estimate_position_alignment(
        awsim.position[awsim_for_alignment],
        gps.position_enu[gps_for_awsim_alignment],
        gps_alignment,
    )
    awsim_position_aligned = transform_positions(
        awsim.position, awsim_rotation, awsim_translation
    )

    imu_rpy_aligned = aligned_rpy_deg(imu.quaternion, rotation)
    lidar_rpy_aligned = aligned_rpy_deg(lidar.quaternion, rotation)
    output_rpy_aligned = aligned_rpy_deg(mimosa_output.quaternion, rotation)
    awsim_rpy_aligned = aligned_rpy_deg(awsim.quaternion, awsim_rotation)
    orientation_delta = wrapped_angle_difference_deg(
        lidar_rpy_aligned[lidar_matches], imu_rpy_aligned[imu_matches]
    )
    output_orientation_error = wrapped_angle_difference_deg(
        output_rpy_aligned[output_matches], imu_rpy_aligned[imu_output_matches]
    )

    imu_position_aligned = zero_at_first_position(imu_position_aligned)
    lidar_position_aligned = zero_at_first_position(lidar_position_aligned)
    output_position_aligned = zero_at_first_position(output_position_aligned)
    gps_position = zero_at_first_position(gps.position_enu)
    awsim_position_aligned = zero_at_first_position(awsim_position_aligned)

    imu_gps_matches, gps_imu_matches = nearest_matches(
        gps.time, imu.time, gps_max_time_difference
    )
    lidar_gps_matches, gps_lidar_matches = nearest_matches(
        gps.time, lidar.time, gps_max_time_difference
    )
    output_gps_matches, gps_output_matches = nearest_matches(
        gps.time, mimosa_output.time, gps_max_time_difference
    )
    imu_gps_error = (
        imu_position_aligned[imu_gps_matches] - gps_position[gps_imu_matches]
    )
    lidar_gps_error = (
        lidar_position_aligned[lidar_gps_matches] - gps_position[gps_lidar_matches]
    )
    gps_output_error = (
        gps_position[gps_output_matches] - output_position_aligned[output_gps_matches]
    )
    output_gt_matches, gt_output_matches = nearest_matches(
        awsim.time, mimosa_output.time, max_time_difference
    )
    output_gt_position_error = (
        awsim_position_aligned[gt_output_matches]
        - output_position_aligned[output_gt_matches]
    )
    output_gt_orientation_error = wrapped_angle_difference_deg(
        awsim_rpy_aligned[gt_output_matches], output_rpy_aligned[output_gt_matches]
    )

    imu_error_time = imu.time[imu_gps_matches] - start_time
    lidar_error_time = lidar.time[lidar_gps_matches] - start_time
    output_error_time = mimosa_output.time[output_gps_matches] - start_time
    output_gt_error_time = mimosa_output.time[output_gt_matches] - start_time
    figures = []
    for component in range(3):
        figures.append(
            gps_component_figure(
                component,
                imu_time,
                imu_position_aligned,
                lidar_time,
                lidar_position_aligned,
                output_time,
                output_position_aligned,
                gps_time,
                gps_position,
                awsim_time,
                awsim_position_aligned,
                imu_error_time,
                imu_gps_error,
                lidar_error_time,
                lidar_gps_error,
                output_error_time,
                gps_output_error,
                output_gt_error_time,
                output_gt_position_error,
            )
        )

    for component in range(3):
        figures.append(
            orientation_component_figure(
                component,
                imu_time,
                imu_rpy_aligned,
                lidar_time,
                lidar_rpy_aligned,
                output_time,
                output_rpy_aligned,
                awsim_time,
                awsim_rpy_aligned,
                matched_time,
                orientation_delta,
                output_orientation_error_time,
                output_orientation_error,
                output_gt_error_time,
                output_gt_orientation_error,
            )
        )

    figures.append(
        gps_trajectory_figure(
            imu_position_aligned,
            lidar_position_aligned,
            output_position_aligned,
            gps_position,
            awsim_position_aligned,
        )
    )

    include_plotlyjs = True if plotlyjs == "inline" else "cdn"
    fragments = []
    for index, figure in enumerate(figures):
        fragments.append(
            pio.to_html(
                figure,
                full_html=False,
                include_plotlyjs=include_plotlyjs if index == 0 else False,
                config={"responsive": True, "displaylogo": False},
            )
        )

    summary = summary_html(
        bag_path,
        git_branch,
        git_commit,
        imu,
        lidar,
        gps,
        mimosa_output,
        awsim,
        len(lidar_matches),
        len(output_matches),
        position_delta,
        orientation_delta,
        gps_alignment,
        imu_gps_error,
        lidar_gps_error,
        gps_output_error,
        output_gt_position_error,
        output_gt_orientation_error,
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Mimosa state and output comparison report</title>
  <style>
    body {{ font-family: sans-serif; max-width: 1500px; margin: auto; padding: 1rem; }}
    h1, h2 {{ color: #263238; }}
    table {{ border-collapse: collapse; margin-bottom: 2rem; }}
    th, td {{ border: 1px solid #cfd8dc; padding: 0.45rem 0.8rem; text-align: left; }}
    th {{ background: #eceff1; }}
    .note {{ color: #455a64; }}
  </style>
</head>
<body>
  <h1>Mimosa IMU/LiDAR/output/GPS/AWSIM comparison</h1>
  <p class="note">The AWSIM PoseStamped stream is already expressed in ROS coordinates with Z-up.
  Mimosa and AWSIM frames are aligned independently to GPS using {gps_alignment.upper()}, and the
  same rotations are applied to their orientations. Mimosa output is paired to the nearest AWSIM
  ground-truth sample within {max_time_difference:.6g} s for position and orientation errors. GPS
  is converted from WGS84 to local ENU and matched within {gps_max_time_difference:.6g} s. Each
  position trace is relative to its own first sample, so every trace begins at (0, 0, 0). Errors
  therefore show relative-motion differences from the start of each source. Only Mimosa output,
  AWSIM ground truth, and their error are visible by default; the other traces can be enabled from
  the legends. Source traces retain their original timestamps.</p>
  <h2>Summary</h2>
  {summary}
  {''.join(fragments)}
</body>
</html>
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(document, encoding="utf-8")


def default_output_path(bag_path: Path) -> Path:
    if bag_path.is_dir():
        return bag_path / "factor_graph_report.html"
    return bag_path.with_name(f"{bag_path.stem}_factor_graph_report.html")


def find_m113_config() -> Path:
    script_path = Path(__file__).resolve()
    candidates = (
        script_path.parents[1] / M113_CONFIG_RELATIVE_PATH,
        script_path.parents[2] / "share/mimosa" / M113_CONFIG_RELATIVE_PATH,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not find the M113 configuration; searched: {searched}")


def copy_m113_config(output: Path, config: Path) -> Path:
    destination = output.parent / "m113.yaml"
    shutil.copy2(config, destination)
    return destination


def main() -> int:
    args = parse_args()
    if args.max_time_difference < 0.0:
        raise SystemExit("--max-time-difference must be non-negative")
    if args.gps_max_time_difference < 0.0:
        raise SystemExit("--gps-max-time-difference must be non-negative")
    try:
        m113_config = find_m113_config()
    except FileNotFoundError as exc:
        raise SystemExit(str(exc)) from exc
    load_plotly()

    try:
        series, gps, mimosa_output, awsim = read_bag(
            args.bag.expanduser(),
            args.imu_topic,
            args.lidar_topic,
            args.gps_topic,
            args.odometry_topic,
            args.awsim_topic,
        )
    except (FileNotFoundError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc

    imu = series[args.imu_topic]
    lidar = series[args.lidar_topic]
    if (
        len(imu) == 0
        or len(lidar) == 0
        or len(gps) == 0
        or len(mimosa_output) == 0
        or len(awsim) == 0
    ):
        raise SystemExit(
            "The bag contains no usable messages: "
            f"IMU={len(imu)}, LiDAR={len(lidar)}, valid GPS fixes={len(gps)}, "
            f"Mimosa output={len(mimosa_output)}, AWSIM ground truth={len(awsim)}"
        )

    output = args.output.expanduser() if args.output else default_output_path(args.bag.expanduser())
    write_report(
        output,
        args.bag.expanduser(),
        imu,
        lidar,
        gps,
        mimosa_output,
        awsim,
        args.max_time_difference,
        args.gps_max_time_difference,
        args.gps_alignment,
        args.plotlyjs,
    )
    copied_config = copy_m113_config(output, m113_config)
    print(f"Report written to: {output.resolve()}")
    print(f"M113 configuration copied to: {copied_config.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
