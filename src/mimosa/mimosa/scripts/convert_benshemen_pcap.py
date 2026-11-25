#!/usr/bin/env python3
"""Convert a raw Ouster pcap (lidar + imu UDP streams) into a ROS2 bag with
decoded sensor_msgs/PointCloud2 + sensor_msgs/Imu topics, matching the same
field layout mimosa already consumes from the enwide/rellis bags (since
ouster-cli's own `save --ros2` only writes raw undecoded packet topics, not
usable by mimosa directly). One-off conversion tool for the BenShemen Hatal
dataset, not a generic pipeline."""
import argparse
import time
from pathlib import Path

import numpy as np

import ouster.sdk.core as core
import ouster.sdk.pcap as sdk_pcap
from ouster.sdk.open_source import open_source
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "noise", "range"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u4", "<u2", "u1", "<u2", "<u4"],
    "offsets": [0, 4, 8, 16, 20, 24, 26, 28, 32],
    "itemsize": 48,
})


def to_time(ns):
    ns = int(ns)
    return ns // 1_000_000_000, ns % 1_000_000_000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("output")
    ap.add_argument("--lidar-topic", default="/os1_cloud_node/points")
    ap.add_argument("--imu-topic", default="/os1_cloud_node/imu")
    ap.add_argument("--lidar-frame", default="os1_lidar")
    ap.add_argument("--imu-frame", default="os1_imu")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = no limit")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_JAZZY)
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
    PointField = ts.types["sensor_msgs/msg/PointField"]
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]

    field_specs = [
        ("x", 0, PointField.FLOAT32, 1),
        ("y", 4, PointField.FLOAT32, 1),
        ("z", 8, PointField.FLOAT32, 1),
        ("intensity", 16, PointField.FLOAT32, 1),
        ("t", 20, PointField.UINT32, 1),
        ("reflectivity", 24, PointField.UINT16, 1),
        ("ring", 26, PointField.UINT8, 1),
        ("noise", 28, PointField.UINT16, 1),
        ("range", 32, PointField.UINT32, 1),
    ]
    fields = [PointField(name=n, offset=o, datatype=d, count=c) for n, o, d, c in field_specs]

    out_path = Path(args.output)
    if out_path.exists():
        raise SystemExit(f"output already exists: {out_path}")

    writer = Writer(out_path, version=9)
    writer.open()
    lidar_conn = writer.add_connection(args.lidar_topic, PointCloud2.__msgtype__, typestore=ts)
    imu_conn = writer.add_connection(args.imu_topic, Imu.__msgtype__, typestore=ts)

    t0 = time.time()
    src = open_source(args.pcap)
    si = src.sensor_info[0]
    xyzlut = core.XYZLut(si)

    n_frames = 0
    for fs in src:
        scan = fs[0]
        h, w = scan.h, scan.w
        xyz = xyzlut(scan).astype(np.float32)
        signal = scan.field(core.ChanField.SIGNAL).astype(np.float32)
        reflectivity = scan.field(core.ChanField.REFLECTIVITY).astype(np.uint16)
        near_ir = scan.field(core.ChanField.NEAR_IR).astype(np.uint16)
        rng = scan.field(core.ChanField.RANGE).astype(np.uint32)
        ring = np.broadcast_to(np.arange(h, dtype=np.uint8)[:, None], (h, w))
        col_ts = scan.timestamp.astype(np.int64)
        valid_cols = scan.status != 0
        # An incomplete frame can have leading/trailing/gap columns with
        # timestamp==0 (never received) -- anchor on the first column that
        # actually has valid data, not raw index 0, or a single bad column
        # corrupts this whole frame's (and the bag's) timestamps.
        ref_ts = int(col_ts[int(np.argmax(valid_cols))]) if valid_cols.any() else int(col_ts[0])
        scan_t0 = ref_ts
        t_rel = np.clip(col_ts - ref_ts, 0, None).astype(np.uint32)
        t_arr = np.broadcast_to(t_rel[None, :], (h, w))

        buf = np.zeros((h, w), dtype=POINT_DTYPE)
        buf["x"] = xyz[:, :, 0]
        buf["y"] = xyz[:, :, 1]
        buf["z"] = xyz[:, :, 2]
        buf["intensity"] = signal
        buf["t"] = t_arr
        buf["reflectivity"] = reflectivity
        buf["ring"] = ring
        buf["noise"] = near_ir
        buf["range"] = rng

        sec, nsec = to_time(scan_t0)
        header = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=args.lidar_frame)
        msg = PointCloud2(
            header=header, height=h, width=w, fields=fields,
            is_bigendian=False, point_step=48, row_step=48 * w,
            data=buf.reshape(-1).view(np.uint8), is_dense=False,
        )
        raw = ts.serialize_cdr(msg, PointCloud2.__msgtype__)
        writer.write(lidar_conn, scan_t0, raw)
        n_frames += 1
        if n_frames % 500 == 0:
            print(f"  ...{n_frames} lidar frames ({time.time()-t0:.1f}s)", flush=True)
        if args.max_frames and n_frames >= args.max_frames:
            break

    n_imu = 0
    pkt_src = sdk_pcap.PcapPacketSource(args.pcap)
    for _idx, pk in pkt_src:
        if not isinstance(pk, core.ImuPacket):
            continue
        ts_ns = int(pk.timestamp()[0])
        accel = pk.accel()[0]
        gyro = pk.gyro()[0]
        sec, nsec = to_time(ts_ns)
        header = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=args.imu_frame)
        msg = Imu(
            header=header,
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0),
            orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
            angular_velocity=Vector3(x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2])),
            angular_velocity_covariance=np.array(
                [0.0006, 0, 0, 0, 0.0006, 0, 0, 0, 0.0006], dtype=np.float64),
            linear_acceleration=Vector3(x=float(accel[0]), y=float(accel[1]), z=float(accel[2])),
            linear_acceleration_covariance=np.array(
                [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01], dtype=np.float64),
        )
        raw = ts.serialize_cdr(msg, Imu.__msgtype__)
        writer.write(imu_conn, ts_ns, raw)
        n_imu += 1
        if args.max_frames and n_imu >= args.max_frames * 20:
            break

    writer.close()
    print(f"DONE frames={n_frames} imu={n_imu} elapsed={time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
