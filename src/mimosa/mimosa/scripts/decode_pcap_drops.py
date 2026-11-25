#!/usr/bin/env python3
"""Decode an Ouster pcap with ouster-sdk and report frame-id continuity,
per-frame completeness, and packet-count stats, to detect dropped
frames/packets. One-off diagnostic, not part of the mimosa pipeline."""
import sys
import time

from ouster.sdk.open_source import open_source


def main():
    path = sys.argv[1]
    src = open_source(path)
    expected_packets_per_frame = None
    try:
        si = src.sensor_info[0]
        cfg = si.config
        w = cfg.columns_per_frame if hasattr(cfg, "columns_per_frame") else None
    except Exception:
        w = None

    prev_frame_id = None
    n_frames = 0
    n_incomplete = 0
    frame_id_gaps = []
    packet_counts = []
    status_bad_cols_total = 0
    status_total_cols = 0
    t0 = time.time()

    for fs in src:
        scan = fs[0]
        n_frames += 1
        fid = scan.frame_id
        if prev_frame_id is not None and fid != prev_frame_id + 1:
            frame_id_gaps.append((prev_frame_id, fid))
        prev_frame_id = fid

        if not scan.complete():
            n_incomplete += 1

        pc = scan.packet_count
        packet_counts.append(pc)

        status = scan.status
        status_total_cols += status.size
        status_bad_cols_total += int((status == 0).sum())

        if n_frames % 500 == 0:
            print(f"  ...{n_frames} frames processed ({time.time()-t0:.1f}s)", flush=True)

    print(f"\npath: {path}")
    print(f"total frames decoded: {n_frames}")
    print(f"frame_id range: first processed prev={prev_frame_id}")
    print(f"frame_id gaps (dropped frames): {len(frame_id_gaps)}")
    print(f"  sample gaps: {frame_id_gaps[:20]}")
    print(f"incomplete frames (scan.complete()==False): {n_incomplete} / {n_frames}")
    if packet_counts:
        import statistics
        print(f"packet_count per frame: min={min(packet_counts)} max={max(packet_counts)} "
              f"mean={statistics.mean(packet_counts):.2f} mode={statistics.mode(packet_counts)}")
    print(f"bad/missing columns: {status_bad_cols_total} / {status_total_cols} "
          f"({100*status_bad_cols_total/max(status_total_cols,1):.4f}%)")
    print(f"elapsed: {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
