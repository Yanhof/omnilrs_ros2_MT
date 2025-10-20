#!/usr/bin/env python3
# filter_bag_range.py
# Create a new ROS 2 bag while:
#   - excluding or including specific topics
#   - trimming by start/end time, duration, or by percentage of the original bag
#
# Requires: rosbag2_py (ROS 2 Humble or later), Python 3.8+
#
# Examples:
#   1) Exclude a topic and keep only the first 120 s:
#      python3 filter_bag_range.py -i /path/to/bag -o bag_trim --exclude /boxi/stim320/gyroscope_temperature --duration-sec 120
#
#   2) Include only two topics and keep from t=30 s to t=150 s:
#      python3 filter_bag_range.py -i /path/to/bag -o bag_trim --include /imu/data --include /tf --start-sec 30 --end-sec 150
#
#   3) Keep only the first 40% of the recording:
#      python3 filter_bag_range.py -i /path/to/bag -o bag_40pct --percent 40
#
#   4) Keep last 60 seconds:
#      python3 filter_bag_range.py -i /path/to/bag -o bag_last60 --duration-sec 60 --align-end
#
# Notes:
#  - Time parameters refer to the bag’s recorded timestamps (not wall time).
#  - If both --include and --exclude are empty, all topics are copied.
#  - If both include and exclude are given, include takes precedence (copy only includes, then drop excludes within that set).
#  - For percentage mode, the span is computed as first_ts + percent*(last-first).
#  - The script does two passes: quick scan for time bounds, then filtered copy.

import argparse
from csv import reader
import sys
import os
from typing import Set, Optional, Tuple

PRINT_EVERY = 1000  # adjust as you like


try:
    import rosbag2_py
except ImportError:
    print("[ERROR] rosbag2_py is not available. Make sure you’re in a ROS 2 (Humble+) Python environment.", file=sys.stderr)
    sys.exit(1)


def ns_to_sec(t_ns: int) -> float:
    return t_ns / 1e9


def sec_to_ns(t_s: float) -> int:
    return int(round(t_s * 1e9))


def compute_bag_time_bounds(uri: str, storage_id: str = "mcap") -> Tuple[int, int]:
    """Return (first_ts, last_ts) in nanoseconds by scanning the bag once."""
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id)
    conv_opts = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_opts, conv_opts)

    first_ts = None
    last_ts = None

    while reader.has_next():
        _, _, t = reader.read_next()
        if first_ts is None:
            first_ts = t
        last_ts = t

    if first_ts is None or last_ts is None:
        raise RuntimeError("Bag appears empty or unreadable.")

    return first_ts, last_ts


def resolve_time_window(
    bag_first_ns: int,
    bag_last_ns: int,
    start_sec: Optional[float],
    end_sec: Optional[float],
    duration_sec: Optional[float],
    percent: Optional[float],
    align_end: bool,
) -> Tuple[int, int]:
    """
    Compute [t_start_ns, t_end_ns] inclusive window within [bag_first_ns, bag_last_ns].

    Priority:
      - If percent is set: keep [bag_first, bag_first + percent*(span)].
      - Else if start/end/duration are set:
          * If align_end and duration: [bag_last - duration, bag_last]
          * Else if start only: [bag_first + start, bag_last]
          * Else if end only: [bag_first, bag_first + end]
          * Else if start+end: [bag_first + start, bag_first + end]
          * Else if duration only: [bag_first, bag_first + duration]
      - Else: full span.
    """
    span_ns = bag_last_ns - bag_first_ns
    if span_ns <= 0:
        raise RuntimeError("Non-positive bag span.")

    # Percentage mode
    if percent is not None:
        if not (0 < percent <= 100):
            raise ValueError("--percent must be in (0, 100].")
        end_ns = bag_first_ns + int(round((percent / 100.0) * span_ns))
        end_ns = min(end_ns, bag_last_ns)
        return bag_first_ns, end_ns

    # Absolute/relative seconds mode
    start_ns = bag_first_ns
    end_ns = bag_last_ns

    if align_end and duration_sec is not None:
        # Align to the tail
        duration_ns = sec_to_ns(duration_sec)
        start_ns = max(bag_last_ns - duration_ns, bag_first_ns)
        end_ns = bag_last_ns
        return start_ns, end_ns

    # From start of bag:
    if start_sec is not None:
        start_ns = bag_first_ns + sec_to_ns(start_sec)
    if end_sec is not None:
        end_ns = bag_first_ns + sec_to_ns(end_sec)

    if duration_sec is not None:
        # If start specified, use [start, start+duration]; else [bag_first, bag_first+duration]
        base = start_ns if start_sec is not None else bag_first_ns
        end_ns = base + sec_to_ns(duration_sec)

    # Clamp
    start_ns = max(start_ns, bag_first_ns)
    end_ns = min(end_ns, bag_last_ns)
    if end_ns < start_ns:
        raise ValueError("Computed end time precedes start time. Check your --start/--end/--duration values.")
    return start_ns, end_ns


def build_topic_sets(reader: rosbag2_py.SequentialReader,
                     includes: Set[str],
                     excludes: Set[str]) -> Set[str]:
    """
    Decide which topics to copy:
      - If includes non-empty: start from includes ∩ bag_topics
      - Else: start from all bag topics
      - Then remove any in excludes
    """
    bag_topics = {t.name for t in reader.get_all_topics_and_types()}
    if includes:
        selected = bag_topics.intersection(includes)
    else:
        selected = set(bag_topics)
    selected.difference_update(excludes)
    return selected


def copy_filtered(
    input_uri: str,
    output_uri: str,
    selected_topics: Set[str],
    t_start_ns: int,
    t_end_ns: int,
    storage_id: str = "mcap",
) -> None:
    reader = rosbag2_py.SequentialReader()
    storage_in = rosbag2_py.StorageOptions(uri=input_uri, storage_id=storage_id)
    conv = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_in, conv)

    writer = rosbag2_py.SequentialWriter()
    storage_out = rosbag2_py.StorageOptions(uri=output_uri, storage_id=storage_id)
    writer.open(storage_out, conv)

    # Mirror metadata (only for selected topics)
    for tm in reader.get_all_topics_and_types():
        if tm.name in selected_topics:
            writer.create_topic(tm)

    n_in = 0
    n_out = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        n_in += 1

        # EARLY EXIT: once past end of window, we can stop (SequentialReader is time-ordered)
        if t > t_end_ns:
            break

        if topic not in selected_topics:
            continue
        if t < t_start_ns:
            continue

        writer.write(topic, data, t)
        n_out += 1

        if n_out % PRINT_EVERY == 0:
            elapsed = ns_to_sec(t - t_start_ns)
            span = ns_to_sec(t_end_ns - t_start_ns)
            pct = 100.0 * elapsed / span if span > 0 else 0.0
            sys.stdout.write(f"\r[PROGRESS] {n_out} msgs written, ~{pct:.1f}% of time window")
            sys.stdout.flush()

    print()  # newline after loop

    print(f"[INFO] Input messages: {n_in}")
    print(f"[INFO] Written messages: {n_out}")
    print(f"[INFO] Time window kept: [{ns_to_sec(t_start_ns):.6f}s, {ns_to_sec(t_end_ns):.6f}s] "
          f"(duration {(ns_to_sec(t_end_ns - t_start_ns)):.6f}s)")
    print(f"[OK] New bag written: {output_uri}")


def main():
    p = argparse.ArgumentParser(description="Filter a ROS 2 bag by topics and time range / percentage.")
    p.add_argument("-i", "--input", required=True, help="Input bag folder (URI).")
    p.add_argument("-o", "--output", required=True, help="Output bag folder (URI).")
    p.add_argument("--storage-id", default="mcap", help="Storage plugin id (default: mcap).")

    grp_topics = p.add_argument_group("Topic selection")
    grp_topics.add_argument("--include", action="append", default=[],
                            help="Topic to include (repeatable). If omitted, all topics are considered.")
    grp_topics.add_argument("--exclude", action="append", default=[],
                            help="Topic to exclude (repeatable).")

    grp_time = p.add_argument_group("Time filtering (choose one strategy)")
    grp_time.add_argument("--start-sec", type=float, help="Start offset [s] from the beginning of the bag.")
    grp_time.add_argument("--end-sec", type=float, help="End offset [s] from the beginning of the bag.")
    grp_time.add_argument("--duration-sec", type=float, help="Duration to keep [s].")
    grp_time.add_argument("--percent", type=float, help="Keep the first PERCENT of the recording (0 < p ≤ 100).")
    grp_time.add_argument("--align-end", action="store_true",
                          help="If used with --duration-sec, keep the *last* D seconds instead of the first D seconds.")

    args = p.parse_args()

    includes = set(args.include or [])
    excludes = set(args.exclude or [])
    if not os.path.isabs(args.output):
        input_parent = os.path.dirname(os.path.abspath(args.input.rstrip("/")))
        output_uri = os.path.join(input_parent, args.output)
    else:
        output_uri = args.output

    # First pass: bag time bounds
    bag_first_ns, bag_last_ns = compute_bag_time_bounds(args.input, args.storage_id)
    print(f"[INFO] Bag time span: {ns_to_sec(bag_first_ns):.6f}s → {ns_to_sec(bag_last_ns):.6f}s "
          f"(Δ {ns_to_sec(bag_last_ns - bag_first_ns):.6f}s)")

    # Compute desired window
    t_start_ns, t_end_ns = resolve_time_window(
        bag_first_ns, bag_last_ns,
        args.start_sec, args.end_sec, args.duration_sec,
        args.percent, args.align_end
    )

    # Prepare reader to discover topics (second reader instance created in copy)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.input, storage_id=args.storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))

    selected_topics = build_topic_sets(reader, includes, excludes)
    if not selected_topics:
        print("[WARN] No topics selected (after include/exclude). Nothing to write.", file=sys.stderr)
        sys.exit(2)

    print("[INFO] Topics to copy:")
    for t in sorted(selected_topics):
        print(f"  - {t}")

    # Do the filtered copy
    copy_filtered(args.input, output_uri, selected_topics, t_start_ns, t_end_ns, storage_id=args.storage_id)


if __name__ == "__main__":
    main()
