"""
Benchmark for Transform.lookup() — finds the max sustainable Hz of the
walkie_tf/get_transform service over rosbridge.

Runs N calls as fast as possible (no sleep) for a configurable duration, then
reports:
  • achieved Hz (calls/s, successful calls/s)
  • per-call latency: min / p50 / p90 / p99 / max
  • success / failure counts

Usage:
    python tests/benchmark_transform.py --ip 192.168.1.100
    python tests/benchmark_transform.py --ip 192.168.1.100 --source map --target base_link
    python tests/benchmark_transform.py --ip 192.168.1.100 --duration 30 --timeout 2.0
    python tests/benchmark_transform.py --ip 192.168.1.100 --namespace robot1
"""

import argparse
import statistics
import sys
import time

from walkie_sdk import WalkieRobot


def _percentile(sorted_data: list[float], pct: float) -> float:
    if not sorted_data:
        return float("nan")
    k = (len(sorted_data) - 1) * pct / 100.0
    lo, hi = int(k), min(int(k) + 1, len(sorted_data) - 1)
    return sorted_data[lo] + (sorted_data[hi] - sorted_data[lo]) * (k - lo)


def run_benchmark(
    robot: WalkieRobot,
    source: str,
    target: str,
    duration: float,
    call_timeout: float,
) -> None:
    latencies: list[float] = []
    failures = 0
    value_updates = 0   # how many times the position changed between consecutive calls
    prev_pos: tuple | None = None

    print(f"\nBenchmarking  '{source}' → '{target}'  for {duration:.0f}s  (Ctrl+C to stop early)\n")
    wall_start = time.perf_counter()
    deadline = wall_start + duration
    call_count = 0

    try:
        while True:
            now = time.perf_counter()
            if now >= deadline:
                break

            t0 = time.perf_counter()
            result = robot.transform.lookup(source, target, timeout=call_timeout)
            elapsed = time.perf_counter() - t0

            call_count += 1
            if result is None:
                failures += 1
            else:
                latencies.append(elapsed)
                pos = (
                    result["position"]["x"],
                    result["position"]["y"],
                    result["position"]["z"],
                )
                if prev_pos is not None and pos != prev_pos:
                    value_updates += 1
                prev_pos = pos

            # Progress line every 50 calls
            if call_count % 50 == 0:
                elapsed_wall = time.perf_counter() - wall_start
                hz = call_count / elapsed_wall if elapsed_wall > 0 else 0
                update_hz = value_updates / elapsed_wall if elapsed_wall > 0 else 0
                print(f"  {call_count:5d} calls  {elapsed_wall:5.1f}s  {hz:.1f} Hz  "
                      f"value_updates={value_updates} ({update_hz:.1f} Hz)  failures={failures}",
                      flush=True)

    except KeyboardInterrupt:
        print("\n  Interrupted early.")

    wall_elapsed = time.perf_counter() - wall_start
    successes = call_count - failures

    print(f"\n{'=' * 60}")
    print(f"  Benchmark results  ({source} → {target})")
    print(f"{'=' * 60}")
    print(f"  Duration          : {wall_elapsed:.2f} s")
    print(f"  Total calls       : {call_count}")
    print(f"  Successes         : {successes}")
    print(f"  Failures          : {failures}")
    print(f"  Success rate      : {100 * successes / call_count:.1f}%  (of {call_count})" if call_count else "  n/a")
    print()

    if wall_elapsed > 0:
        print(f"  Overall Hz        : {call_count / wall_elapsed:.2f}  (calls/s total)")
        print(f"  Successful Hz     : {successes / wall_elapsed:.2f}  (successful calls/s)")
        print(f"  Value update Hz   : {value_updates / wall_elapsed:.2f}  (distinct position changes/s)")
    print()

    if latencies:
        latencies.sort()
        print(f"  Latency (success calls only, s):")
        print(f"    min  : {min(latencies) * 1000:.1f} ms")
        print(f"    p50  : {_percentile(latencies, 50) * 1000:.1f} ms")
        print(f"    p90  : {_percentile(latencies, 90) * 1000:.1f} ms")
        print(f"    p99  : {_percentile(latencies, 99) * 1000:.1f} ms")
        print(f"    max  : {max(latencies) * 1000:.1f} ms")
        print(f"    mean : {statistics.mean(latencies) * 1000:.1f} ms")
        if len(latencies) > 1:
            print(f"    stdev: {statistics.stdev(latencies) * 1000:.1f} ms")
    else:
        print("  No successful calls — no latency stats.")

    print(f"{'=' * 60}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark Transform.lookup() Hz — requires walkie_tf service"
    )
    parser.add_argument("--ip",        default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--port",      type=int, default=9090, help="Rosbridge port (default: 9090)")
    parser.add_argument("--source",    default="map",       help="Source TF frame (default: map)")
    parser.add_argument("--target",    default="base_link", help="Target TF frame (default: base_link)")
    parser.add_argument("--duration",  type=float, default=15.0, help="Benchmark duration in s (default: 15)")
    parser.add_argument("--timeout",   type=float, default=2.0,  help="Per-call service timeout in s (default: 2.0)")
    parser.add_argument("--namespace", default="",           help="ROS namespace (default: none)")
    args = parser.parse_args()

    print(f"\nConnecting to rosbridge at {args.ip}:{args.port} ...")
    robot = WalkieRobot(
        ros_protocol="rosbridge",
        ip=args.ip,
        ros_port=args.port,
        camera_protocol="none",
        namespace=args.namespace,
    )
    print("Connected.")

    try:
        run_benchmark(
            robot=robot,
            source=args.source,
            target=args.target,
            duration=args.duration,
            call_timeout=args.timeout,
        )
    finally:
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
