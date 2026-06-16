"""
ROS 2 native benchmark for walkie_tf/get_transform service.

Calls the service synchronously as fast as possible using rclpy — no rosbridge,
no SDK, just a raw ROS 2 service client.  Run this ON the robot (or any machine
with ROS 2 and walkie_tf_interfaces installed) to get a baseline ceiling before
measuring the rosbridge path with benchmark_transform.py.

Usage:
    ros2 run <pkg> benchmark_transform_ros2   # (if installed)
    python tests/benchmark_transform_ros2.py
    python tests/benchmark_transform_ros2.py --source map --target base_link
    python tests/benchmark_transform_ros2.py --duration 30 --namespace robot1
    python tests/benchmark_transform_ros2.py --async    # use async (non-blocking) calls
    python tests/benchmark_transform_ros2.py --domain 23

Requirements:
    ROS 2 (Humble / Iron / Jazzy) + walkie_tf_interfaces installed.
    The walkie_tf tf_server node must be running:
        ros2 run walkie_tf tf_server
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

# Fail early with a readable message if rclpy isn't available.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
except ImportError:
    sys.exit(
        "[ERROR] rclpy not found.  Source your ROS 2 workspace first:\n"
        "    source /opt/ros/<distro>/setup.bash\n"
        "    source install/setup.bash   # (if using a local ws)"
    )

try:
    from walkie_tf_interfaces.srv import GetTransform
except ImportError:
    sys.exit(
        "[ERROR] walkie_tf_interfaces not found.\n"
        "    Build & source the workspace that contains walkie_tf_interfaces."
    )


def _percentile(sorted_data: list[float], pct: float) -> float:
    if not sorted_data:
        return float("nan")
    k = (len(sorted_data) - 1) * pct / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_data) - 1)
    return sorted_data[lo] + (sorted_data[hi] - sorted_data[lo]) * (k - lo)


def _print_stats(
    latencies: list[float],
    failures: int,
    wall_elapsed: float,
    source: str,
    target: str,
    mode: str,
    value_updates: int = 0,
) -> None:
    call_count = len(latencies) + failures
    successes = len(latencies)

    print(f"\n{'=' * 62}")
    print(f"  ROS 2 native benchmark  ({source} → {target})  [{mode}]")
    print(f"{'=' * 62}")
    print(f"  Duration          : {wall_elapsed:.2f} s")
    print(f"  Total calls       : {call_count}")
    print(f"  Successes         : {successes}")
    print(f"  Failures / None   : {failures}")
    if call_count:
        print(f"  Success rate      : {100 * successes / call_count:.1f}%")

    print()
    if wall_elapsed > 0:
        print(f"  Overall Hz        : {call_count / wall_elapsed:.2f}  calls/s (total)")
        print(f"  Successful Hz     : {successes / wall_elapsed:.2f}  calls/s (successful)")
        print(f"  Value update Hz   : {value_updates / wall_elapsed:.2f}  (distinct position changes/s)")
    print()

    if latencies:
        s = sorted(latencies)
        print("  Latency (ms)  — successful calls only:")
        print(f"    min  : {min(s) * 1000:.2f}")
        print(f"    p50  : {_percentile(s, 50) * 1000:.2f}")
        print(f"    p90  : {_percentile(s, 90) * 1000:.2f}")
        print(f"    p99  : {_percentile(s, 99) * 1000:.2f}")
        print(f"    max  : {max(s) * 1000:.2f}")
        print(f"    mean : {statistics.mean(s) * 1000:.2f}")
        if len(s) > 1:
            print(f"    stdev: {statistics.stdev(s) * 1000:.2f}")
    else:
        print("  No successful calls — no latency stats.")

    print(f"{'=' * 62}\n")


# ── Synchronous benchmark ─────────────────────────────────────────────────────

class SyncBenchmarkNode(Node):
    def __init__(self, service_name: str) -> None:
        super().__init__("walkie_tf_benchmark_sync")
        self._client = self.create_client(GetTransform, service_name)

    def wait_for_service(self, timeout: float = 10.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout)

    def call_once(self, source: str, target: str, call_timeout: float) -> tuple[bool, float, tuple | None]:
        req = GetTransform.Request()
        req.source_frame = source
        req.target_frame = target
        req.timeout_sec = call_timeout

        t0 = time.perf_counter()
        future = self._client.call_async(req)

        # Spin until done (synchronous feel, but avoids blocking the executor)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        deadline = time.perf_counter() + call_timeout + 1.0
        while not future.done():
            executor.spin_once(timeout_sec=0.001)
            if time.perf_counter() > deadline:
                return False, time.perf_counter() - t0, None
        elapsed = time.perf_counter() - t0

        try:
            result = future.result()
        except Exception:
            return False, elapsed, None

        if not result.success:
            return False, elapsed, None
        return True, elapsed, (result.x, result.y, result.z)


def run_sync_benchmark(
    service_name: str,
    source: str,
    target: str,
    duration: float,
    call_timeout: float,
) -> None:
    rclpy.init()
    node = SyncBenchmarkNode(service_name)

    print(f"  Waiting for service '{service_name}' ...")
    if not node.wait_for_service(timeout=10.0):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(f"[ERROR] Service '{service_name}' not available after 10 s.")
    print("  Service ready.\n")

    latencies: list[float] = []
    failures = 0
    call_count = 0
    value_updates = 0
    prev_pos: tuple | None = None

    print(f"  Benchmarking '{source}' → '{target}'  for {duration:.0f}s  (Ctrl+C to stop early)\n")
    wall_start = time.perf_counter()
    deadline = wall_start + duration

    try:
        while time.perf_counter() < deadline:
            ok, elapsed, pos = node.call_once(source, target, call_timeout)
            call_count += 1
            if ok and pos is not None:
                latencies.append(elapsed)
                if prev_pos is not None and pos != prev_pos:
                    value_updates += 1
                prev_pos = pos
            else:
                failures += 1

            if call_count % 50 == 0:
                wall_now = time.perf_counter() - wall_start
                hz = call_count / wall_now if wall_now > 0 else 0.0
                update_hz = value_updates / wall_now if wall_now > 0 else 0.0
                print(f"  {call_count:5d} calls  {wall_now:5.1f}s  {hz:.1f} Hz  "
                      f"value_updates={value_updates} ({update_hz:.1f} Hz)  failures={failures}",
                      flush=True)

    except KeyboardInterrupt:
        print("\n  Interrupted early.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    wall_elapsed = time.perf_counter() - wall_start
    _print_stats(latencies, failures, wall_elapsed, source, target,
                 mode="sync via call_async+spin", value_updates=value_updates)


# ── Async benchmark (concurrent calls via executor) ───────────────────────────

class AsyncBenchmarkNode(Node):
    def __init__(self, service_name: str) -> None:
        super().__init__("walkie_tf_benchmark_async")
        self._client = self.create_client(GetTransform, service_name)
        self._latencies: list[float] = []
        self._failures = 0
        self._value_updates = 0
        self._prev_pos: tuple | None = None
        self._in_flight = 0
        self._max_in_flight: int = 4  # pipeline depth — tune if needed
        self._source = ""
        self._target = ""
        self._call_timeout = 2.0
        self._deadline = 0.0

    def wait_for_service(self, timeout: float = 10.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout)

    def _send_one(self) -> None:
        req = GetTransform.Request()
        req.source_frame = self._source
        req.target_frame = self._target
        req.timeout_sec = self._call_timeout
        t0 = time.perf_counter()
        future = self._client.call_async(req)
        self._in_flight += 1

        def _on_done(fut):
            elapsed = time.perf_counter() - t0
            self._in_flight -= 1
            try:
                res = fut.result()
                if res.success:
                    self._latencies.append(elapsed)
                    pos = (res.x, res.y, res.z)
                    if self._prev_pos is not None and pos != self._prev_pos:
                        self._value_updates += 1
                    self._prev_pos = pos
                else:
                    self._failures += 1
            except Exception:
                self._failures += 1
            # Immediately fire the next call if still within window
            if time.perf_counter() < self._deadline:
                self._send_one()

        future.add_done_callback(_on_done)

    def run(
        self,
        source: str,
        target: str,
        duration: float,
        call_timeout: float,
        pipeline: int = 4,
    ) -> None:
        self._source = source
        self._target = target
        self._call_timeout = call_timeout
        self._max_in_flight = pipeline
        self._deadline = time.perf_counter() + duration

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        # Seed the pipeline
        for _ in range(pipeline):
            self._send_one()

        progress_t = time.perf_counter()
        wall_start = time.perf_counter()

        try:
            while time.perf_counter() < self._deadline or self._in_flight > 0:
                executor.spin_once(timeout_sec=0.001)
                now = time.perf_counter()
                if now - progress_t >= 2.0:
                    elapsed = now - wall_start
                    total = len(self._latencies) + self._failures
                    hz = total / elapsed if elapsed > 0 else 0.0
                    update_hz = self._value_updates / elapsed if elapsed > 0 else 0.0
                    print(f"  {total:5d} calls  {elapsed:5.1f}s  {hz:.1f} Hz  "
                          f"value_updates={self._value_updates} ({update_hz:.1f} Hz)  "
                          f"failures={self._failures}  in_flight={self._in_flight}",
                          flush=True)
                    progress_t = now
        except KeyboardInterrupt:
            print("\n  Interrupted early.")


def run_async_benchmark(
    service_name: str,
    source: str,
    target: str,
    duration: float,
    call_timeout: float,
    pipeline: int,
) -> None:
    rclpy.init()
    node = AsyncBenchmarkNode(service_name)

    print(f"  Waiting for service '{service_name}' ...")
    if not node.wait_for_service(timeout=10.0):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(f"[ERROR] Service '{service_name}' not available after 10 s.")
    print(f"  Service ready.  pipeline depth = {pipeline}\n")
    print(f"  Benchmarking '{source}' → '{target}'  for {duration:.0f}s  (Ctrl+C to stop early)\n")

    wall_start = time.perf_counter()
    node.run(source, target, duration, call_timeout, pipeline)
    wall_elapsed = time.perf_counter() - wall_start

    latencies = node._latencies
    failures = node._failures
    value_updates = node._value_updates
    node.destroy_node()
    rclpy.shutdown()

    _print_stats(latencies, failures, wall_elapsed, source, target,
                 mode=f"async pipeline={pipeline}", value_updates=value_updates)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ROS 2 native benchmark for walkie_tf/get_transform service"
    )
    parser.add_argument("--source",    default="map",       help="Source TF frame (default: map)")
    parser.add_argument("--target",    default="base_link", help="Target TF frame (default: base_link)")
    parser.add_argument("--duration",  type=float, default=15.0, help="Benchmark duration in s (default: 15)")
    parser.add_argument("--timeout",   type=float, default=2.0,  help="Per-call service timeout in s (default: 2.0)")
    parser.add_argument("--namespace", default="",  help="ROS namespace prefix (default: none)")
    parser.add_argument("--service",   default="",  help="Full service name override (default: [ns/]get_transform)")
    parser.add_argument("--domain",    type=int, default=None, help="ROS_DOMAIN_ID override (default: env / 0)")
    parser.add_argument("--async",     dest="use_async", action="store_true",
                        help="Use async pipelined calls instead of sequential")
    parser.add_argument("--pipeline",  type=int, default=4,
                        help="Number of concurrent in-flight calls in async mode (default: 4)")
    args = parser.parse_args()

    if args.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)

    # Build service name
    if args.service:
        service_name = args.service
    elif args.namespace:
        ns = args.namespace.strip("/")
        service_name = f"/{ns}/get_transform"
    else:
        service_name = "get_transform"

    print(f"\nROS_DOMAIN_ID : {os.environ.get('ROS_DOMAIN_ID', '0 (default)')}")
    print(f"Service name  : {service_name}")
    print(f"Mode          : {'async (pipelined)' if args.use_async else 'sync (sequential)'}")

    if args.use_async:
        run_async_benchmark(
            service_name=service_name,
            source=args.source,
            target=args.target,
            duration=args.duration,
            call_timeout=args.timeout,
            pipeline=args.pipeline,
        )
    else:
        run_sync_benchmark(
            service_name=service_name,
            source=args.source,
            target=args.target,
            duration=args.duration,
            call_timeout=args.timeout,
        )


if __name__ == "__main__":
    main()
