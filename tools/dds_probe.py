#!/usr/bin/env python3
"""DDS 往返延迟探针: 以固定频率调用 Nav2 各服务器的 get_state 服务 (与行为树的动作握手走同一条路:
跨进程的 DDS 请求/应答)，统计往返时间。用法 (在实例的 ROS 环境里): python3 tools/dds_probe.py [秒数=60] [频率=10]
输出一行 JSON: 每个服务器的 n / 中位 / p90 / p99 / 最大 (ms)、超过 20/200/1000 ms 的次数、无应答次数"""
import json
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.executors import SingleThreadedExecutor

SERVERS = ["planner_server", "controller_server", "bt_navigator", "behavior_server"]


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    node = rclpy.create_node("agv_dds_probe")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    cli = {s: node.create_client(GetState, f"/{s}/get_state") for s in SERVERS}
    t_wait = time.time() + 30.0
    while time.time() < t_wait and not all(c.service_is_ready() for c in cli.values()):
        ex.spin_once(timeout_sec=0.2)
    ready = {s: c.service_is_ready() for s, c in cli.items()}
    discovery_s = round(30.0 - (t_wait - time.time()), 1)
    lat = {s: [] for s in SERVERS}
    lost = {s: 0 for s in SERVERS}
    t_end = time.time() + dur
    while time.time() < t_end:
        t_cycle = time.time()
        for s, c in cli.items():
            if not ready[s]:
                continue
            t0 = time.perf_counter()
            fut = c.call_async(GetState.Request())
            while not fut.done() and time.perf_counter() - t0 < 3.0:
                ex.spin_once(timeout_sec=0.05)
            if fut.done():
                lat[s].append((time.perf_counter() - t0) * 1000.0)
            else:
                lost[s] += 1
                fut.cancel()
        rest = 1.0 / hz - (time.time() - t_cycle)
        if rest > 0:
            time.sleep(rest)
    out = {"discovery_s": discovery_s, "servers": {}}
    for s in SERVERS:
        v = sorted(lat[s])
        if not v:
            out["servers"][s] = {"n": 0, "ready": ready[s], "lost": lost[s]}
            continue
        q = lambda p: round(v[min(len(v) - 1, int(len(v) * p))], 1)
        out["servers"][s] = {"n": len(v), "p50": q(0.5), "p90": q(0.9), "p99": q(0.99), "max": round(v[-1], 1),
                             "gt20": sum(x > 20 for x in v), "gt200": sum(x > 200 for x in v),
                             "gt1000": sum(x > 1000 for x in v), "lost": lost[s]}
    print("PROBE " + json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
