"""Run --simulate for isolated virtual hardware; default only reads existing WIT topics."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import socket
import struct
import subprocess
import sys
import threading
import time

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "rbnx-build/codegen/proto_gen"))


def wait_for(fn, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = fn()
            if result:
                return result
        except Exception:
            pass
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for Atlas/provider")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    if args.simulate:
        os.environ.update(RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_DOMAIN_ID="218", ROS_LOCALHOST_ONLY="1")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        endpoint = f"127.0.0.1:{sock.getsockname()[1]}"
    os.environ["ROBONIX_ATLAS"] = endpoint
    env = dict(os.environ, RBNX_INSTANCE_NAME="wit_imu_test")
    env.pop("RBNX_DEPLOY_MANAGED", None)
    from robonix_api import ATLAS
    import grpc
    import lifecycle_pb2
    from wit_imu.driver import TOPICS

    config = {"manage_driver": False}
    topics = dict(TOPICS)
    stop = threading.Event()
    send_frames = threading.Event()
    serial_fds = None
    writer = None
    node = executor = context = None
    processes = []
    report = {"simulated": args.simulate}
    log_dir = PKG / "rbnx-build/tests" / ("simulated" if args.simulate else "external")
    log_dir.mkdir(parents=True, exist_ok=True)
    try:
        if args.simulate:
            serial_fds = pty.openpty()
            topics = {key: f"/wit_primitive_test/{key}" for key in TOPICS}
            config = {"serial": {"port": os.ttyname(serial_fds[1]), "baud": 9600},
                      "frame_id": "other_body_imu", "topics": topics}

            def frame(kind, payload):
                data = bytes((0x55, kind)) + payload
                return data + bytes((sum(data) & 0xff,))

            burst = b"".join((frame(0x51, struct.pack("<hhhh", 0, 0, 2048, 0)),
                              frame(0x52, struct.pack("<hhhh", 0, 0, 1475, 0)),
                              frame(0x53, struct.pack("<hhhh", 0, 0, 16384, 0)),
                              frame(0x54, struct.pack("<hhhh", 100, 200, -300, 0)),
                              frame(0x56, struct.pack("<Ii", 101325, 1234))))

            def write_frames():
                while not stop.wait(0.05):
                    if send_frames.is_set():
                        os.write(serial_fds[0], burst)

            send_frames.set()
            writer = threading.Thread(target=write_frames)
            writer.start()

        source = subprocess.check_output(["rbnx", "path", "root"], text=True).strip()
        with (log_dir / "atlas.log").open("w") as atlas_log, (log_dir / "provider.log").open("w") as provider_log:
            processes.append(subprocess.Popen(["robonix-atlas", "--listen", endpoint, "--capabilities",
                f"{source}/capabilities,{PKG}/capabilities"], cwd=PKG, env=env, stdout=atlas_log, stderr=subprocess.STDOUT))
            wait_for(lambda: ATLAS.query_contract("robonix/primitive/imu/status"))
            processes.append(subprocess.Popen(["bash", "scripts/start.sh"], cwd=PKG, env=env,
                                               stdout=provider_log, stderr=subprocess.STDOUT))
            cap = wait_for(lambda: ATLAS.find_unique_capability(contract_id="robonix/lifecycle/driver", provider_id="wit_imu_test"))
            with ATLAS.connect_capability(consumer_id="wit_test", provider_id=cap.provider_id,
                                          contract_id=cap.contract_id, transport="grpc") as channel:
                with grpc.insecure_channel(channel.endpoint) as wire:
                    rpc = wire.unary_unary(cap.params.method,
                        request_serializer=lifecycle_pb2.Driver_Request.SerializeToString,
                        response_deserializer=lifecycle_pb2.Driver_Response.FromString)

                    def call(command, cfg=None, ok=True):
                        result = rpc(lifecycle_pb2.Driver_Request(command=command,
                                     config_json=json.dumps(cfg or {})), timeout=16)
                        assert result.ok == ok, result.error
                        return result

                    call(0, {"sentinel_timeout_s": -1}, ok=False)
                    states = []
                    for command in (0, 1, 2, 1):
                        states.append(call(command, config).state)
                    for key, expected in topics.items():
                        contract = f"robonix/primitive/imu/{key}"
                        ATLAS.find_unique_capability(contract_id=contract, provider_id=cap.provider_id, transport="ros2")
                        with ATLAS.connect_capability(consumer_id="wit_test", provider_id=cap.provider_id,
                                                      contract_id=contract, transport="ros2") as stream:
                            assert stream.endpoint == expected, stream.endpoint
                    if args.simulate:
                        import rclpy
                        from rclpy.context import Context
                        from rclpy.executors import SingleThreadedExecutor
                        from sensor_msgs.msg import Imu, FluidPressure
                        from geometry_msgs.msg import Vector3Stamped
                        from std_msgs.msg import Float64, String
                        context = Context()
                        rclpy.init(context=context)
                        executor = SingleThreadedExecutor(context=context)
                        node = rclpy.create_node("wit_data_test", context=context)
                        executor.add_node(node)
                        received = {}
                        for key, msg_type in (("imu", Imu), ("magnetic_raw", Vector3Stamped),
                                              ("pressure", FluidPressure), ("barometric_altitude", Float64), ("status", String)):
                            node.create_subscription(msg_type, topics[key],
                                lambda msg, name=key: received.__setitem__(name, msg), 10)
                        deadline = time.monotonic() + 8
                        while len(received) < 5 and time.monotonic() < deadline:
                            executor.spin_once(timeout_sec=0.1)
                        assert len(received) == 5, list(received)
                        sample = received["imu"]
                        assert sample.header.frame_id == "other_body_imu"
                        assert abs(sample.linear_acceleration.z - 9.80665) < 0.01
                        assert abs(sample.angular_velocity.z - math.pi / 2) < 0.01
                        assert abs(sample.orientation.z - math.sqrt(0.5)) < 0.01
                        assert received["magnetic_raw"].vector.z == -300
                        assert received["pressure"].fluid_pressure == 101325
                        assert received["barometric_altitude"].data == 12.34
                        status = json.loads(received["status"].data)
                        assert status["geographic_heading_valid"] is False
                        assert status["frame_id"] == "other_body_imu"
                        report["units_and_remapping"] = True
                        call(2)
                        send_frames.clear()
                        time.sleep(0.3)
                        call(0, {**config, "sentinel_timeout_s": 0.6})
                        call(1, ok=False)
                        assert not list((PKG / "rbnx-build/data").glob("wit-*.yaml")), "Failed activation leaked parameters"
                        report["missing_data_rejected_and_cleaned"] = True
                    else:
                        call(0)
                        failed = call(1, ok=False)
                        assert "external WIT publisher" in failed.error, failed.error
                        report["existing_driver_preserved"] = True
                    call(0, config)
                    send_frames.set()
                    call(1)
                    states.append(call(3).state)
                    report.update(passed=True, states=states, endpoints=topics)
                    (log_dir / "result.json").write_text(json.dumps(report, indent=2) + "\n")
                    print(json.dumps(report, indent=2))
    finally:
        stop.set()
        if writer is not None:
            writer.join(timeout=2)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        if serial_fds is not None:
            for fd in serial_fds:
                os.close(fd)


if __name__ == "__main__":
    main()
