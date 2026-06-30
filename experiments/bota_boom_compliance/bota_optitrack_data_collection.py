#!/usr/bin/env python3
"""Collect BOTA force/torque with body-34 deflection from an initial OptiTrack pose."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import socket
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

BOTA_PYTHON = Path("/home/stanleywang/miniconda3/envs/BOTA/bin/python3.12")
if BOTA_PYTHON.exists() and Path(sys.executable).resolve() != BOTA_PYTHON.resolve():
    print(f"Restarting with BOTA Python: {BOTA_PYTHON}", flush=True)
    os.execv(str(BOTA_PYTHON), [str(BOTA_PYTHON), __file__, *sys.argv[1:]])

import bota_driver
from natnet import NatNetClient, Version
from natnet.packet_buffer import PacketBuffer

from pipeline_paths import raw_data_path

RIGID_ID = 34
RATE_HZ = 10.0
ORIGIN_DELAY_S = 5.0
MAX_OPTI_AGE_MS = 100.0
BOOM_LENGTH_M = 1.5

CAP_NET_ADMIN = 12
CAP_NET_RAW = 13
STOP = False


def patch_natnet_string_decoder() -> None:
    original = PacketBuffer.read_string
    if getattr(original, "_bota_optitrack_patched", False):
        return

    def read_string_lossy(self, max_length=None, static_length=False):
        if max_length is None:
            data_slice = self._PacketBuffer__data[self.pointer :]
        else:
            data_slice = self._PacketBuffer__data[self.pointer : self.pointer + max_length]
        str_enc, _separator, _remainder = bytes(data_slice).partition(b"\0")
        str_dec = str_enc.decode("utf-8", errors="replace")
        if static_length:
            assert max_length is not None
            self.pointer += max_length
        else:
            self.pointer += len(str_enc) + 1
        return str_dec

    read_string_lossy._bota_optitrack_patched = True
    PacketBuffer.read_string = read_string_lossy


patch_natnet_string_decoder()


def on_signal(_signum, _frame):
    global STOP
    STOP = True


def has_effective_cap(cap_bit: int) -> bool:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("CapEff:"):
                return bool(int(line.split()[1], 16) & (1 << cap_bit))
    except OSError:
        pass
    return False


def link_is_up(iface: str) -> bool:
    try:
        return (Path("/sys/class/net") / iface / "carrier").read_text().strip() == "1"
    except OSError:
        return False


def auto_bota_iface() -> str | None:
    try:
        names = sorted(p.name for p in Path("/sys/class/net").iterdir())
    except OSError:
        return None
    for name in names:
        if name in {"lo", "docker0"} or name.startswith(("wl", "br-", "veth", "p2p-")):
            continue
        if link_is_up(name):
            return name
    return None


def local_ip_for_server(server_ip: str) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((server_ip, 1510))
        return sock.getsockname()[0]
    finally:
        sock.close()


def make_bota_config(args: argparse.Namespace) -> str:
    driver_config = {
        "product_name": args.bota_product,
        "serial_number": args.bota_serial,
        "communication_interface_name": "CANopen_over_EtherCAT_gen0",
        "communication_interface_params": {"network_interface": args.bota_iface},
        "sensor_operation_params": {
            "sinc_length": args.sinc_length,
            "wrench_offset": {"fx": 0.0, "fy": 0.0, "fz": 0.0, "tx": 0.0, "ty": 0.0, "tz": 0.0},
        },
        "driver_operation_params": {"runtime_verbosity": args.verbose_bota},
    }
    handle = tempfile.NamedTemporaryFile("w", suffix="_bota_ethercat.json", delete=False)
    with handle:
        json.dump({"driver_config": driver_config}, handle, indent=2)
        handle.write("\n")
    return handle.name


@dataclass
class OptiSample:
    local_ns: int
    frame: int | None
    motive_timestamp: float | None
    rigid_id: int
    seen: bool | None
    px: float | None
    py: float | None
    pz: float | None
    qx: float | None
    qy: float | None
    qz: float | None
    qw: float | None


class OptiTrackReceiver:
    def __init__(self, server_ip: str, client_ip: str, rigid_id: int, data_port: int, command_port: int, use_multicast: bool):
        self.rigid_id = rigid_id
        self.lock = threading.Lock()
        self.sample: OptiSample | None = None
        self.error: Exception | None = None
        self.client = NatNetClient(
            server_ip_address=server_ip,
            local_ip_address=client_ip,
            command_port=command_port,
            data_port=data_port,
            use_multicast=use_multicast,
        )
        self.client._NatNetClient__current_protocol_version = Version(4, 3)
        self.client.on_data_frame_received_event.handlers.append(self._on_frame)

    def _on_frame(self, frame):
        local_ns = time.monotonic_ns()
        body = None
        for rb in frame.rigid_bodies or ():
            if rb.id_num == self.rigid_id:
                body = rb
                break
        if body is None:
            sample = OptiSample(local_ns, frame.prefix.frame_number, frame.suffix.timestamp, self.rigid_id, None, None, None, None, None, None, None, None)
        else:
            sample = OptiSample(
                local_ns=local_ns,
                frame=frame.prefix.frame_number,
                motive_timestamp=frame.suffix.timestamp,
                rigid_id=body.id_num,
                seen=body.tracking_valid,
                px=body.pos[0], py=body.pos[1], pz=body.pos[2],
                qx=body.rot[0], qy=body.rot[1], qz=body.rot[2], qw=body.rot[3],
            )
        with self.lock:
            self.sample = sample

    def start(self):
        self.client.connect(timeout=5.0)
        print(f"OptiTrack connected: protocol={self.client.protocol_version} server={self.client.server_info}")
        self.client.run_async()

    def latest(self) -> OptiSample | None:
        with self.lock:
            return self.sample

    def stop(self):
        try:
            self.client.shutdown()
        except Exception as exc:
            self.error = exc


def quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    return tuple(v / norm for v in q)


def quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_matrix(q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    qx, qy, qz, qw = quat_normalize(q)
    return (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )


def mat_transpose_vec_mul(matrix, vec: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vec[0] + matrix[1][0] * vec[1] + matrix[2][0] * vec[2],
        matrix[0][1] * vec[0] + matrix[1][1] * vec[1] + matrix[2][1] * vec[2],
        matrix[0][2] * vec[0] + matrix[1][2] * vec[1] + matrix[2][2] * vec[2],
    )


def sample_pose(sample: OptiSample) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    if None in (sample.px, sample.py, sample.pz, sample.qx, sample.qy, sample.qz, sample.qw):
        raise ValueError("OptiTrack sample does not contain a full pose")
    return (sample.px, sample.py, sample.pz), quat_normalize((sample.qx, sample.qy, sample.qz, sample.qw))


def fresh_pose(opti: OptiTrackReceiver, max_age_ms: float) -> tuple[OptiSample, float] | None:
    now_ns = time.monotonic_ns()
    sample = opti.latest()
    if sample is None or sample.px is None:
        return None
    age_ms = (now_ns - sample.local_ns) / 1e6
    if age_ms > max_age_ms:
        return None
    return sample, age_ms


def capture_origin(opti: OptiTrackReceiver, bota, delay_s: float, max_age_ms: float) -> tuple[OptiSample, tuple[float, float, float], tuple[float, float, float]]:
    print(f"Waiting {delay_s:.1f}s before capturing origin pose for rigid body {RIGID_ID}...")
    end = time.monotonic() + delay_s
    while not STOP and time.monotonic() < end:
        time.sleep(0.05)

    timeout = time.monotonic() + 5.0
    while not STOP and time.monotonic() < timeout:
        fresh = fresh_pose(opti, max_age_ms)
        if fresh is not None:
            sample, age_ms = fresh
            print(
                f"Origin frame collected: frame={sample.frame} age={age_ms:.1f}ms "
                f"pos=({sample.px:.6f},{sample.py:.6f},{sample.pz:.6f}) "
                f"quat=({sample.qx:.6f},{sample.qy:.6f},{sample.qz:.6f},{sample.qw:.6f})"
            )
            frame = bota.read_frame()
            force_zero = tuple(float(value) for value in frame.force)
            torque_zero = tuple(float(value) for value in frame.torque)
            print(
                f"BOTA zero collected: "
                f"F=({force_zero[0]:.6f},{force_zero[1]:.6f},{force_zero[2]:.6f}) "
                f"T=({torque_zero[0]:.6f},{torque_zero[1]:.6f},{torque_zero[2]:.6f})"
            )
            return sample, force_zero, torque_zero
        time.sleep(0.02)
    raise RuntimeError("could not capture a fresh OptiTrack origin pose")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect BOTA force data with body-34 compliance deflection.")
    parser.add_argument("--server-ip", default="172.24.68.77", help="Field Bay Motive/NatNet server IP")
    parser.add_argument("--client-ip", default=None, help="Local SRC network IP; auto-detected by default")
    parser.add_argument("--rigid-id", type=int, default=RIGID_ID, help="OptiTrack rigid body ID")
    parser.add_argument("--data-port", type=int, default=1511, help="NatNet data port")
    parser.add_argument("--multicast", action="store_true", help="Use NatNet multicast data mode instead of unicast")
    parser.add_argument("--command-port", type=int, default=1510, help="NatNet command port")
    parser.add_argument("--bota-iface", default=auto_bota_iface(), help="BOTA EtherCAT interface")
    parser.add_argument("--bota-product", default="BFT-ROKS-ECAT-M8")
    parser.add_argument("--bota-serial", default="SN000856")
    parser.add_argument("--sinc-length", type=int, default=100)
    parser.add_argument("--tare", dest="tare", action="store_true", default=True, help="Tare BOTA before logging (default)")
    parser.add_argument("--no-tare", dest="tare", action="store_false", help="Do not tare BOTA")
    parser.add_argument("--origin-delay", type=float, default=ORIGIN_DELAY_S, help="Seconds to wait before capturing the origin pose")
    parser.add_argument("--max-opti-age-ms", type=float, default=MAX_OPTI_AGE_MS, help="Maximum OptiTrack sample age accepted as fresh")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to log after origin capture; 0 means until Ctrl+C")
    parser.add_argument("--rate", type=float, default=RATE_HZ, help="BOTA polling/logging loop rate in Hz")
    parser.add_argument("--print-rate", type=float, default=5.0, help="Terminal status rate in Hz")
    parser.add_argument("--boom-length-m", type=float, default=BOOM_LENGTH_M, help="Boom length used for automatic CSV naming")
    parser.add_argument("--csv", default=None, help="Output CSV path")
    parser.add_argument("--verbose-bota", action="store_true")
    args = parser.parse_args()
    if not args.bota_iface:
        parser.error("no live BOTA Ethernet interface detected; pass --bota-iface enp47s0")
    if args.client_ip is None:
        args.client_ip = local_ip_for_server(args.server_ip)
    csv_path = raw_data_path(args.boom_length_m) if args.csv is None else Path(args.csv).expanduser()
    if not csv_path.is_absolute():
        csv_path = Path(__file__).resolve().parent / csv_path
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    args.csv = str(csv_path)
    return args


def main() -> int:
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    args = parse_args()

    print(f"BOTA EtherCAT interface: {args.bota_iface}")
    print(f"OptiTrack server/client: {args.server_ip} / {args.client_ip}")
    print(f"OptiTrack data mode: {'multicast' if args.multicast else 'unicast'}")
    print(f"Target rigid body ID: {args.rigid_id}")
    print(f"CSV: {args.csv}")
    if not has_effective_cap(CAP_NET_RAW) or not has_effective_cap(CAP_NET_ADMIN):
        print("WARN: BOTA Python lacks cap_net_raw/cap_net_admin; EtherCAT may fail", file=sys.stderr)

    opti = None
    bota = None
    config_path = make_bota_config(args)
    try:
        opti = OptiTrackReceiver(args.server_ip, args.client_ip, args.rigid_id, args.data_port, args.command_port, args.multicast)
        opti.start()

        bota = bota_driver.BotaDriver(config_path)
        print(f"BotaDriver: {bota.get_driver_version_string() if hasattr(bota, 'get_driver_version_string') else 'unknown'}")
        if not bota.configure():
            raise RuntimeError("BOTA configure() failed")
        if args.tare and not bota.tare():
            raise RuntimeError("BOTA tare() failed")
        if not bota.activate():
            raise RuntimeError("BOTA activate() failed")

        origin, force_zero, torque_zero = capture_origin(opti, bota, args.origin_delay, args.max_opti_age_ms)
        origin_pos, origin_quat = sample_pose(origin)
        origin_rot = quat_to_matrix(origin_quat)
        origin_inv_quat = quat_conjugate(origin_quat)

        fieldnames = [
            "local_ns", "elapsed_s", "bota_timestamp", "fx", "fy", "fz", "tx", "ty", "tz", "bota_temperature", "bota_status",
            "force_zero_fx", "force_zero_fy", "force_zero_fz", "torque_zero_tx", "torque_zero_ty", "torque_zero_tz",
            "opti_local_ns", "opti_dt_ms", "opti_frame", "opti_timestamp", "opti_rigid_id", "opti_seen",
            "opti_px", "opti_py", "opti_pz", "opti_qx", "opti_qy", "opti_qz", "opti_qw",
            "origin_frame", "origin_px", "origin_py", "origin_pz", "origin_qx", "origin_qy", "origin_qz", "origin_qw",
            "deflection_x_m", "deflection_y_m", "deflection_z_m", "deflection_norm_m",
            "rel_qx", "rel_qy", "rel_qz", "rel_qw", "reorientation_angle_deg",
        ]
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            start_ns = time.monotonic_ns()
            last_print = 0.0
            period = 1.0 / max(args.rate, 1e-6)
            while not STOP:
                loop_start = time.perf_counter()
                now_ns = time.monotonic_ns()
                elapsed = (now_ns - start_ns) / 1e9
                if args.duration > 0 and elapsed >= args.duration:
                    break

                frame = bota.read_frame()
                force = tuple(float(frame.force[i]) - force_zero[i] for i in range(3))
                torque = tuple(float(frame.torque[i]) - torque_zero[i] for i in range(3))
                status = getattr(frame, "status", "")
                sample = opti.latest()
                dt_ms = None if sample is None else (now_ns - sample.local_ns) / 1e6

                rel_pos = ("", "", "")
                rel_norm = ""
                rel_quat = ("", "", "", "")
                rel_angle_deg = ""
                if sample is not None and sample.px is not None:
                    pos, quat = sample_pose(sample)
                    world_delta = (pos[0] - origin_pos[0], pos[1] - origin_pos[1], pos[2] - origin_pos[2])
                    rel_pos = mat_transpose_vec_mul(origin_rot, world_delta)
                    rel_norm = math.sqrt(sum(v * v for v in rel_pos))
                    rel_quat = quat_normalize(quat_multiply(origin_inv_quat, quat))
                    rel_angle_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(rel_quat[3])))))

                row = {
                    "local_ns": now_ns,
                    "elapsed_s": f"{elapsed:.9f}",
                    "bota_timestamp": getattr(frame, "timestamp", ""),
                    "fx": force[0], "fy": force[1], "fz": force[2],
                    "tx": torque[0], "ty": torque[1], "tz": torque[2],
                    "bota_temperature": getattr(frame, "temperature", ""),
                    "bota_status": getattr(status, "raw", ""),
                    "force_zero_fx": force_zero[0], "force_zero_fy": force_zero[1], "force_zero_fz": force_zero[2],
                    "torque_zero_tx": torque_zero[0], "torque_zero_ty": torque_zero[1], "torque_zero_tz": torque_zero[2],
                    "opti_local_ns": "" if sample is None else sample.local_ns,
                    "opti_dt_ms": "" if dt_ms is None else f"{dt_ms:.3f}",
                    "opti_frame": "" if sample is None else sample.frame,
                    "opti_timestamp": "" if sample is None else sample.motive_timestamp,
                    "opti_rigid_id": args.rigid_id,
                    "opti_seen": "" if sample is None else sample.seen,
                    "opti_px": "" if sample is None else sample.px,
                    "opti_py": "" if sample is None else sample.py,
                    "opti_pz": "" if sample is None else sample.pz,
                    "opti_qx": "" if sample is None else sample.qx,
                    "opti_qy": "" if sample is None else sample.qy,
                    "opti_qz": "" if sample is None else sample.qz,
                    "opti_qw": "" if sample is None else sample.qw,
                    "origin_frame": origin.frame,
                    "origin_px": origin_pos[0], "origin_py": origin_pos[1], "origin_pz": origin_pos[2],
                    "origin_qx": origin_quat[0], "origin_qy": origin_quat[1], "origin_qz": origin_quat[2], "origin_qw": origin_quat[3],
                    "deflection_x_m": rel_pos[0], "deflection_y_m": rel_pos[1], "deflection_z_m": rel_pos[2],
                    "deflection_norm_m": rel_norm,
                    "rel_qx": rel_quat[0], "rel_qy": rel_quat[1], "rel_qz": rel_quat[2], "rel_qw": rel_quat[3],
                    "reorientation_angle_deg": rel_angle_deg,
                }
                writer.writerow(row)

                if elapsed - last_print >= 1.0 / max(args.print_rate, 0.001):
                    last_print = elapsed
                    if sample is None:
                        opti_text = "OptiTrack: waiting"
                    elif sample.px is None:
                        opti_text = f"OptiTrack: ID {args.rigid_id} not in latest frame"
                    else:
                        stale_text = " STALE" if dt_ms is not None and dt_ms > args.max_opti_age_ms else ""
                        opti_text = (
                            f"frame={sample.frame} dt={dt_ms:.1f}ms{stale_text} "
                            f"def=({rel_pos[0]:.4f},{rel_pos[1]:.4f},{rel_pos[2]:.4f})m "
                            f"|def|={rel_norm:.4f}m angle={rel_angle_deg:.2f}deg"
                        )
                    print(
                        f"t={elapsed:7.3f}s F=({force[0]: .3f},{force[1]: .3f},{force[2]: .3f}) "
                        f"T=({torque[0]: .4f},{torque[1]: .4f},{torque[2]: .4f}) | {opti_text}",
                        flush=True,
                    )
                sleep_s = period - (time.perf_counter() - loop_start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        print(f"Wrote CSV: {args.csv}")
        return 0
    except Exception as exc:
        print(f"FATAL: {exc}", file=sys.stderr)
        return 1
    finally:
        if opti is not None:
            opti.stop()
        if bota is not None:
            try:
                bota.deactivate()
            except Exception:
                pass
            try:
                bota.shutdown()
            except Exception:
                pass
        try:
            os.unlink(config_path)
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
