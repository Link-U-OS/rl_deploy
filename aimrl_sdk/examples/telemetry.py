from __future__ import annotations

import asyncio
import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from loguru import logger

import aimrl_sdk


def _now_ns_fallback(stamp_ns: int) -> int:
    s = int(stamp_ns)
    return s if s > 0 else time.time_ns()


def _json_dumps_bytes(obj: Any) -> bytes:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _ensure_mcap_path(path: Path) -> Path:
    p = Path(path)
    if p.suffix.lower() == ".mcap":
        p.parent.mkdir(parents=True, exist_ok=True)
        return p
    p.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return p / f"aimrl_{ts}.mcap"


def _schema_arm_state() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "position": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
            "velocity": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
            "effort": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
        },
        "required": ["stamp_ns", "position", "velocity", "effort"],
        "additionalProperties": False,
    }


def _schema_leg_state() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "position": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
            "velocity": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
            "effort": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
        },
        "required": ["stamp_ns", "position", "velocity", "effort"],
        "additionalProperties": False,
    }


def _schema_imu() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "quat_xyzw": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4},
            "gyro_xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
            "acc_xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
            "euler_xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        },
        "required": ["stamp_ns", "quat_xyzw", "gyro_xyz", "acc_xyz", "euler_xyz"],
        "additionalProperties": False,
    }


def _schema_frame_status() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "aligned": {"type": "boolean"},
            "complete": {"type": "boolean"},
        },
        "required": ["stamp_ns", "aligned", "complete"],
        "additionalProperties": False,
    }


def _quat_xyzw_to_euler_xyz(q_xyzw: list[float]) -> list[float]:
    x, y, z, w = [float(v) for v in q_xyzw]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [float(roll), float(pitch), float(yaw)]


def _schema_cmd_arm() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "position": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
            "stiffness": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
            "damping": {"type": "array", "items": {"type": "number"}, "minItems": 14, "maxItems": 14},
        },
        "required": ["stamp_ns", "position", "stiffness", "damping"],
        "additionalProperties": False,
    }


def _schema_cmd_leg() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "stamp_ns": {"type": "integer"},
            "position": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
            "stiffness": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
            "damping": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
        },
        "required": ["stamp_ns", "position", "stiffness", "damping"],
        "additionalProperties": False,
    }


class _McapRecorder:
    def __init__(self, path: Path, *, metadata: dict[str, str], record_cmd: bool):
        try:
            from mcap.writer import Writer
        except Exception as e:  # pragma: no cover
            raise RuntimeError("recording requires `mcap` (pip install mcap)") from e

        self._path = _ensure_mcap_path(path)
        self._fp = self._path.open("wb")
        self._w = Writer(self._fp)
        self._w.add_metadata("aimrl_sdk", dict(metadata))

        self._schema_id_arm = self._w.register_schema(
            "aimrl/state/arm", "jsonschema", _json_dumps_bytes(_schema_arm_state())
        )
        self._schema_id_leg = self._w.register_schema(
            "aimrl/state/leg", "jsonschema", _json_dumps_bytes(_schema_leg_state())
        )
        self._schema_id_imu = self._w.register_schema(
            "aimrl/state/imu", "jsonschema", _json_dumps_bytes(_schema_imu())
        )
        self._schema_id_status = self._w.register_schema(
            "aimrl/frame_status", "jsonschema", _json_dumps_bytes(_schema_frame_status())
        )
        self._schema_id_cmd_arm = (
            self._w.register_schema("aimrl/cmd_arm", "jsonschema", _json_dumps_bytes(_schema_cmd_arm()))
            if record_cmd
            else 0
        )
        self._schema_id_cmd_leg = (
            self._w.register_schema("aimrl/cmd_leg", "jsonschema", _json_dumps_bytes(_schema_cmd_leg()))
            if record_cmd
            else 0
        )

        self._chan_arm = self._w.register_channel("/aimrl/state/arm", "json", self._schema_id_arm)
        self._chan_leg = self._w.register_channel("/aimrl/state/leg", "json", self._schema_id_leg)
        self._chan_imu = self._w.register_channel("/aimrl/state/imu", "json", self._schema_id_imu)
        self._chan_status = self._w.register_channel("/aimrl/frame_status", "json", self._schema_id_status)
        self._chan_cmd_arm = (
            self._w.register_channel("/aimrl/cmd/arm", "json", self._schema_id_cmd_arm) if record_cmd else None
        )
        self._chan_cmd_leg = (
            self._w.register_channel("/aimrl/cmd/leg", "json", self._schema_id_cmd_leg) if record_cmd else None
        )

    @property
    def path(self) -> Path:
        return self._path

    def write_json(self, channel_id: int, stamp_ns: int, msg: Any) -> None:
        t = _now_ns_fallback(stamp_ns)
        self._w.add_message(channel_id, log_time=t, publish_time=t, data=_json_dumps_bytes(msg))

    def write_arm(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        self.write_json(self._chan_arm, stamp_ns, msg)

    def write_leg(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        self.write_json(self._chan_leg, stamp_ns, msg)

    def write_imu(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        self.write_json(self._chan_imu, stamp_ns, msg)

    def write_status(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        self.write_json(self._chan_status, stamp_ns, msg)

    def write_cmd_arm(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        if self._chan_cmd_arm is None:
            return
        self.write_json(self._chan_cmd_arm, stamp_ns, msg)

    def write_cmd_leg(self, stamp_ns: int, msg: dict[str, Any]) -> None:
        if self._chan_cmd_leg is None:
            return
        self.write_json(self._chan_cmd_leg, stamp_ns, msg)

    def close(self) -> None:
        try:
            self._w.finish()
        finally:
            self._fp.close()


class _FoxgloveLive:
    def __init__(self, *, host: str, port: int, name: str, metadata: dict[str, str], record_cmd: bool):
        try:
            from foxglove_websocket.server import FoxgloveServer
            from foxglove_websocket.types import ChannelWithoutId
        except Exception as e:  # pragma: no cover
            raise RuntimeError("live view requires `foxglove-websocket` (pip install foxglove-websocket)") from e

        self._FoxgloveServer = FoxgloveServer
        self._ChannelWithoutId = ChannelWithoutId

        self._host = str(host)
        self._port = int(port)
        self._name = str(name)
        self._metadata = dict(metadata)
        self._record_cmd = bool(record_cmd)

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="foxglove-live", daemon=True)
        self._ready = threading.Event()
        self._closing = threading.Event()

        self._server = None
        self._chan_by_topic: dict[str, int] = {}

    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(timeout=2.0):
            logger.warning("Foxglove live server not ready yet; messages may be dropped briefly")

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        server = self._FoxgloveServer(
            self._host,
            self._port,
            self._name,
            capabilities=["time"],
            supported_encodings=["json"],
            metadata=self._metadata,
        )
        self._server = server

        def _add_channel(topic: str, schema_name: str, schema_obj: dict[str, Any]) -> None:
            ch = self._ChannelWithoutId(
                topic=topic,
                encoding="json",
                schemaName=schema_name,
                schema=json.dumps(schema_obj, ensure_ascii=False),
                schemaEncoding="jsonschema",
            )
            self._pending_channels.append((topic, ch))

        self._pending_channels: list[tuple[str, Any]] = []
        _add_channel("/aimrl/state/arm", "aimrl/state/arm", _schema_arm_state())
        _add_channel("/aimrl/state/leg", "aimrl/state/leg", _schema_leg_state())
        _add_channel("/aimrl/state/imu", "aimrl/state/imu", _schema_imu())
        _add_channel("/aimrl/frame_status", "aimrl/frame_status", _schema_frame_status())
        if self._record_cmd:
            _add_channel("/aimrl/cmd/arm", "aimrl/cmd_arm", _schema_cmd_arm())
            _add_channel("/aimrl/cmd/leg", "aimrl/cmd_leg", _schema_cmd_leg())

        for topic, ch in self._pending_channels:
            chan_id = await server.add_channel(ch)
            self._chan_by_topic[str(topic)] = int(chan_id)

        server.start()
        self._ready.set()

        try:
            await server.wait_closed()
        finally:
            self._closing.set()

    def send(self, topic: str, stamp_ns: int, msg: Any) -> None:
        if self._closing.is_set():
            return
        chan = self._chan_by_topic.get(topic)
        if chan is None:
            return
        t = _now_ns_fallback(stamp_ns)
        payload = _json_dumps_bytes(msg)

        async def _send() -> None:
            if self._server is None:
                return
            await self._server.broadcast_time(t)
            await self._server.send_message(chan, t, payload)

        try:
            asyncio.run_coroutine_threadsafe(_send(), self._loop)
        except RuntimeError:
            return

    def close(self) -> None:
        if self._server is None:
            return

        def _close() -> None:
            try:
                self._server.close()
            except Exception:
                pass

        self._loop.call_soon_threadsafe(_close)
        self._thread.join(timeout=2.0)
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass


@dataclass(frozen=True)
class TelemetryConfig:
    mode: str = "off"  # off|live|record
    record_mcap: Optional[Path] = None
    foxglove_host: str = "0.0.0.0"
    foxglove_port: Optional[int] = None
    foxglove_live_hz: float = 20.0
    queue_size: int = 2000
    # Commands are always recorded/streamed when telemetry is enabled.


class Telemetry:
    def __init__(self, cfg: TelemetryConfig, *, metadata: dict[str, str]):
        mode = str(cfg.mode).strip().lower()
        if mode not in {"off", "live", "record"}:
            raise ValueError(f"unsupported telemetry mode: {cfg.mode!r} (expected: off|live|record)")

        self._cfg = cfg
        self._metadata = dict(metadata)

        self._q: queue.Queue[tuple[str, tuple[Any, ...]]] = queue.Queue(maxsize=max(1, int(cfg.queue_size)))
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="telemetry-io", daemon=True)

        self._rec: Optional[_McapRecorder] = None
        self._live: Optional[_FoxgloveLive] = None

        self._last_live_ns = 0
        self._min_live_dt_ns = 0
        if cfg.foxglove_live_hz and cfg.foxglove_live_hz > 0:
            self._min_live_dt_ns = int(1e9 / float(cfg.foxglove_live_hz))

        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    @property
    def recording_path(self) -> Optional[Path]:
        return self._rec.path if self._rec is not None else None

    def push_frame(self, stamp_ns: int, aligned: bool, complete: bool, obs: Any) -> None:
        self._put(("frame", (int(stamp_ns), bool(aligned), bool(complete), obs)))

    def push_cmd(
        self,
        *,
        stamp_ns: int,
        arm_position: Any,
        arm_stiffness: Any,
        arm_damping: Any,
        leg_position: Any,
        leg_stiffness: Any,
        leg_damping: Any,
    ) -> None:
        self._put(("cmd_arm", (int(stamp_ns), arm_position, arm_stiffness, arm_damping)))
        self._put(("cmd_leg", (int(stamp_ns), leg_position, leg_stiffness, leg_damping)))

    def _put(self, item: tuple[str, tuple[Any, ...]]) -> None:
        if self._stop.is_set():
            return
        try:
            self._q.put_nowait(item)
        except queue.Full:
            try:
                _ = self._q.get_nowait()
            except queue.Empty:
                return
            try:
                self._q.put_nowait(item)
            except queue.Full:
                return

    def close(self) -> None:
        self._stop.set()
        while True:
            try:
                self._q.put_nowait(("__stop__", tuple()))
                break
            except queue.Full:
                try:
                    _ = self._q.get_nowait()
                except queue.Empty:
                    break
        self._thread.join(timeout=5.0)

        if self._rec is not None:
            try:
                self._rec.close()
            finally:
                self._rec = None

        if self._live is not None:
            try:
                self._live.close()
            finally:
                self._live = None

    def _run(self) -> None:
        mode = str(self._cfg.mode).strip().lower()
        if mode == "record" and self._cfg.record_mcap is not None:
            try:
                self._rec = _McapRecorder(self._cfg.record_mcap, metadata=self._metadata, record_cmd=True)
                logger.info(f"Recording MCAP: {self._rec.path}")
            except Exception as e:
                self._rec = None
                logger.error(f"Failed to start MCAP recorder: {e}")

        if mode == "live" and self._cfg.foxglove_port is not None:
            try:
                self._live = _FoxgloveLive(
                    host=self._cfg.foxglove_host,
                    port=int(self._cfg.foxglove_port),
                    name="aimrl_sdk",
                    metadata=self._metadata,
                    record_cmd=True,
                )
                self._live.start()
                logger.info(f"Foxglove live: ws://{self._cfg.foxglove_host}:{int(self._cfg.foxglove_port)}")
            except Exception as e:
                self._live = None
                logger.error(f"Failed to start Foxglove live server: {e}")

        while True:
            try:
                typ, payload = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if typ == "__stop__":
                break

            if typ == "frame":
                stamp_ns, aligned, complete, obs = payload
                self._handle_frame(int(stamp_ns), bool(aligned), bool(complete), obs)
                continue

            if typ == "cmd_arm":
                self._handle_cmd_arm(*payload)
                continue

            if typ == "cmd_leg":
                self._handle_cmd_leg(*payload)
                continue

    def _handle_frame(self, stamp_ns: int, aligned: bool, complete: bool, obs: Any) -> None:
        t = _now_ns_fallback(stamp_ns)
        status = {"stamp_ns": t, "aligned": bool(aligned), "complete": bool(complete)}

        need_record = self._rec is not None
        need_live = False
        if self._live is not None and self._min_live_dt_ns > 0:
            if (t - self._last_live_ns) >= self._min_live_dt_ns:
                need_live = True
                self._last_live_ns = t

        if not need_record and not need_live:
            return

        arm_pos = [float(x) for x in obs[aimrl_sdk.OBS.arm_pos]]
        arm_vel = [float(x) for x in obs[aimrl_sdk.OBS.arm_vel]]
        arm_eff = [float(x) for x in obs[aimrl_sdk.OBS.arm_eff]]

        leg_pos = [float(x) for x in obs[aimrl_sdk.OBS.leg_pos]]
        leg_vel = [float(x) for x in obs[aimrl_sdk.OBS.leg_vel]]
        leg_eff = [float(x) for x in obs[aimrl_sdk.OBS.leg_eff]]

        imu_quat = [float(x) for x in obs[aimrl_sdk.OBS.imu_quat_xyzw]]
        imu_gyro = [float(x) for x in obs[aimrl_sdk.OBS.imu_gyro_xyz]]
        imu_acc = [float(x) for x in obs[aimrl_sdk.OBS.imu_acc_xyz]]
        imu_euler = _quat_xyzw_to_euler_xyz(imu_quat)

        msg_arm = {"stamp_ns": t, "position": arm_pos, "velocity": arm_vel, "effort": arm_eff}
        msg_leg = {"stamp_ns": t, "position": leg_pos, "velocity": leg_vel, "effort": leg_eff}
        msg_imu = {"stamp_ns": t, "quat_xyzw": imu_quat, "gyro_xyz": imu_gyro, "acc_xyz": imu_acc, "euler_xyz": imu_euler}

        if need_record and self._rec is not None:
            self._rec.write_status(t, status)
            self._rec.write_arm(t, msg_arm)
            self._rec.write_leg(t, msg_leg)
            self._rec.write_imu(t, msg_imu)

        if need_live and self._live is not None:
            self._live.send("/aimrl/frame_status", t, status)
            self._live.send("/aimrl/state/arm", t, msg_arm)
            self._live.send("/aimrl/state/leg", t, msg_leg)
            self._live.send("/aimrl/state/imu", t, msg_imu)

    def _handle_cmd_arm(self, stamp_ns: int, position: Any, stiffness: Any, damping: Any) -> None:
        t = _now_ns_fallback(stamp_ns)
        msg = {
            "stamp_ns": t,
            "position": [float(x) for x in position],
            "stiffness": [float(x) for x in stiffness],
            "damping": [float(x) for x in damping],
        }
        if self._rec is not None:
            self._rec.write_cmd_arm(t, msg)
        if self._live is not None:
            self._live.send("/aimrl/cmd/arm", t, msg)

    def _handle_cmd_leg(self, stamp_ns: int, position: Any, stiffness: Any, damping: Any) -> None:
        t = _now_ns_fallback(stamp_ns)
        msg = {
            "stamp_ns": t,
            "position": [float(x) for x in position],
            "stiffness": [float(x) for x in stiffness],
            "damping": [float(x) for x in damping],
        }
        if self._rec is not None:
            self._rec.write_cmd_leg(t, msg)
        if self._live is not None:
            self._live.send("/aimrl/cmd/leg", t, msg)


class FrameCaptureThread:
    def __init__(self, state: Any, telemetry: Telemetry, *, timeout_s: float = 1.0):
        self._state = state
        self._telemetry = telemetry
        self._timeout_s = float(timeout_s)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="frame-capture", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                stamp_ns, aligned, complete, obs = self._state.wait_frame(timeout_s=self._timeout_s)
            except Exception:
                continue
            self._telemetry.push_frame(int(stamp_ns), bool(aligned), bool(complete), obs)
