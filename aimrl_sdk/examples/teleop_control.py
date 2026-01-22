from __future__ import annotations

import enum
import errno
import os
import select
import sys
import termios
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from loguru import logger

import aimrl_sdk
from rl_deploy_config import AppCfg


class Mode(enum.Enum):
    DEFAULT = "default"
    LIE = "lie"
    STAND = "stand"
    WALK = "walk"


@dataclass
class InputSnapshot:
    cmd_x: float
    cmd_y: float
    cmd_yaw: float
    deadman: bool
    start_control_edge: bool
    switch_mode_edge: bool
    walk_mode_edge: bool
    position_control_edge: bool
    emergency_stop_edge: bool
    quit_edge: bool


class LinuxJoystick:
    # Linux joystick event format: u32 time, s16 value, u8 type, u8 number
    _EVENT_SIZE = 8
    _FMT = "IhBB"
    _TYPE_BUTTON = 0x01
    _TYPE_AXIS = 0x02
    _TYPE_INIT = 0x80

    def __init__(self, path: Path):
        self.path = Path(path)
        self.fd: int | None = None
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self._last_buttons = self.buttons.copy()

    def open(self) -> bool:
        try:
            self.fd = os.open(str(self.path), os.O_RDONLY | os.O_NONBLOCK)
            return True
        except OSError:
            self.fd = None
            return False

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            finally:
                self.fd = None
        self.axes = [0.0] * len(self.axes)
        self.buttons = [0] * len(self.buttons)
        self._last_buttons = self.buttons.copy()

    def _mark_disconnected(self) -> None:
        self.close()

    def poll(self) -> bool:
        if self.fd is None:
            return False

        import struct

        while True:
            r, _, _ = select.select([self.fd], [], [], 0.0)
            if not r:
                return True

            try:
                data = os.read(self.fd, self._EVENT_SIZE)
            except OSError as e:
                # Common when the receiver is unplugged while the program is running.
                if e.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    self._mark_disconnected()
                    return False
                raise
            except BlockingIOError:
                return True
            if len(data) != self._EVENT_SIZE:
                # Unexpected short read; treat as "no more data" but keep the device.
                return True

            _, value, typ, number = struct.unpack(self._FMT, data)
            typ_no_init = typ & ~self._TYPE_INIT
            if typ_no_init == self._TYPE_AXIS:
                if 0 <= number < len(self.axes):
                    v = float(value) / 32767.0
                    self.axes[number] = float(np.clip(v, -1.0, 1.0))
            elif typ_no_init == self._TYPE_BUTTON:
                if 0 <= number < len(self.buttons):
                    self.buttons[number] = 1 if value else 0

    def consume_rising_edges(self) -> list[int]:
        edges: list[int] = []
        for i, (prev, cur) in enumerate(zip(self._last_buttons, self.buttons, strict=True)):
            if prev == 0 and cur == 1:
                edges.append(i)
        self._last_buttons = self.buttons.copy()
        return edges


class RawKeyboard:
    def __init__(self):
        self._enabled = False
        self._stdin_fd: int | None = None
        self._old_termios: list[int] | None = None
        self._pending = bytearray()

    def open(self) -> bool:
        if not sys.stdin.isatty():
            return False
        self._stdin_fd = sys.stdin.fileno()
        try:
            self._old_termios = termios.tcgetattr(self._stdin_fd)
            # Avoid `tty.setraw()`: it disables output post-processing (OPOST/ONLCR) for the whole TTY,
            # which makes '\n' not return the cursor to column 0 and causes "diagonal"/indented logs.
            #
            # Instead, use a cbreak-like mode: non-canonical input, no echo, but keep normal output.
            new_attr = termios.tcgetattr(self._stdin_fd)
            new_attr[3] &= ~(termios.ECHO | termios.ICANON)
            new_attr[1] |= termios.OPOST
            if hasattr(termios, "ONLCR"):
                new_attr[1] |= termios.ONLCR
            new_attr[6][termios.VMIN] = 0
            new_attr[6][termios.VTIME] = 0
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, new_attr)
            self._enabled = True
            return True
        except Exception:
            self.close()
            return False

    def close(self) -> None:
        if self._stdin_fd is not None and self._old_termios is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass
        self._enabled = False
        self._stdin_fd = None
        self._old_termios = None
        self._pending.clear()

    def _read_available(self) -> None:
        if not self._enabled or self._stdin_fd is None:
            return
        while True:
            r, _, _ = select.select([self._stdin_fd], [], [], 0.0)
            if not r:
                return
            try:
                chunk = os.read(self._stdin_fd, 64)
            except BlockingIOError:
                return
            if not chunk:
                return
            self._pending.extend(chunk)

    def poll_keys(self) -> list[str]:
        self._read_available()
        out: list[str] = []

        def take(n: int) -> bytes | None:
            if len(self._pending) < n:
                return None
            b = bytes(self._pending[:n])
            del self._pending[:n]
            return b

        while self._pending:
            # Arrow keys can arrive as:
            # - CSI: ESC [ A/B/C/D
            # - SS3: ESC O A/B/C/D (some terminals / application cursor mode)
            # - CSI w/ modifiers: ESC [ 1 ; 2 A  (etc.)
            if self._pending[:1] == b"\x1b":
                if len(self._pending) < 2:
                    break

                second = self._pending[1]
                if second == ord("O"):
                    seq = take(3)
                    if seq is None:
                        break
                    last = seq[-1:]
                    if last == b"A":
                        out.append("UP")
                    elif last == b"B":
                        out.append("DOWN")
                    elif last == b"C":
                        out.append("RIGHT")
                    elif last == b"D":
                        out.append("LEFT")
                    else:
                        out.append("ESC")
                    continue

                if second == ord("["):
                    # CSI: consume until final byte (0x40..0x7E). Cap length to avoid getting stuck.
                    max_seq_len = 16
                    end_idx: int | None = None
                    scan_limit = min(len(self._pending), max_seq_len)
                    for i in range(2, scan_limit):
                        if 0x40 <= self._pending[i] <= 0x7E:
                            end_idx = i
                            break
                    if end_idx is None:
                        if len(self._pending) < max_seq_len:
                            break
                        # Malformed/too-long CSI: treat as ESC and advance 1 byte.
                        take(1)
                        out.append("ESC")
                        continue

                    seq = take(end_idx + 1)
                    if seq is None:
                        break
                    last = seq[-1:]
                    if last == b"A":
                        out.append("UP")
                    elif last == b"B":
                        out.append("DOWN")
                    elif last == b"C":
                        out.append("RIGHT")
                    elif last == b"D":
                        out.append("LEFT")
                    else:
                        out.append("ESC")
                    continue

            b1 = take(1)
            if b1 is None:
                break
            if b1 == b"\x03":
                out.append("CTRL_C")
            elif b1 == b"\x04":
                out.append("CTRL_D")
            elif b1 == b"\x1b":
                out.append("ESC")
            else:
                try:
                    out.append(b1.decode("utf-8"))
                except UnicodeDecodeError:
                    continue
        return out


class TeleopInput:
    # Matches deploy/rl_controllers/config/joy.yaml:
    # - cmd_vel deadman: button 4
    # - start_control: button 7
    # - switch_mode: deadman(button 4) + button 0
    # - walk_mode: button 2
    # - position_control: button 3
    # - emergency_stop: requires buttons 5 and 1, then press button 6
    def __init__(self, joystick_path: Path, init_cmd: tuple[float, float, float]):
        self._js = LinuxJoystick(joystick_path)
        self._kb = RawKeyboard()

        self._kb_deadman = False
        self._cmd_x, self._cmd_y, self._cmd_yaw = init_cmd
        self._last_js_reconnect_t = 0.0
        self._js_reconnect_period_s = 1.0

        # These maxima match joy.yaml scales.
        self._max_x = 2.4
        self._max_y = 1.5
        self._max_yaw = 1.0
        self._step_x = 0.12
        self._step_y = 0.075
        self._step_yaw = 0.05

        self.joystick_ok = self._js.open()
        self.keyboard_ok = self._kb.open()

        logger.info(
            "Keyboard: w/s or ↑/↓ fwd, a/d or ←/→ yaw, q/e lateral, space=toggle deadman, "
            "p=start, m=switch, k=walk, l=pos, x=E-stop"
        )
        if self.joystick_ok:
            logger.info(f"Joystick enabled: {joystick_path}")
        else:
            logger.warning(f"Joystick not available: {joystick_path} (keyboard still works)")
        if self.keyboard_ok:
            logger.info("Keyboard enabled (raw mode)")
        else:
            logger.warning("Keyboard not available (stdin is not a TTY)")

    def _maybe_reconnect_joystick(self) -> None:
        if self.joystick_ok:
            return
        now = time.monotonic()
        if now - self._last_js_reconnect_t < self._js_reconnect_period_s:
            return
        self._last_js_reconnect_t = now
        if self._js.open():
            self.joystick_ok = True
            logger.warning(f"Joystick reconnected: {self._js.path}")

    def close(self) -> None:
        self._js.close()
        self._kb.close()

    def _apply_cmd_limits(self) -> None:
        self._cmd_x = float(np.clip(self._cmd_x, -self._max_x, self._max_x))
        self._cmd_y = float(np.clip(self._cmd_y, -self._max_y, self._max_y))
        self._cmd_yaw = float(np.clip(self._cmd_yaw, -self._max_yaw, self._max_yaw))

    def poll(self) -> InputSnapshot:
        if self.joystick_ok:
            ok = self._js.poll()
            if not ok:
                self.joystick_ok = False
                # Safety defaults: stop walking command and release deadman.
                self._kb_deadman = False
                self._cmd_x = 0.0
                self._cmd_y = 0.0
                self._cmd_yaw = 0.0
                logger.warning("Joystick disconnected; holding mode and zeroing cmd (deadman released)")
        else:
            self._maybe_reconnect_joystick()

        edges = set(self._js.consume_rising_edges()) if self.joystick_ok else set()

        start_control_edge = 7 in edges
        switch_mode_edge = (0 in edges) and (self._js.buttons[4] == 1)
        walk_mode_edge = 2 in edges
        position_control_edge = 3 in edges
        emergency_stop_edge = (6 in edges) and (self._js.buttons[1] == 1) and (self._js.buttons[5] == 1)
        quit_edge = False

        keys = self._kb.poll_keys()
        for k in keys:
            if k in ("CTRL_C", "CTRL_D"):
                quit_edge = True
            elif k in (" ",):
                self._kb_deadman = not self._kb_deadman
                self._cmd_x = 0.0
                self._cmd_y = 0.0
                self._cmd_yaw = 0.0
            elif k in ("p", "P", "\r"):
                start_control_edge = True
            elif k in ("m", "M"):
                switch_mode_edge = True
            elif k in ("k", "K"):
                walk_mode_edge = True
            elif k in ("l", "L"):
                position_control_edge = True
            elif k in ("x", "X", "ESC"):
                emergency_stop_edge = True
            elif k in ("0",):
                self._cmd_x = 0.0
                self._cmd_y = 0.0
                self._cmd_yaw = 0.0
            elif k in ("UP", "w", "W"):
                self._cmd_x += self._step_x
            elif k in ("DOWN", "s", "S"):
                self._cmd_x -= self._step_x
            elif k in ("LEFT", "a", "A"):
                self._cmd_yaw += self._step_yaw
            elif k in ("RIGHT", "d", "D"):
                self._cmd_yaw -= self._step_yaw
            elif k in ("q", "Q"):
                self._cmd_y -= self._step_y
            elif k in ("e", "E"):
                self._cmd_y += self._step_y

        # Joystick takes precedence for continuous commands if present.
        if self.joystick_ok:
            # Many gamepads report "push stick forward" as negative; map to +cmd_x (forward).
            self._cmd_x = -self._js.axes[1] * self._max_x
            self._cmd_y = self._js.axes[0] * self._max_y
            # Match intuitive turning direction (left stick -> +yaw or vice versa depending on hardware).
            self._cmd_yaw = -self._js.axes[3] * self._max_yaw

        self._apply_cmd_limits()

        deadman = (self._js.buttons[4] == 1) if self.joystick_ok else self._kb_deadman
        if not deadman:
            cmd_x, cmd_y, cmd_yaw = 0.0, 0.0, 0.0
        else:
            cmd_x, cmd_y, cmd_yaw = self._cmd_x, self._cmd_y, self._cmd_yaw

        return InputSnapshot(
            cmd_x=cmd_x,
            cmd_y=cmd_y,
            cmd_yaw=cmd_yaw,
            deadman=deadman,
            start_control_edge=start_control_edge,
            switch_mode_edge=switch_mode_edge,
            walk_mode_edge=walk_mode_edge,
            position_control_edge=position_control_edge,
            emergency_stop_edge=emergency_stop_edge,
            quit_edge=quit_edge,
        )


class PolicyRunner(Protocol):
    def step(self, obs: np.ndarray, cmd_x: float, cmd_y: float, cmd_yaw: float) -> np.ndarray: ...


class MotionFSM:
    def __init__(self, app_cfg: AppCfg):
        self.app_cfg = app_cfg

        self.mode = Mode.DEFAULT
        self.start_control = False
        self.emergency_stop = False

        self._switch_time = time.monotonic()
        self._debounce_s = 0.5

        self._transition_t = 0.0
        self._transition_duration_s = 2.0
        self._from_leg = np.zeros(12, dtype=np.float32)

        # Keep pose simple/neutral (repo's deploy defaults are ~0), but keep the same mode semantics.
        self._lie_pose = np.array(app_cfg.default_joint_angles, dtype=np.float32).reshape(12)
        self._stand_pose = np.array(app_cfg.default_joint_angles, dtype=np.float32).reshape(12)

        self._lie_stiffness = np.array(app_cfg.leg_stiffness, dtype=np.float32) * 0.5
        self._lie_damping = np.array(app_cfg.leg_damping, dtype=np.float32) * 0.5
        self._stand_stiffness = np.array(app_cfg.leg_stiffness, dtype=np.float32)
        self._stand_damping = np.array(app_cfg.leg_damping, dtype=np.float32)
        self._emg_damping = np.array(app_cfg.leg_damping, dtype=np.float32)

    def _debounced(self) -> bool:
        now = time.monotonic()
        if now - self._switch_time < self._debounce_s:
            return False
        self._switch_time = now
        return True

    def _start_transition(self, from_leg: np.ndarray) -> None:
        self._transition_t = 0.0
        self._from_leg = from_leg.astype(np.float32, copy=True).reshape(12)

    def on_input(self, snap: InputSnapshot, obs: np.ndarray) -> None:
        leg_pos_cur = obs[aimrl_sdk.OBS.leg_pos].astype(np.float32, copy=False)

        if snap.emergency_stop_edge and self._debounced():
            self.emergency_stop = True
            logger.warning("Emergency stop set (press start_control to clear)")

        if snap.start_control_edge and self._debounced():
            if not self.start_control:
                self.start_control = True
                self._start_transition(leg_pos_cur)
                self.mode = Mode.LIE
                logger.info("Start control -> LIE")
            else:
                self.start_control = False
                self.mode = Mode.DEFAULT
                logger.info("Shutdown control -> DEFAULT")

        if self.emergency_stop and self.start_control:
            self.emergency_stop = False
            self.start_control = False
            self.mode = Mode.DEFAULT
            logger.warning("Emergency stop cleared -> DEFAULT")

        if snap.switch_mode_edge and self._debounced():
            if self.start_control:
                if self.mode == Mode.STAND:
                    self._start_transition(leg_pos_cur)
                    self.mode = Mode.LIE
                    logger.info("STAND -> LIE")
                elif self.mode == Mode.LIE:
                    self._start_transition(leg_pos_cur)
                    self.mode = Mode.STAND
                    logger.info("LIE -> STAND")

        if snap.walk_mode_edge and self._debounced():
            if self.mode == Mode.STAND:
                self.mode = Mode.WALK
                logger.info("STAND -> WALK")

        if snap.position_control_edge and self._debounced():
            if self.mode == Mode.WALK:
                self._start_transition(leg_pos_cur)
                self.mode = Mode.STAND
                logger.info("WALK -> STAND")
            elif self.mode == Mode.DEFAULT:
                self._start_transition(leg_pos_cur)
                self.mode = Mode.LIE
                logger.info("DEFAULT -> LIE")

    def step(
        self, obs: np.ndarray, policy: PolicyRunner, snap: InputSnapshot, dt: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        leg_pos_cur = obs[aimrl_sdk.OBS.leg_pos].astype(np.float32, copy=False)

        if self.emergency_stop:
            return (
                leg_pos_cur.astype(np.float64),
                np.zeros(12, dtype=np.float64),
                self._emg_damping.astype(np.float64),
            )

        if self.mode == Mode.DEFAULT:
            return leg_pos_cur.astype(np.float64), np.zeros(12, dtype=np.float64), np.full(12, 0.1, dtype=np.float64)

        if self.mode == Mode.WALK:
            leg_pos_des = policy.step(obs, snap.cmd_x, snap.cmd_y, snap.cmd_yaw).astype(np.float64, copy=False)
            return leg_pos_des, self._stand_stiffness.astype(np.float64), self._stand_damping.astype(np.float64)

        self._transition_t = min(self._transition_duration_s, self._transition_t + dt)
        alpha = self._transition_t / self._transition_duration_s if self._transition_duration_s > 0 else 1.0

        if self.mode == Mode.LIE:
            pose = (1.0 - alpha) * self._from_leg + alpha * self._lie_pose
            return pose.astype(np.float64), self._lie_stiffness.astype(np.float64), self._lie_damping.astype(np.float64)

        if self.mode == Mode.STAND:
            pose = (1.0 - alpha) * self._from_leg + alpha * self._stand_pose
            return pose.astype(np.float64), self._stand_stiffness.astype(np.float64), self._stand_damping.astype(np.float64)

        raise RuntimeError(f"unknown mode: {self.mode}")
