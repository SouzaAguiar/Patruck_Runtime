"""Browser-based command source for the Open Duck Mini runtime.

The public API intentionally mirrors XBoxController.get_last_command() so the
walking loop does not need to know where commands came from.
"""

from __future__ import annotations

import asyncio
import json
import secrets
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from aiohttp import WSMsgType, web

from mini_bdx_runtime.buttons import Buttons
from mini_bdx_runtime.xbox_controller import (
    HEAD_PITCH_RANGE,
    HEAD_ROLL_RANGE,
    HEAD_YAW_RANGE,
    X_RANGE,
    Y_RANGE,
    YAW_RANGE,
)


def _clamp(value: Any, low: float = -1.0, high: float = 1.0) -> float:
    try:
        return max(low, min(high, float(value)))
    except (TypeError, ValueError):
        return 0.0


def _scale_axis(value: float, limits: list[float]) -> float:
    return value * abs(limits[1] if value >= 0 else limits[0])


@dataclass
class WebCommandState:
    """Thread-safe state shared by aiohttp and the 50 Hz control loop."""

    timeout: float = 0.6
    allow_head_control: bool = False
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _commands: list[float] = field(default_factory=lambda: [0.0] * 7, init=False)
    _buttons: dict[str, bool] = field(
        default_factory=lambda: {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "LB": False,
            "RB": False,
            "dpad_up": False,
            "dpad_down": False,
        },
        init=False,
    )
    _left_trigger: float = field(default=0.0, init=False)
    _right_trigger: float = field(default=0.0, init=False)
    _last_update: float = field(default=0.0, init=False)
    _connected: bool = field(default=False, init=False)
    _desired_paused: bool | None = field(default=None, init=False)

    def update(self, payload: dict[str, Any]) -> None:
        mode = payload.get("mode", "walk")
        left_x = _clamp(payload.get("left_x", 0))
        left_y = _clamp(payload.get("left_y", 0))
        right_x = _clamp(payload.get("right_x", 0))
        commands = [0.0] * 7

        if mode == "head" and self.allow_head_control:
            commands[4] = _scale_axis(left_y, HEAD_PITCH_RANGE)
            # Preserve the sign mapping used by XBoxController.
            commands[5] = left_x * abs(
                HEAD_YAW_RANGE[0] if left_x >= 0 else HEAD_YAW_RANGE[1]
            )
            commands[6] = right_x * abs(
                HEAD_ROLL_RANGE[0] if right_x >= 0 else HEAD_ROLL_RANGE[1]
            )
        else:
            commands[0] = _scale_axis(left_y, X_RANGE)
            commands[1] = _scale_axis(left_x, Y_RANGE)
            commands[2] = _scale_axis(right_x, YAW_RANGE)

        raw_buttons = payload.get("buttons", {})
        buttons = {
            name: bool(raw_buttons.get(name, False)) for name in self._buttons
        }

        with self._lock:
            self._commands = [round(value, 3) for value in commands]
            self._buttons = buttons
            self._left_trigger = _clamp(payload.get("left_trigger", 0), 0, 1)
            self._right_trigger = _clamp(payload.get("right_trigger", 0), 0, 1)
            self._last_update = time.monotonic()
            self._connected = True
            if "paused" in payload:
                self._desired_paused = bool(payload["paused"])

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False
            self._commands = [0.0] * 7
            self._buttons = {name: False for name in self._buttons}
            self._left_trigger = 0.0
            self._right_trigger = 0.0

    def snapshot(self) -> tuple[list[float], dict[str, bool], float, float, bool]:
        with self._lock:
            fresh = self._connected and time.monotonic() - self._last_update <= self.timeout
            if not fresh:
                return [0.0] * 7, {name: False for name in self._buttons}, 0.0, 0.0, False
            return (
                self._commands.copy(),
                self._buttons.copy(),
                self._left_trigger,
                self._right_trigger,
                True,
            )

    def consume_desired_paused(self) -> bool | None:
        with self._lock:
            desired = self._desired_paused
            self._desired_paused = None
            return desired


class WebController:
    def __init__(
        self,
        command_freq: float,
        host: str = "0.0.0.0",
        port: int = 8080,
        token: str | None = None,
        command_timeout: float = 0.6,
        allow_head_control: bool = False,
        camera: bool = False,
        camera_size: tuple[int, int] = (640, 480),
        camera_fps: int = 10,
    ):
        self.command_freq = command_freq
        self.host = host
        self.port = port
        self.token = token or secrets.token_urlsafe(18)
        self.buttons = Buttons()
        self.state = WebCommandState(command_timeout, allow_head_control)
        self.camera_enabled = camera
        self.camera_size = camera_size
        self.camera_fps = camera_fps
        self.camera_stream = None
        self._started = threading.Event()
        self._startup_error: Exception | None = None

        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        if not self._started.wait(timeout=10):
            raise RuntimeError("Timed out while starting the web controller")
        if self._startup_error:
            raise RuntimeError("Could not start the web controller") from self._startup_error

        print(f"Web control: http://<robot-ip>:{self.port}/?token={self.token}")

    def _authorized(self, request: web.Request) -> bool:
        supplied = request.query.get("token", "")
        return bool(supplied) and secrets.compare_digest(supplied, self.token)

    async def _index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(Path(__file__).with_name("web") / "index.html")

    async def _config(self, request: web.Request) -> web.Response:
        return web.json_response(
            {
                "camera": self.camera_stream is not None,
                "head_control": self.state.allow_head_control,
                "command_timeout_ms": round(self.state.timeout * 1000),
            }
        )

    async def _websocket(self, request: web.Request) -> web.StreamResponse:
        if not self._authorized(request):
            raise web.HTTPUnauthorized(text="Invalid control token")

        ws = web.WebSocketResponse(heartbeat=10, receive_timeout=30)
        await ws.prepare(request)
        await ws.send_json({"type": "ready"})
        try:
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    try:
                        payload = json.loads(message.data)
                        if payload.get("type") == "command":
                            self.state.update(payload)
                    except (json.JSONDecodeError, AttributeError):
                        await ws.send_json({"type": "error", "message": "Invalid command"})
                elif message.type == WSMsgType.ERROR:
                    break
        finally:
            self.state.disconnect()
        return ws

    async def _video(self, request: web.Request) -> web.StreamResponse:
        if not self._authorized(request):
            raise web.HTTPUnauthorized(text="Invalid control token")
        if self.camera_stream is None:
            raise web.HTTPNotFound(text="Camera streaming is disabled")

        response = web.StreamResponse(
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=FRAME",
                "Cache-Control": "no-store, no-cache, must-revalidate",
            }
        )
        await response.prepare(request)
        last_sequence = -1
        try:
            while True:
                frame, last_sequence = await asyncio.to_thread(
                    self.camera_stream.wait_for_frame, last_sequence, 2.0
                )
                if frame is None:
                    continue
                await response.write(
                    b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode()
                    + b"\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _start_app(self) -> None:
        if self.camera_enabled:
            from mini_bdx_runtime.camera_stream import CameraStream

            self.camera_stream = CameraStream(self.camera_size, self.camera_fps)
            self.camera_stream.start()

        app = web.Application()
        web_dir = Path(__file__).with_name("web")
        app.router.add_get("/", self._index)
        app.router.add_get("/api/config", self._config)
        app.router.add_get("/ws", self._websocket)
        app.router.add_get("/stream.mjpg", self._video)
        app.router.add_static("/static", web_dir)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        await web.TCPSite(runner, self.host, self.port).start()

    def _run_server(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._start_app())
        except Exception as exc:
            self._startup_error = exc
            self._started.set()
            return
        self._started.set()
        loop.run_forever()

    def get_last_command(self):
        commands, raw_buttons, left_trigger, right_trigger, _connected = self.state.snapshot()
        self.buttons.update(
            raw_buttons["A"],
            raw_buttons["B"],
            raw_buttons["X"],
            raw_buttons["Y"],
            raw_buttons["LB"],
            raw_buttons["RB"],
            raw_buttons["dpad_up"],
            raw_buttons["dpad_down"],
        )
        return np.array(commands), self.buttons, left_trigger, right_trigger

    def consume_desired_paused(self) -> bool | None:
        return self.state.consume_desired_paused()
