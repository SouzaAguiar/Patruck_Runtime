#!/usr/bin/env python3
"""Interacao visual do Patruck: rosto, gestos, expressoes e monitor web.

Este processo controla SOMENTE os servos da cabeca (IDs 31, 32 e 33),
os olhos, as antenas e o alto-falante. Nenhum comando e enviado as pernas.

Uso recomendado no Raspberry Pi, a partir da raiz do Runtime:

    workon patruck-runtime
    python scripts/patrick_human_interaction.py --rotation 90 --token duck-test

Primeiro teste, sem acionar motores nem recursos de expressao:

    python scripts/patrick_human_interaction.py --rotation 90 --dry-run

Abra no navegador: http://IP-DO-ROBO:8090/?token=TOKEN
"""

from __future__ import annotations

import argparse
import asyncio
import math
import secrets
import signal
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from aiohttp import web


HEAD_JOINTS = ("head_pitch", "head_yaw", "head_roll")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rotate_frame(frame: np.ndarray, angle: int) -> np.ndarray:
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


@dataclass
class RuntimeState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    frame_condition: threading.Condition = field(default_factory=threading.Condition)
    jpeg: bytes | None = None
    sequence: int = 0
    status: dict[str, Any] = field(
        default_factory=lambda: {
            "state": "iniciando",
            "face": False,
            "gesture": False,
            "faces": 0,
            "fps": 0.0,
            "head": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
            "message": "Preparando camera",
        }
    )

    def publish_frame(self, jpeg: bytes) -> None:
        with self.frame_condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.frame_condition.notify_all()

    def wait_frame(self, previous: int, timeout: float = 2.0):
        with self.frame_condition:
            self.frame_condition.wait_for(
                lambda: self.sequence != previous, timeout=timeout
            )
            return self.jpeg, self.sequence

    def update_status(self, **values: Any) -> None:
        with self.lock:
            self.status.update(values)

    def status_snapshot(self) -> dict[str, Any]:
        with self.lock:
            result = dict(self.status)
            result["head"] = dict(self.status["head"])
            return result


class FaceDetector:
    """Detector Haar leve sobre o fluxo reduzido pelo ISP da camera."""

    def __init__(self, every: int = 2):
        cascade = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"
        self.detector = cv2.CascadeClassifier(str(cascade))
        if self.detector.empty():
            raise RuntimeError(f"Nao foi possivel carregar o detector: {cascade}")
        self.every = max(1, every)
        self.counter = 0
        self.last_faces: list[tuple[int, int, int, int]] = []

    def detect(
        self, gray: np.ndarray, output_shape: tuple[int, int]
    ) -> list[tuple[int, int, int, int]]:
        self.counter += 1
        if self.counter % self.every:
            return self.last_faces

        source_height, source_width = gray.shape[:2]
        output_height, output_width = output_shape
        scale_x = output_width / source_width
        scale_y = output_height / source_height
        gray = cv2.equalizeHist(gray)
        found = self.detector.detectMultiScale(
            gray,
            scaleFactor=1.12,
            minNeighbors=5,
            minSize=(36, 36),
        )
        self.last_faces = [
            (
                int(x * scale_x),
                int(y * scale_y),
                int(w * scale_x),
                int(h * scale_y),
            )
            for x, y, w, h in found
        ]
        self.last_faces.sort(key=lambda box: box[2] * box[3], reverse=True)
        return self.last_faces


class MotionGestureDetector:
    """Detecta um gesto como movimento localizado proximo ao rosto.

    Nao tenta classificar a mao. Movimentos globais, comuns quando a propria
    cabeca gira, sao descartados para reduzir falsos positivos.
    """

    def __init__(self, cooldown: float = 3.0):
        self.previous: np.ndarray | None = None
        self.activity_frames = 0
        self.last_trigger = 0.0
        self.cooldown = cooldown
        self.score = 0.0

    def update(self, gray: np.ndarray, has_face: bool) -> bool:
        if gray.shape[1] > 240:
            scale = 240 / gray.shape[1]
            gray = cv2.resize(gray, None, fx=scale, fy=scale)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        if self.previous is None:
            self.previous = gray
            return False

        diff = cv2.absdiff(self.previous, gray)
        self.previous = gray
        mask = cv2.threshold(diff, 24, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = float(mask.shape[0] * mask.shape[1])
        areas = [cv2.contourArea(contour) / frame_area for contour in contours]
        localized = any(0.008 <= area <= 0.14 for area in areas)
        global_motion = sum(areas) > 0.32
        self.score = round(sum(areas), 3)

        if has_face and localized and not global_motion:
            self.activity_frames += 1
        else:
            self.activity_frames = max(0, self.activity_frames - 1)

        now = time.monotonic()
        if self.activity_frames >= 3 and now - self.last_trigger >= self.cooldown:
            self.activity_frames = 0
            self.last_trigger = now
            return True
        return False


class HeadOnlyController:
    """Envia posicoes somente aos tres motores da cabeca."""

    LIMITS = {
        "head_pitch": (math.radians(-35), math.radians(25)),
        "head_yaw": (math.radians(-35), math.radians(35)),
        "head_roll": (math.radians(-12), math.radians(12)),
    }

    def __init__(
        self,
        config_path: str,
        serial_port: str,
        kp: int,
        yaw_sign: float,
        pitch_sign: float,
        dry_run: bool,
        release_on_exit: bool,
    ):
        self.dry_run = dry_run
        self.release_on_exit = release_on_exit
        self.yaw_sign = yaw_sign
        self.pitch_sign = pitch_sign
        self.positions = {joint: 0.0 for joint in HEAD_JOINTS}
        self.last_send = 0.0
        self.hwi = None

        if dry_run:
            return

        from mini_bdx_runtime.duck_config import DuckConfig
        from mini_bdx_runtime.rustypot_position_hwi import HWI

        duck_config = DuckConfig(config_path)
        self.hwi = HWI(duck_config, usb_port=serial_port)
        self.ids = [self.hwi.joints[joint] for joint in HEAD_JOINTS]
        self.offsets = [self.hwi.joints_offsets[joint] for joint in HEAD_JOINTS]

        # Nunca chamar HWI.turn_on(): ele escreve posicoes e KP nos 14 motores.
        self.hwi.io.set_kps(self.ids, [kp] * len(self.ids))
        try:
            current = self.hwi.io.read_present_position(self.ids)
            self.positions = {
                joint: clamp(float(pos) - offset, *self.LIMITS[joint])
                for joint, pos, offset in zip(HEAD_JOINTS, current, self.offsets)
            }
        except Exception as exc:
            print(f"Aviso: leitura inicial da cabeca falhou ({exc}); usando centro.")

    def update(
        self,
        face_error: tuple[float, float] | None,
        state: str,
        now: float,
        dt: float,
    ) -> dict[str, float]:
        dead_zone = 0.10
        yaw = self.positions["head_yaw"]
        pitch = self.positions["head_pitch"]

        if face_error is not None:
            error_x, error_y = face_error
            if abs(error_x) > dead_zone:
                yaw += self.yaw_sign * error_x * math.radians(42) * dt
            if abs(error_y) > dead_zone:
                pitch += self.pitch_sign * error_y * math.radians(32) * dt
        elif state == "procurando":
            yaw = math.radians(22) * math.sin(now * 0.55)
            pitch = math.radians(-3 + 4 * math.sin(now * 0.31))

        target_roll = 0.0
        if state == "curioso":
            target_roll = math.radians(9) * math.sin(now * 2.1)

        max_step = math.radians(55) * dt
        self.positions["head_yaw"] += clamp(yaw - self.positions["head_yaw"], -max_step, max_step)
        self.positions["head_pitch"] += clamp(pitch - self.positions["head_pitch"], -max_step, max_step)
        self.positions["head_roll"] += clamp(
            target_roll - self.positions["head_roll"], -max_step, max_step
        )
        for joint in HEAD_JOINTS:
            self.positions[joint] = clamp(self.positions[joint], *self.LIMITS[joint])

        if now - self.last_send >= 1 / 15:
            self.last_send = now
            if not self.dry_run:
                goals = [
                    self.positions[joint] + offset
                    for joint, offset in zip(HEAD_JOINTS, self.offsets)
                ]
                self.hwi.io.write_goal_position(self.ids, goals)

        return {
            "pitch": round(math.degrees(self.positions["head_pitch"]), 1),
            "yaw": round(math.degrees(self.positions["head_yaw"]), 1),
            "roll": round(math.degrees(self.positions["head_roll"]), 1),
        }

    def stop(self) -> None:
        if not self.dry_run and self.release_on_exit:
            # Libera somente os tres motores da cabeca.
            self.hwi.io.disable_torque(self.ids)


class EyeAnimator:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.queue: list[int] = []
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.left = self.right = None
        if enabled:
            import board
            import digitalio

            self.left = digitalio.DigitalInOut(board.D24)
            self.right = digitalio.DigitalInOut(board.D23)
            self.left.direction = digitalio.Direction.OUTPUT
            self.right.direction = digitalio.Direction.OUTPUT
            self._set(True)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _set(self, value: bool) -> None:
        if self.enabled:
            self.left.value = value
            self.right.value = value

    def blink(self, count: int = 1) -> None:
        if self.enabled:
            with self.lock:
                self.queue.append(count)

    def _run(self) -> None:
        next_natural = time.monotonic() + 2.5
        while not self.stop_event.is_set():
            count = 0
            with self.lock:
                if self.queue:
                    count = self.queue.pop(0)
            now = time.monotonic()
            if count == 0 and now >= next_natural:
                count = 1
                next_natural = now + 2.0 + secrets.randbelow(300) / 100
            if count:
                for _ in range(count):
                    self._set(False)
                    if self.stop_event.wait(0.10):
                        break
                    self._set(True)
                    if self.stop_event.wait(0.14):
                        break
            self.stop_event.wait(0.03)

    def stop(self) -> None:
        if not self.enabled:
            return
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self._set(False)
        self.left.deinit()
        self.right.deinit()


class AntennaAnimator:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.pattern = "idle"
        self.pattern_started = time.monotonic()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.antennas = None
        if enabled:
            from mini_bdx_runtime.antennas import Antennas

            self.antennas = Antennas()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def trigger(self, pattern: str) -> None:
        if self.enabled:
            with self.lock:
                self.pattern = pattern
                self.pattern_started = time.monotonic()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                pattern, started = self.pattern, self.pattern_started
            elapsed = time.monotonic() - started
            if pattern == "face" and elapsed < 1.6:
                value = 0.42 * math.sin(elapsed * math.pi * 4)
                left, right = value, value
            elif pattern == "gesture" and elapsed < 2.0:
                left = 0.55 * math.sin(elapsed * math.pi * 3)
                right = -left
            else:
                left = right = 0.0
                if pattern != "idle":
                    with self.lock:
                        self.pattern = "idle"
            self.antennas.set_position_left(left)
            self.antennas.set_position_right(right)
            self.stop_event.wait(0.04)

    def stop(self) -> None:
        if self.enabled:
            self.stop_event.set()
            self.thread.join(timeout=1.0)
            self.antennas.stop()


class SoundResponder:
    def __init__(self, enabled: bool, assets_dir: Path):
        self.enabled = enabled
        self.player = None
        self.assets_dir = assets_dir
        self.last_play = 0.0
        if enabled:
            try:
                from mini_bdx_runtime.sounds import Sounds

                self.player = Sounds(volume=0.75, sound_directory=str(assets_dir))
                self.enabled = bool(self.player.ok)
            except Exception as exc:
                self.enabled = False
                print(f"Aviso: audio desativado ({exc}).")

    def play(self, event: str) -> None:
        now = time.monotonic()
        if not self.enabled or now - self.last_play < 2.2:
            return
        choices = {
            "face": ("beep1.wav", "happy1.wav"),
            "gesture": ("beep2.wav", "happy2.wav"),
        }.get(event, ("beep1.wav",))
        available = [name for name in choices if name in self.player.sounds]
        if available:
            self.player.play(secrets.choice(available))
            self.last_play = now


class Behavior:
    def __init__(self, eyes: EyeAnimator, antennas: AntennaAnimator, sounds: SoundResponder):
        self.eyes = eyes
        self.antennas = antennas
        self.sounds = sounds
        self.had_face = False
        self.last_face = 0.0
        self.curious_until = 0.0

    def update(self, has_face: bool, gesture: bool, now: float) -> tuple[str, str]:
        acquired = has_face and not self.had_face
        if has_face:
            self.last_face = now
        if acquired:
            self.curious_until = now + 2.0
            self.eyes.blink(2)
            self.antennas.trigger("face")
            self.sounds.play("face")
        if gesture:
            self.curious_until = now + 2.5
            self.eyes.blink(1)
            self.antennas.trigger("gesture")
            self.sounds.play("gesture")

        self.had_face = has_face
        if now < self.curious_until:
            return "curioso", "Investigando o movimento"
        if has_face:
            return "acompanhando", "Rosto localizado"
        if now - self.last_face < 1.2:
            return "aguardando", "Tentando reencontrar o rosto"
        return "procurando", "Procurando alguem"


PAGE = r"""<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#07131d"><title>Visao do Patruck</title>
<style>
:root{color-scheme:dark;font-family:Inter,system-ui,sans-serif;background:#07131d;color:#ecf7f6}
*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;padding:18px}
main{width:min(920px,100%)}header{display:flex;align-items:end;justify-content:space-between;margin-bottom:12px}
.tag{color:#65e6ca;font-size:.72rem;font-weight:800;letter-spacing:.14em}h1{margin:.18rem 0 0;font-size:clamp(1.45rem,4vw,2.5rem)}
.live{display:flex;gap:8px;align-items:center;color:#91a9ae}.dot{width:9px;height:9px;border-radius:50%;background:#ff6e6e}.online .dot{background:#65e6ca;box-shadow:0 0 12px #65e6ca}
.video{position:relative;border-radius:20px;overflow:hidden;background:#0b202b;box-shadow:0 24px 70px #0008;aspect-ratio:4/3}
.video img{width:100%;height:100%;display:block;object-fit:contain}.badge{position:absolute;left:14px;top:14px;padding:8px 11px;border-radius:99px;background:#061018cc;backdrop-filter:blur(8px);font-weight:750}
.grid{display:grid;grid-template-columns:1.5fr repeat(3,1fr);gap:10px;margin-top:10px}.card{background:#0d202a;border:1px solid #193743;border-radius:14px;padding:12px}.label{display:block;color:#829ca3;font-size:.7rem;text-transform:uppercase;letter-spacing:.09em;margin-bottom:5px}.value{font-size:1rem;font-weight:750}.curioso{color:#ffd166}.acompanhando{color:#65e6ca}
@media(max-width:650px){.grid{grid-template-columns:1fr 1fr}.message{grid-column:1/-1}}
</style></head><body><main><header><div><span class="tag">PATRUCK · VISAO LOCAL</span><h1>O que o robo esta vendo</h1></div><div id="live" class="live"><i class="dot"></i><span>Conectando</span></div></header>
<section class="video"><img id="video" alt="Camera do Patruck"><span id="badge" class="badge">INICIANDO</span></section>
<section class="grid"><div class="card message"><span class="label">Comportamento</span><span id="message" class="value">Preparando camera</span></div><div class="card"><span class="label">Faces</span><span id="faces" class="value">0</span></div><div class="card"><span class="label">Imagem</span><span id="fps" class="value">0 FPS</span></div><div class="card"><span class="label">Cabeca</span><span id="head" class="value">0° · 0°</span></div></section>
</main><script>
const token=new URLSearchParams(location.search).get('token')||'';
const live=document.querySelector('#live');
if(token)document.querySelector('#video').src=`/stream.mjpg?token=${encodeURIComponent(token)}`;
async function refresh(){try{const r=await fetch(`/api/status?token=${encodeURIComponent(token)}`,{cache:'no-store'});if(!r.ok)throw 0;const s=await r.json();live.classList.add('online');live.querySelector('span').textContent='Ao vivo';const b=document.querySelector('#badge');b.textContent=s.state.toUpperCase();b.className=`badge ${s.state}`;document.querySelector('#message').textContent=s.message;document.querySelector('#faces').textContent=s.faces;document.querySelector('#fps').textContent=`${s.fps.toFixed(1)} FPS`;document.querySelector('#head').textContent=`Y ${s.head.yaw}° · P ${s.head.pitch}°`;}catch(e){live.classList.remove('online');live.querySelector('span').textContent=token?'Reconectando':'Token ausente';}}
setInterval(refresh,500);refresh();
</script></body></html>"""


class WebMonitor:
    def __init__(self, state: RuntimeState, host: str, port: int, token: str):
        self.state = state
        self.host = host
        self.port = port
        self.token = token
        self.started = threading.Event()
        self.error: Exception | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        threading.Thread(target=self._run, daemon=True).start()
        if not self.started.wait(10) or self.error:
            raise RuntimeError("Falha ao iniciar a interface web") from self.error

    def _authorized(self, request: web.Request) -> bool:
        supplied = request.query.get("token", "")
        return bool(supplied) and secrets.compare_digest(supplied, self.token)

    async def _index(self, _request: web.Request) -> web.Response:
        return web.Response(text=PAGE, content_type="text/html")

    async def _status(self, request: web.Request) -> web.Response:
        if not self._authorized(request):
            raise web.HTTPUnauthorized(text="Token invalido")
        return web.json_response(self.state.status_snapshot())

    async def _stream(self, request: web.Request) -> web.StreamResponse:
        if not self._authorized(request):
            raise web.HTTPUnauthorized(text="Token invalido")
        response = web.StreamResponse(
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=FRAME",
                "Cache-Control": "no-store, no-cache, must-revalidate",
            }
        )
        await response.prepare(request)
        sequence = -1
        try:
            while True:
                jpeg, sequence = await asyncio.to_thread(
                    self.state.wait_frame, sequence, 2.0
                )
                if jpeg is not None:
                    await response.write(
                        b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _start(self) -> None:
        app = web.Application()
        app.router.add_get("/", self._index)
        app.router.add_get("/api/status", self._status)
        app.router.add_get("/stream.mjpg", self._stream)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        await web.TCPSite(runner, self.host, self.port).start()

    def _run(self) -> None:
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._start())
        except Exception as exc:
            self.error = exc
            self.started.set()
            return
        self.started.set()
        self.loop.run_forever()

    def stop(self) -> None:
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)


class PiCamera:
    """Dois fluxos: RGB para web e luminancia reduzida gerada pelo ISP."""

    def __init__(
        self, width: int, height: int, fps: int, detection_width: int
    ):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 ausente. Instale python3-picamera2 e use um venv "
                "com --system-site-packages."
            ) from exc
        self.camera = Picamera2()
        detection_height = max(2, round(height * detection_width / width / 2) * 2)
        self.lores_size = (detection_width, detection_height)
        config = self.camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            lores={"size": self.lores_size, "format": "YUV420"},
            controls={"FrameRate": fps},
            buffer_count=4,
        )
        self.camera.configure(config)
        self.camera.start()

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        (main, lores), _metadata = self.camera.capture_arrays(["main", "lores"])
        lores_width, lores_height = self.lores_size
        # Em YUV420, as primeiras H linhas sao o plano Y (luminancia).
        gray = lores[:lores_height, :lores_width]
        return main, gray

    def stop(self) -> None:
        self.camera.stop()
        self.camera.close()


def draw_overlay(
    rgb: np.ndarray,
    faces: list[tuple[int, int, int, int]],
    primary: tuple[int, int, int, int] | None,
    state_name: str,
) -> bytes:
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for box in faces:
        x, y, width, height = box
        color = (102, 230, 202) if box == primary else (130, 160, 165)
        cv2.rectangle(bgr, (x, y), (x + width, y + height), color, 2)
    cv2.putText(
        bgr,
        state_name.upper(),
        (14, bgr.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (102, 230, 202),
        2,
        cv2.LINE_AA,
    )
    ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 76])
    if not ok:
        raise RuntimeError("Falha ao codificar quadro JPEG")
    return encoded.tobytes()


def locate_assets(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).expanduser()
    script = Path(__file__).resolve()
    candidates = (
        script.parent.parent / "mini_bdx_runtime" / "assets",
        script.parent / "mini_bdx_runtime" / "assets",
        Path.cwd() / "mini_bdx_runtime" / "assets",
    )
    return next((path for path in candidates if path.is_dir()), candidates[0])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--token", default=None)
    parser.add_argument("--rotation", type=int, choices=(0, 90, 180, 270), default=90)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--web-fps", type=int, default=5)
    parser.add_argument("--detection-width", type=int, default=320)
    parser.add_argument("--detect-every", type=int, default=2)
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--config", default=str(Path.home() / "duck_config.json"))
    parser.add_argument("--assets-dir")
    parser.add_argument("--head-kp", type=int, default=8)
    parser.add_argument("--yaw-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--pitch-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--no-gestures", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Nao aciona atuadores nem audio")
    parser.add_argument(
        "--release-head-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    token = args.token or secrets.token_urlsafe(18)
    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

    feature_flags = {"eyes": False, "antennas": False, "speaker": False}
    if not args.dry_run:
        from mini_bdx_runtime.duck_config import DuckConfig

        config = DuckConfig(args.config)
        feature_flags = {
            "eyes": config.eyes,
            "antennas": config.antennas,
            "speaker": config.speaker,
        }

    state = RuntimeState()
    camera = PiCamera(args.width, args.height, args.fps, args.detection_width)
    head = HeadOnlyController(
        args.config,
        args.serial_port,
        args.head_kp,
        args.yaw_sign,
        args.pitch_sign,
        args.dry_run,
        args.release_head_on_exit,
    )
    eyes = EyeAnimator(feature_flags["eyes"] and not args.dry_run)
    antennas = AntennaAnimator(feature_flags["antennas"] and not args.dry_run)
    sounds = SoundResponder(
        feature_flags["speaker"] and not args.dry_run, locate_assets(args.assets_dir)
    )
    detector = FaceDetector(args.detect_every)
    gestures = None if args.no_gestures else MotionGestureDetector()
    behavior = Behavior(eyes, antennas, sounds)
    monitor = WebMonitor(state, args.host, args.port, token)

    print(f"Interface: http://<IP-DO-ROBO>:{args.port}/?token={token}")
    print("Controle ativo: somente head_pitch, head_yaw e head_roll.")
    if args.dry_run:
        print("MODO SEGURO: atuadores e audio desativados.")

    last = time.monotonic()
    fps_started = last
    fps_frames = 0
    measured_fps = 0.0
    last_web_frame = 0.0
    try:
        while not stop_event.is_set():
            frame, detection_gray = camera.read()
            frame = rotate_frame(frame, args.rotation)
            detection_gray = rotate_frame(detection_gray, args.rotation)
            now = time.monotonic()
            dt = clamp(now - last, 0.001, 0.2)
            last = now

            faces = detector.detect(detection_gray, frame.shape[:2])
            primary = faces[0] if faces else None
            gesture = (
                gestures.update(detection_gray, primary is not None) if gestures else False
            )
            state_name, message = behavior.update(primary is not None, gesture, now)

            error = None
            if primary is not None:
                x, y, width, height = primary
                center_x = x + width / 2
                center_y = y + height / 2
                error = (
                    (center_x - frame.shape[1] / 2) / (frame.shape[1] / 2),
                    (center_y - frame.shape[0] / 2) / (frame.shape[0] / 2),
                )
            head_status = head.update(error, state_name, now, dt)

            fps_frames += 1
            if now - fps_started >= 1.0:
                measured_fps = fps_frames / (now - fps_started)
                fps_started, fps_frames = now, 0

            state.update_status(
                state=state_name,
                message=message,
                face=primary is not None,
                gesture=gesture,
                faces=len(faces),
                fps=round(measured_fps, 1),
                head=head_status,
            )
            if now - last_web_frame >= 1 / max(1, args.web_fps):
                state.publish_frame(draw_overlay(frame, faces, primary, state_name))
                last_web_frame = now
    finally:
        state.update_status(state="parado", message="Processo encerrado")
        monitor.stop()
        eyes.stop()
        antennas.stop()
        head.stop()
        camera.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
