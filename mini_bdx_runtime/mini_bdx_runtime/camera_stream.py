"""Low-overhead MJPEG producer for Raspberry Pi Camera Module."""

from __future__ import annotations

import io
import threading


class _FrameOutput(io.BufferedIOBase):
    def __init__(self):
        self.condition = threading.Condition()
        self.frame: bytes | None = None
        self.sequence = 0

    def write(self, buffer):
        with self.condition:
            self.frame = bytes(buffer)
            self.sequence += 1
            self.condition.notify_all()
        return len(buffer)

    def wait_for_frame(self, previous_sequence: int, timeout: float):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence != previous_sequence, timeout)
            return self.frame, self.sequence


class CameraStream:
    def __init__(self, size=(640, 480), fps=10):
        self.size = size
        self.fps = fps
        self.output = _FrameOutput()
        self.camera = None

    def start(self):
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import MJPEGEncoder
            from picamera2.outputs import FileOutput
        except ImportError as exc:
            raise RuntimeError(
                "Camera mode requires Picamera2 (sudo apt install python3-picamera2)"
            ) from exc

        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": self.size, "format": "RGB888"},
            controls={"FrameRate": self.fps},
            buffer_count=4,
        )
        self.camera.configure(config)
        self.camera.start_recording(MJPEGEncoder(), FileOutput(self.output))

    def wait_for_frame(self, previous_sequence: int, timeout: float):
        return self.output.wait_for_frame(previous_sequence, timeout)

    def stop(self):
        if self.camera is not None:
            self.camera.stop_recording()
            self.camera.close()

