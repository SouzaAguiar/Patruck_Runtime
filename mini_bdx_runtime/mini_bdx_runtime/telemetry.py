"""Bounded asynchronous telemetry; no hardware imports or control decisions."""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
from queue import Queue, Empty, Full
import sys
from threading import Event, Thread
import time
from uuid import uuid4


def file_sha256(path):
    path = Path(path)
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def plain(value):
    """Take an independent JSON snapshot, converting nonfinite readings to null."""
    if hasattr(value, 'tolist'):
        return plain(value.tolist())
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(plain(value), stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class TelemetryRecorder:
    def __init__(self, root, metadata=None, label='test', flush_seconds=1.0, queue_size=512, writer_yield_ms=0):
        if flush_seconds <= 0 or queue_size < 1:
            raise ValueError('flush_seconds and queue_size must be positive')
        if not math.isfinite(writer_yield_ms) or not 0 <= writer_yield_ms <= 2:
            raise ValueError('writer_yield_ms must be in [0, 2]')
        self.writer_yield_s = writer_yield_ms/1000
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.folder = Path(root) / (stamp + '-' + uuid4().hex[:8])
        self.folder.mkdir(parents=True, exist_ok=False)
        self.queue = Queue(maxsize=queue_size)
        self.stop = Event()
        self.flush_seconds = flush_seconds
        self.sequence = self.dropped = self.written = self.chunks = 0
        self.error = None
        self.closed = False
        self.close_reason = 'completed'
        atomic_json(self.folder / 'metadata.json', {
            'schema': 1, 'label': label, 'started_utc': stamp,
            'clock_anchor': {'unix_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns()},
            'python': sys.version, 'platform': platform.platform(),
            'flush_seconds': flush_seconds, 'queue_capacity_records': queue_size,
            'writer_yield_ms': writer_yield_ms,
            'nonfinite_values': 'null', 'settings': metadata or {},
        })
        self._status('recording')
        self.thread = Thread(target=self._worker, name='robot-telemetry', daemon=True)
        self.thread.start()
        print('Telemetria:', self.folder.resolve(), flush=True)

    def record(self, kind, **fields):
        if self.closed:
            return False
        sequence = self.sequence
        self.sequence += 1
        if self.error is not None:
            self.dropped += 1
            return False
        row = plain(dict(fields, kind=kind, sequence=sequence,
                         recorded_monotonic_ns=time.monotonic_ns(), recorded_unix_ns=time.time_ns()))
        try:
            self.queue.put_nowait(row)
            return True
        except Full:
            self.dropped += 1
            if self.dropped == 1:
                print('TELEMETRIA: fila cheia; registros perdidos serao contabilizados.', file=sys.stderr)
            return False

    def _status(self, state):
        atomic_json(self.folder / 'status.json', {
            'state': state, 'written_records': self.written, 'chunks': self.chunks,
            'attempted_records': self.sequence, 'dropped_records': self.dropped,
            'unwritten_records': self.sequence - self.written,
            'writer_error': self.error, 'close_reason': self.close_reason if self.closed else None,
        })

    def _publish(self, rows):
        path = self.folder / f'chunk-{self.chunks:06d}.jsonl'
        temporary = path.with_suffix('.jsonl.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
                if self.writer_yield_s:
                    time.sleep(self.writer_yield_s)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        self.written += len(rows)
        self.chunks += 1
        self._status('recording')

    def _worker(self):
        rows = []
        deadline = time.monotonic() + self.flush_seconds
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    rows.append(self.queue.get(timeout=min(.1, max(.001, deadline-time.monotonic()))))
                except Empty:
                    pass
                if rows and (len(rows) >= 256 or time.monotonic() >= deadline):
                    self._publish(rows)
                    rows = []
                    deadline = time.monotonic() + self.flush_seconds
            if rows:
                self._publish(rows)
            self._status('closed_with_loss' if self.dropped else 'closed')
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
            print('TELEMETRIA: falha de gravacao: ' + self.error, file=sys.stderr, flush=True)
            try:
                self._status('failed')
            except Exception:
                pass

    def close(self, reason='completed'):
        if self.closed:
            return
        self.record('session_end', reason=reason)
        self.close_reason = reason
        self.closed = True
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            print('TELEMETRIA: gravacao ainda pendente; confira os blocos e status.', file=sys.stderr)
        elif self.error:
            # Reflect further drops after a writer failure, when the disk permits.
            try:
                self._status('failed')
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        reason = 'interrupted' if exc_type is KeyboardInterrupt else 'error' if exc_type else 'completed'
        if exc is not None:
            self.record('session_error', error_type=exc_type.__name__, message=str(exc))
        self.close(reason)
