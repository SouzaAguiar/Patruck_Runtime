"""Bounded active windows and GC maintenance. No hardware imports."""
import gc
import math
from pathlib import Path
import time


class RuntimeBudgetError(RuntimeError):
    pass


def memory_snapshot():
    try:
        status = dict(line.split(':', 1) for line in Path('/proc/self/status').read_text().splitlines() if ':' in line)
        info = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines() if ':' in line)
        return int(status['VmRSS'].split()[0])*1024, int(info['MemAvailable'].split()[0])*1024
    except (OSError, ValueError, KeyError) as exc:
        raise RuntimeBudgetError('Memoria indisponivel: requer Linux /proc') from exc


class RuntimeBudget:
    def __init__(self, active_seconds=30, growth_mb=64, available_mb=64, recorder=None):
        if not math.isfinite(active_seconds) or not 0 < active_seconds <= 120:
            raise ValueError('active_seconds must be in (0, 120]')
        if any(not math.isfinite(x) or x <= 0 for x in (growth_mb, available_mb)):
            raise ValueError('Memory limits must be positive and finite')
        self.seconds = active_seconds
        self.growth = int(growth_mb*1024**2)
        self.available = int(available_mb*1024**2)
        self.recorder = recorder
        self.original_gc = gc.isenabled()
        self.started = None
        self.baseline = None
        self.next_memory_check = 0
        self.needs_maintenance = True
        self.preflight()

    def record(self, kind, **fields):
        if self.recorder is not None:
            self.recorder.record(kind, **fields)

    def preflight(self):
        rss, available = memory_snapshot()
        if available < self.available:
            raise RuntimeBudgetError('Memoria disponivel abaixo do limite')
        if self.baseline is not None and rss-self.baseline > self.growth:
            raise RuntimeBudgetError('Crescimento de memoria acima do limite; encerre e investigue')
        return rss, available

    def _restore_gc(self):
        gc.enable() if self.original_gc else gc.disable()

    def pause(self):
        if self.started is not None:
            self.record('runtime_active_end', elapsed_s=time.monotonic()-self.started)
        self.started = None
        self._restore_gc()

    def maintain(self):
        if self.started is not None:
            raise RuntimeError('GC maintenance requires paused control')
        if not self.needs_maintenance:
            return
        start = time.monotonic_ns()
        before, _ = memory_snapshot()
        collected = gc.collect()
        after, available = memory_snapshot()
        if self.baseline is None:
            self.baseline = after  # Keep across resumes; do not hide cumulative growth.
        self.record('runtime_gc_maintenance', start_monotonic_ns=start,
                    end_monotonic_ns=time.monotonic_ns(), collected=collected,
                    rss_before_bytes=before, rss_after_bytes=after, available_bytes=available)
        self.needs_maintenance = False
        self.preflight()

    def begin(self):
        if self.started is not None:
            return  # Repeated web start messages cannot extend an active window.
        if self.needs_maintenance:
            raise RuntimeBudgetError('Manutencao pendente antes de iniciar')
        self.preflight()
        if self.recorder is not None and (self.recorder.error or self.recorder.dropped):
            raise RuntimeBudgetError('Falha ou perda na telemetria; encerre e investigue')
        gc.disable()
        self.started = time.monotonic()
        self.next_memory_check = 0
        self.needs_maintenance = True
        self.record('runtime_active_start', limit_s=self.seconds, gc_enabled=gc.isenabled())

    def check(self):
        if self.started is None:
            raise RuntimeBudgetError('Sessao ativa nao inicializada')
        now = time.monotonic()
        if now-self.started >= self.seconds:
            raise RuntimeBudgetError(f'Limite de {self.seconds:g} s atingido; apoie o robo e reconheca a pausa')
        if self.recorder is not None and (self.recorder.error or self.recorder.dropped):
            raise RuntimeBudgetError('Falha ou perda na telemetria; encerre e investigue')
        if now >= self.next_memory_check:
            rss, available = self.preflight()
            self.next_memory_check = now+1
            self.record('runtime_memory', rss_bytes=rss, available_bytes=available,
                        baseline_rss_bytes=self.baseline, elapsed_s=now-self.started)

    def close(self):
        self.started = None
        self._restore_gc()
