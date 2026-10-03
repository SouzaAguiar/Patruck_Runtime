"""Optional bounded timing recorder. No hardware imports or bus reads."""
from array import array
from contextlib import contextmanager
import gc
import math
from pathlib import Path
import time

from mini_bdx_runtime.telemetry import atomic_json


CONTROL_EVENTS = (
    'cycle_start', 'commands_end', 'paused',
    'runtime_budget_before_observation', 'get_imu', 'validate_imu',
    'read_joint_positions', 'read_joint_velocities', 'read_contacts',
    'after_joint_reads', 'select_imu_before_inference',
    'runtime_budget_before_inference', 'inference',
    'after_inference_used_sample', 'after_inference_latest_sample',
    'prepare_targets', 'runtime_budget_before_write', 'before_motor_write',
    'motor_write_end', 'telemetry_begin', 'telemetry_end', 'cycle_work_end',
    'imu_fault', 'runtime_guard_pause', 'resume_imu_ready',
)
CONTROL_IDS = {name: i for i, name in enumerate(CONTROL_EVENTS)}


class NumericRing:
    """One producer per ring; export only after producers have stopped."""
    def __init__(self, capacity, columns):
        self.capacity = capacity
        self.columns = columns
        self.width = len(columns)
        self.data = array('q', [0]) * (capacity * self.width)
        self.total = 0
        self.enabled = True

    def add(self, *values):
        if not self.enabled:
            return
        offset = (self.total % self.capacity) * self.width
        for i, value in enumerate(values):
            self.data[offset+i] = value
        self.total += 1

    def export(self):
        first = max(0, self.total-self.capacity)
        return dict(columns=self.columns, capacity=self.capacity, total=self.total,
                    overwritten=first,
                    rows=[list(self.data[(i % self.capacity)*self.width:(i % self.capacity+1)*self.width])
                          for i in range(first, self.total)])


class TimedI2CDevice:
    """Wrap existing calls. Begin events retain evidence of an unfinished call."""
    def __init__(self, device, ring):
        self.device, self.ring = device, ring

    def __enter__(self):
        self.device.__enter__()
        return self

    def __exit__(self, *args):
        return self.device.__exit__(*args)

    def __getattr__(self, name):
        return getattr(self.device, name)

    def write_then_readinto(self, outgoing, incoming, **kwargs):
        register = int(outgoing[kwargs.get('out_start', 0)])
        self.ring.add(time.monotonic_ns(), 0, register, 0)
        failed = 0
        try:
            return self.device.write_then_readinto(outgoing, incoming, **kwargs)
        except BaseException:
            failed = 1
            raise
        finally:
            self.ring.add(time.monotonic_ns(), 1, register, failed)


class RuntimeTimingTrace:
    def __init__(self, seconds=120, frequency=50):
        if not math.isfinite(seconds) or not 0 < seconds <= 300:
            raise ValueError('runtime trace seconds must be in (0, 300]')
        if not math.isfinite(frequency) or not 1 <= frequency <= 200:
            raise ValueError('traced control frequency must be in [1, 200]')
        self.seconds, self.frequency = seconds, frequency
        samples = math.ceil(seconds * frequency)
        self.imu = NumericRing(samples*3+32, ['time_ns', 'event', 'sample_index', 'sample_start_ns', 'due_ns'])
        self.i2c = NumericRing(samples*8+64, ['time_ns', 'end', 'register', 'failed'])
        self.control = NumericRing(samples*32+64, ['time_ns', 'event', 'attempt', 'sample_index'])
        self.gc = NumericRing(4096, ['time_ns', 'stop', 'generation', 'collected', 'uncollectable'])
        self.rings = (self.imu, self.i2c, self.control, self.gc)
        self.started_ns = time.monotonic_ns()
        self.closed = False
        self._installed = False

    def wrap_i2c(self, device):
        return TimedI2CDevice(device, self.i2c)

    def control_mark(self, event, now_ns, attempt=-1, sample_index=-1):
        self.control.add(now_ns, CONTROL_IDS[event], attempt, sample_index)

    def observe_gc(self, phase, info):
        self.gc.add(time.monotonic_ns(), int(phase == 'stop'), info.get('generation', -1),
                    info.get('collected', 0), info.get('uncollectable', 0))

    def install(self):
        if not self._installed:
            gc.callbacks.append(self.observe_gc)
            self._installed = True

    def close(self, folder):
        if self.closed:
            return
        self.closed = True
        if self._installed:
            gc.callbacks.remove(self.observe_gc)
            self._installed = False
        for ring in self.rings:
            ring.enabled = False
        atomic_json(Path(folder)/'runtime_timing_trace.json', dict(
            schema=1, clock='monotonic_ns', started_ns=self.started_ns, ended_ns=time.monotonic_ns(),
            retention_seconds_requested=self.seconds, frequency_hz=self.frequency,
            allocated_numeric_bytes=sum(len(r.data)*r.data.itemsize for r in self.rings),
            control_events={i:name for i,name in enumerate(CONTROL_EVENTS)},
            imu_events={0:'acquisition_start', 1:'read_end', 2:'published', 3:'failure'},
            imu=self.imu.export(), i2c=self.i2c.export(), control=self.control.export(), gc=self.gc.export()))


@contextmanager
def runtime_trace_session(folder, seconds, frequency):
    if seconds == 0:
        yield None
        return
    if folder is None:
        raise ValueError('Runtime timing trace requires telemetry directory')
    trace = RuntimeTimingTrace(seconds, frequency)
    trace.install()
    try:
        yield trace
    finally:
        trace.close(folder)
