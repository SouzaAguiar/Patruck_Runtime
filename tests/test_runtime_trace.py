"""Timing instrumentation and stationary guard with sensor/motor stubs only."""
import gc
import importlib.util
import json
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from test_telemetry import ROOT, load_walk, make_walk
from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.runtime_trace import CONTROL_IDS, NumericRing, RuntimeTimingTrace, runtime_trace_session
from mini_bdx_runtime.imu_safety import ImuDataError


def test_ring_keeps_latest_rows_and_reports_overwrite():
    ring = NumericRing(3, ['time', 'value'])
    original = ring.data
    for i in range(10):
        ring.add(i, i*2)
    result = ring.export()
    assert ring.data is original and len(ring.data) == 6
    assert result['total'] == 10 and result['overwritten'] == 7
    assert result['rows'] == [[7,14], [8,16], [9,18]]


def test_i2c_wrapper_adds_no_reads_and_preserves_error_unlock():
    trace = RuntimeTimingTrace(seconds=1)
    calls = []
    class Device:
        def __enter__(self):
            calls.append('lock')
        def __exit__(self, *args):
            calls.append('unlock')
        def write_then_readinto(self, outgoing, incoming, **kwargs):
            calls.append((bytes(outgoing), kwargs))
            raise OSError('injected I2C fault')
    with pytest.raises(OSError, match='injected'):
        with trace.wrap_i2c(Device()) as bus:
            bus.write_then_readinto(bytearray([0,20]), bytearray(6), out_start=1)
    assert calls == ['lock', (bytes([0,20]), {'out_start':1}), 'unlock']
    events = trace.i2c.export()['rows']
    assert len(events) == 2 and events[0][1:] == [0,20,0] and events[1][1:] == [1,20,1]
    assert events[1][0] >= events[0][0]


def test_reader_records_failure_read_and_real_publication_without_refreshing_age():
    trace = RuntimeTimingTrace(seconds=1)
    reader = Imu.__new__(Imu)
    reader.timing_trace = trace
    reader.imu_frame = 'yaw-minus-90'
    reader.x_offset = 0
    reader.sampling_freq = 50
    reader._stop_event = Event()
    captured, failures = [], []
    class Sensor:
        calls = 0
        @property
        def gyro(self):
            self.calls += 1
            if self.calls == 1:
                raise OSError('injected read failure')
            return [1,2,3]
        acceleration = [4,5,6]
    def publish(sample):
        captured.append(sample)
        reader._stop_event.set()
    reader.imu = Sensor()
    reader.samples = SimpleNamespace(publish=publish, fail=failures.append)
    reader.bus = SimpleNamespace(deinit=lambda:None)
    reader.imu_worker()
    rows = trace.imu.export()['rows']
    assert [(r[1],r[2]) for r in rows] == [(0,0),(3,0),(0,1),(1,1),(2,1)]
    sample = captured[0]
    assert rows[2][0] == sample['sample_start_monotonic_ns']
    assert rows[3][0] == sample['sample_end_monotonic_ns']
    assert rows[4][0] >= sample['sample_end_monotonic_ns']
    assert rows[2][4] == rows[0][0]+20_000_000
    assert failures[0].startswith('OSError: injected')
    np.testing.assert_array_equal(sample['gyro'], [2,-1,3])


def test_trace_exports_on_exception_and_unregisters_gc(tmp_path):
    callbacks = list(gc.callbacks)
    enabled = gc.isenabled()
    with pytest.raises(RuntimeError, match='injected'):
        with runtime_trace_session(tmp_path, 1, 50) as trace:
            trace.control_mark('imu_fault', 123, 2, 9)
            trace.observe_gc('start', {'generation':2})
            raise RuntimeError('injected shutdown')
    assert gc.callbacks == callbacks and gc.isenabled() == enabled
    saved = json.loads((tmp_path/'runtime_timing_trace.json').read_text())
    assert saved['control']['rows'][0][2:] == [2,9]
    assert saved['control_events'][str(saved['control']['rows'][0][1])] == 'imu_fault'
    assert saved['gc']['rows']
    assert saved['allocated_numeric_bytes'] < 1_000_000


def test_trace_disabled_adds_no_callbacks_or_output(tmp_path):
    before = list(gc.callbacks)
    with runtime_trace_session(None, 0, 50) as trace:
        assert trace is None and list(gc.callbacks) == before
    assert not list(tmp_path.iterdir())


def test_control_trace_preserves_actual_actions_and_marks_serial_operations():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    old, old_inputs, old_writes = make_walk(cls)
    old.run()
    trace = RuntimeTimingTrace(seconds=1)
    recorder = SimpleNamespace(record=lambda *args,**kwargs:None)
    new, inputs, writes = make_walk(cls, recorder)
    new.timing_trace = trace
    new.run()
    np.testing.assert_array_equal(inputs, old_inputs)
    assert writes == old_writes
    names = {v:k for k,v in CONTROL_IDS.items()}
    events = [names[r[1]] for r in trace.control.export()['rows']]
    assert events.count('motor_write_end') == events.count('telemetry_end') == 3
    assert events.index('read_joint_positions') < events.index('read_joint_velocities') < events.index('before_motor_write')


def test_fault_trace_keeps_aborted_cycle_and_never_writes():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, inputs, writes = make_walk(cls)
    trace = RuntimeTimingTrace(seconds=1)
    walk.timing_trace = trace
    walk.imu.get_data = lambda: (_ for _ in ()).throw(ImuDataError('injected stale'))
    cls.run.__globals__['time'].sleep = lambda s: (_ for _ in ()).throw(KeyboardInterrupt()) if walk.paused else None
    walk.run()
    events = [r[1] for r in trace.control.export()['rows']]
    assert CONTROL_IDS['imu_fault'] in events
    assert CONTROL_IDS['before_motor_write'] not in events
    assert not inputs and not writes and walk.paused


@pytest.mark.parametrize('command', [.01, float('nan')])
def test_stationary_guard_pauses_before_inference_and_motor_write(command):
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, inputs, writes = make_walk(cls)
    walk.stationary_test = True
    walk.last_commands = [command]+[0.]*6
    cls.run.__globals__['time'].sleep = lambda s: (_ for _ in ()).throw(KeyboardInterrupt())
    walk.run()
    assert walk.paused and 'Teste parado' in walk.runtime_fault
    assert not inputs and not writes and walk.imitation_i == 0
    assert walk.last_commands == [0.]*7


def test_stationary_guard_allows_zero_policy_commands():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, inputs, writes = make_walk(cls)
    walk.stationary_test = True
    walk.last_commands = [0.]*7
    walk.run()
    assert len(inputs) == len(writes) == 3
    assert all(not np.any(obs[6:13]) for obs in inputs)


def test_stationary_wall_deadline_exits_and_stops_reader_while_paused():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, inputs, writes = make_walk(cls)
    clock = [1_000_000_000]
    walk.stationary_test = True
    walk.test_duration_s = .2
    walk.paused = True
    stopped, events = [], []
    walk.imu.stop = lambda:stopped.append(True)
    walk.telemetry = SimpleNamespace(record=lambda kind,**fields:events.append(kind))
    cls.run.__globals__['time'].monotonic_ns = lambda:clock[0]
    cls.run.__globals__['time'].sleep = lambda s:clock.__setitem__(0,clock[0]+int(s*1e9))
    walk.run()
    assert stopped == [True] and not inputs and not writes
    assert 'stationary_test_complete' in events


def script_module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_offline_summary_correlates_producer_pending_i2c_and_fault(tmp_path):
    trace = RuntimeTimingTrace(seconds=1)
    trace.imu.add(1_000_000, 0, 7, 1_000_000, 0)
    trace.imu.add(3_000_000, 1, 7, 1_000_000, 0)
    trace.imu.add(4_000_000, 2, 7, 1_000_000, 0)
    trace.i2c.add(5_000_000, 0, 20, 0)
    trace.i2c.add(60_000_000, 1, 20, 0)
    trace.control_mark('imu_fault', 51_000_000, 2, 7)
    trace.gc.add(50_000_000, 0, 2, 0, 0)
    trace.gc.add(70_000_000, 1, 2, 0, 0)
    trace.close(tmp_path)
    fault = dict(kind='imu_fault', detected_monotonic_ns=51_000_000, message='stale',
                 used_sample={'sample_index':7}, diagnostic={'stage':'get_imu'})
    (tmp_path/'chunk-000000.jsonl').write_text(json.dumps(fault)+'\n')
    module = script_module('summarize_runtime_trace')
    report = module.summarize(tmp_path)
    assert report['acquisition_ms']['max_ms'] == 2
    assert report['publication_after_read_ms']['max_ms'] == 1
    event = report['faults'][0]
    assert event['used_sample_producer']['index'] == 7
    assert event['i2c_calls_overlapping_fault'][0]['duration_ms'] == 55
    assert event['gc_overlapping_fault'][0]['duration_ms'] == 20


def test_launcher_dry_run_checks_teacher_and_launches_no_hardware(tmp_path, monkeypatch, capsys):
    module = script_module('runtime_stationary_test')
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    library = tmp_path/'mini_bdx_runtime/mini_bdx_runtime'
    library.mkdir(parents=True)
    for name in ('imu_calib_data.pkl','polynomial_coefficients.pkl','v2_rl_walk_mujoco.py'):
        (scripts/name).write_bytes(b'test file')
    (scripts/'v2_rl_walk_mujoco.py').write_text('IMU_SELECTION_VERSION = 3\n')
    (library/'runtime_trace.py').write_bytes(b'test file')
    (library/'imu_safety.py').write_text('class ImuSampleStaleError: pass\n')
    config, model = tmp_path/'duck_config.json', tmp_path/'teacher.onnx'
    config.write_text('{}')
    model.write_bytes(b'mock teacher')
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(module, 'SCRIPTS', scripts)
    monkeypatch.setattr(module, 'EXPECTED_TEACHER', module.digest(model))
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a,**kw:pytest.fail('Dry run launched runtime'))
    assert module.main(['--onnx-model',str(model),'--config',str(config),'--dry-run']) == 0
    output = capsys.readouterr().out
    assert '--stationary-test' in output and '--imu-frame yaw-minus-90' in output
    assert '--active-window-s 10' in output and '--test-duration-s 90' in output
    (library/'imu_safety.py').write_text('class ImuDataError: pass\n')
    with pytest.raises(SystemExit):
        module.main(['--onnx-model',str(model),'--config',str(config),'--dry-run'])
    (library/'imu_safety.py').write_text('class ImuSampleStaleError: pass\n')
    model.write_bytes(b'wrong teacher')
    with pytest.raises(SystemExit):
        module.main(['--onnx-model',str(model),'--config',str(config),'--dry-run'])
