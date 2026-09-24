"""Failure-path tests: all buses, sensors and motors are in-memory stubs."""
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_telemetry import ROOT, load_walk, make_walk
from mini_bdx_runtime.imu_safety import (
    CheckedModeDevice, ImuDataError, LatestImuSample, check_sample, open_i2c,
)


def sample(index=0):
    now = time.monotonic_ns()
    return dict(gyro=[0., 0., 0.], accelero=[0., 0., 9.81], sample_index=index,
                sample_start_monotonic_ns=now-1_000_000, sample_end_monotonic_ns=now)


def walk():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    instance, inputs, writes = make_walk(cls)
    # A failed iteration must latch pause, not spin, retry inference, or write.
    def sleep(seconds):
        if instance.imu_fault:
            raise KeyboardInterrupt
    cls.run.__globals__['time'].sleep = sleep
    return cls, instance, inputs, writes


def test_explicit_bus_never_falls_back(monkeypatch):
    seen = []
    def unavailable(number):
        seen.append(number)
        raise OSError('bus unavailable')
    monkeypatch.setitem(sys.modules, 'adafruit_extended_bus', SimpleNamespace(ExtendedI2C=unavailable))
    monkeypatch.setitem(sys.modules, 'board', None)
    monkeypatch.setitem(sys.modules, 'busio', None)
    with pytest.raises(OSError, match='unavailable'):
        open_i2c(8)
    assert seen == [8]


def test_latest_sample_replaces_unconsumed_data_and_returns_snapshot():
    buffer = LatestImuSample(.05)
    for i in range(500):
        buffer.publish(sample(i))
    result = buffer.get()
    assert result['sample_index'] == 499
    result['gyro'][0] = 999
    assert buffer.get()['gyro'][0] == 0


def test_failure_does_not_return_previous_good_sample_and_readiness_requires_recovery():
    buffer = LatestImuSample(.05)
    for i in range(3):
        buffer.publish(sample(i))
    buffer.wait_ready(0)
    buffer.fail('I2C failed')
    with pytest.raises(ImuDataError, match='I2C failed'):
        buffer.get()
    buffer.publish(sample(3))
    with pytest.raises(ImuDataError):
        buffer.wait_ready(0)
    buffer.publish(sample(4))
    buffer.publish(sample(5))
    assert buffer.wait_ready(0)['sample_index'] == 5


@pytest.mark.parametrize('field,value', [
    ('gyro', [float('nan'), 0, 0]), ('gyro', [None, 0, 0]), ('gyro', [36, 0, 0]),
    ('accelero', [0, 0, -317.83]), ('accelero', [0, 0]),
    ('sample_start_monotonic_ns', None),
])
def test_bad_samples_are_rejected(field, value):
    data = sample()
    data[field] = value
    with pytest.raises(ImuDataError):
        check_sample(data, .05)


def test_age_includes_time_inside_i2c_even_if_delivery_is_fresh():
    data = sample()
    data['sample_start_monotonic_ns'] -= 80_000_000
    with pytest.raises(ImuDataError, match='antiga'):
        check_sample(data, .05)


def test_corrupt_mode_checked_in_same_transaction_and_lock_released():
    class Device:
        locked = False
        calls = 0
        def __enter__(self):
            self.locked = True
        def __exit__(self, *args):
            self.locked = False
        def write_then_readinto(self, outgoing, incoming, **kwargs):
            self.calls += 1
            assert outgoing[0] == 0x3D
            incoming[1] = 0x8C
    device = Device()
    checked = CheckedModeDevice(device)
    buffer = bytearray([0x3D, 0])
    with pytest.raises(ImuDataError, match='8c'):
        with checked as bus:
            bus.write_then_readinto(buffer, buffer, out_end=1, in_start=1)
    assert not device.locked and device.calls == 1 and buffer[1] == 0x8C


def test_motor_start_never_runs_without_ready_imu():
    cls, instance, _, writes = walk()
    def fail(**kwargs):
        raise ImuDataError('not ready')
    instance.imu.wait_ready = fail
    instance.hwi.set_kps = lambda *args: pytest.fail('Motor access before valid IMU')
    with pytest.raises(ImuDataError, match='not ready'):
        cls.start(instance)
    assert not writes


def test_constructor_selects_8_and_missing_bus_never_opens_motors():
    cls, _, _, _ = walk()
    env = cls.__init__.__globals__
    env['DuckConfig'] = lambda **kwargs: SimpleNamespace(imu_upside_down=False)
    env['OnnxInfer'] = lambda *args, **kwargs: None
    def no_bus(**kwargs):
        assert kwargs['i2c_bus'] == 8
        raise OSError('missing bus 8')
    env['Imu'] = no_bus
    env['HWI'] = lambda *args: pytest.fail('Motor constructor should not be reached')
    with pytest.raises(OSError, match='missing bus 8'):
        cls('mock.onnx')


def test_sensor_failure_latches_pause_and_does_not_infer_or_write():
    _, instance, inputs, writes = walk()
    def fail():
        raise ImuDataError('I2C fault')
    instance.imu.get_data = fail
    instance.run()
    assert instance.paused and instance.imu_fault == 'I2C fault'
    assert not inputs and not writes


@pytest.mark.parametrize('stage', ['joints', 'inference', 'before_write'])
def test_sample_aging_during_work_never_writes_or_commits_action_history(stage):
    cls, instance, inputs, writes = walk()
    initial_targets = instance.motor_targets.copy()
    events = []
    instance.telemetry = SimpleNamespace(record=lambda kind, **fields: events.append(dict(kind=kind, **fields)))
    def expire():
        instance.observation_imu['sample_start_monotonic_ns'] -= 100_000_000
    if stage == 'joints':
        original = instance.hwi.get_present_positions
        def positions(**kwargs):
            expire()
            return original(**kwargs)
        instance.hwi.get_present_positions = positions
    elif stage == 'inference':
        original = instance.policy.infer
        def infer(obs):
            result = original(obs)
            expire()
            return result
        instance.policy.infer = infer
    else:
        def make_action(targets, names):
            expire()
            return dict(zip(names, targets))
        cls.run.__globals__['make_action_dict'] = make_action
    instance.run()
    assert not writes and instance.imu_fault
    fault = next(e for e in events if e['kind'] == 'imu_fault')
    assert fault['diagnostic']['stage'] == {
        'joints': 'after_joint_reads', 'inference': 'after_inference_used_sample',
        'before_write': 'before_motor_write'}[stage]
    assert fault['used_sample']['sample_index'] == 4
    assert fault['detected_monotonic_ns'] >= fault['diagnostic']['cycle_start_ns']
    assert fault['diagnostic']['next_action_cycle'] == 0
    assert instance.imitation_i == 0
    np.testing.assert_array_equal(instance.last_action, np.zeros(14))
    np.testing.assert_array_equal(instance.motor_targets, initial_targets)


def test_fault_needs_acknowledgement_centered_controls_and_good_imu():
    _, instance, _, _ = walk()
    instance._pause_for_imu(ImuDataError('stale'))
    instance._request_pause(False)
    assert instance.paused and instance.imu_fault
    instance._request_pause(True)
    instance._request_pause(False)  # Existing nonzero command must block resume.
    assert instance.paused and instance.imu_fault
    instance.last_commands = [0.]*7
    instance._request_pause(False)
    assert not instance.paused and instance.imu_fault is None


def test_raw_sensor_setup_failure_closes_bus(monkeypatch):
    from mini_bdx_runtime import raw_imu
    closed = []
    monkeypatch.setattr(raw_imu, 'open_i2c', lambda number: SimpleNamespace(deinit=lambda: closed.append(True)))
    monkeypatch.setitem(sys.modules, 'adafruit_bno055', SimpleNamespace(BNO055_I2C=lambda *a, **kw: object()))
    def fail(self, upside_down):
        raise ImuDataError('bad startup registers')
    monkeypatch.setattr(raw_imu.Imu, '_configure_sensor', fail)
    with pytest.raises(ImuDataError):
        raw_imu.Imu(50, i2c_bus=8)
    assert closed == [True]


def test_fault_context_is_captured_before_pause_restores_gc():
    _, instance, _, _ = walk()
    instance.cycle_diagnostics = dict(stage='get_imu', attempt=12)
    instance.observation_imu = None
    instance.imu.diagnostic_snapshot = lambda: {'latest_sample': {'sample_index': 9}, 'error': 'stale'}
    def pause():
        instance.cycle_diagnostics['stage'] = 'changed_by_pause'
    instance.runtime_budget = SimpleNamespace(pause=pause)
    events = []
    instance.telemetry = SimpleNamespace(record=lambda kind, **fields:events.append(fields),
                                        writer_snapshot=lambda:{'active':{'chunk':3}})
    instance._pause_for_imu(ImuDataError('stale'))
    assert events[0]['diagnostic']['stage'] == 'get_imu'
    assert events[0]['used_sample'] is None
    assert events[0]['latest_reader']['latest_sample']['sample_index'] == 9
    assert events[0]['writer']['active']['chunk'] == 3


def test_cached_reader_diagnostics_does_not_read_hardware():
    from mini_bdx_runtime.raw_imu import Imu
    reader = Imu.__new__(Imu)
    reader.samples = LatestImuSample(.05)
    reader.samples.publish(sample(19))
    reader.samples.fail('injected stale')
    snapshot = reader.diagnostic_snapshot()
    assert snapshot['latest_sample']['sample_index'] == 19
    assert snapshot['error'] == 'injected stale'
    snapshot['latest_sample']['sample_index'] = 90
    assert reader.diagnostic_snapshot()['latest_sample']['sample_index'] == 19


@pytest.mark.parametrize('arrives', [True, False])
def test_selection_waits_bounded_and_uses_new_values_without_committing(arrives):
    cls, instance, _, writes = walk()
    clock = [1_000_000_000]
    cls.run.__globals__['time'].monotonic_ns = lambda: clock[0]
    cls.run.__globals__['time'].sleep = lambda s: clock.__setitem__(0, clock[0] + round(s * 1e9))
    old = dict(gyro=[1., 2., 3.], accelero=[0., 0., 9.81], sample_index=10,
               sample_start_monotonic_ns=959_000_000, sample_end_monotonic_ns=980_000_000)
    new = dict(old, gyro=[4., 5., 6.], sample_index=11,
               sample_start_monotonic_ns=980_000_000, sample_end_monotonic_ns=1_001_000_000)
    instance.observation_imu = old
    instance.cycle_diagnostics = {'cycle_start_ns': clock[0] - 7_000_000}
    instance.imu.get_data = lambda: new if arrives and clock[0] >= 1_001_000_000 else old
    instance.telemetry = object()
    instance.telemetry_observation = {}
    obs = np.zeros(101)
    if arrives:
        selected = instance._select_imu_for_inference(obs)
        np.testing.assert_array_equal(selected[:6], new['gyro'] + new['accelero'])
        np.testing.assert_array_equal(selected[6:], obs[6:])
        assert instance.telemetry_observation['imu_sample_index'] == 11
        assert instance.telemetry_observation['imu_selection_wait_ms'] == 1
    else:
        with pytest.raises(ImuDataError, match='margem temporal'):
            instance._select_imu_for_inference(obs)
        assert clock[0] == 1_003_000_000  # Period minus reserve, stricter than 5 ms.
    assert not writes and instance.imitation_i == 0
    np.testing.assert_array_equal(instance.last_action, np.zeros(14))
    np.testing.assert_array_equal(obs, np.zeros(101))


def test_refreshed_sample_is_passed_to_inference_and_telemetry():
    _, instance, inputs, writes = walk()
    events = []
    instance.telemetry = SimpleNamespace(record=lambda kind, **fields: events.append(dict(kind=kind, **fields)))
    original = instance.imu.get_data
    calls = [0]
    def cached():
        calls[0] += 1
        data = original()
        data['gyro'] = [calls[0] / 100., 0., 0.]
        data['sample_index'] = calls[0]
        return data
    instance.imu.get_data = cached
    instance.run()
    cycles = [e for e in events if e['kind'] == 'cycle']
    assert len(writes) == 3
    assert inputs[0][0] == .02  # Second snapshot selected before inference.
    assert cycles[0]['sensors']['imu_sample_index'] == 2
    np.testing.assert_array_equal(cycles[0]['policy_obs'][:6],
        list(cycles[0]['sensors']['gyro_rad_s']) + list(cycles[0]['sensors']['accelerometer_m_s2']))


def test_reader_error_during_selection_pauses_without_inference():
    _, instance, inputs, writes = walk()
    original = instance.imu.get_data
    calls = [0]
    def cached():
        calls[0] += 1
        if calls[0] == 2:
            raise ImuDataError('reader failed during selection')
        return original()
    instance.imu.get_data = cached
    instance.run()
    assert instance.paused and 'reader failed' in instance.imu_fault
    assert not inputs and not writes and instance.imitation_i == 0


def test_selection_rejects_fresh_sample_after_wait_deadline():
    cls, instance, _, writes = walk()
    clock = [1_000_000_000]
    cls.run.__globals__['time'].monotonic_ns = lambda: clock[0]
    cls.run.__globals__['time'].sleep = lambda s: clock.__setitem__(0, clock[0] + 6_000_000)
    old = dict(gyro=[0., 0., 0.], accelero=[0., 0., 9.81], sample_index=1,
               sample_start_monotonic_ns=959_000_000, sample_end_monotonic_ns=980_000_000)
    fresh = dict(old, sample_index=2, sample_start_monotonic_ns=990_000_000,
                 sample_end_monotonic_ns=1_001_000_000)
    instance.observation_imu = old
    instance.cycle_diagnostics = {'cycle_start_ns': clock[0]}
    instance.imu.get_data = lambda: fresh if clock[0] > 1_000_000_000 else old
    with pytest.raises(ImuDataError, match='prazo de selecao'):
        instance._select_imu_for_inference(np.zeros(101))
    assert not writes and instance.imitation_i == 0
