"""Behavioral tests with a fake clock and hardware; no physical devices."""
import builtins
import importlib.util
import json
from pathlib import Path
import pickle
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('static_pose_diagnostic', ROOT / 'scripts/diagnose_static_pose.py')
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)
from mini_bdx_runtime.imu_frame import transform_sample
from mini_bdx_runtime.telemetry import TelemetryRecorder


class Clock:
    def __init__(self):
        self.ns = 1_000_000_000

    def now(self):
        return self.ns

    def sleep(self, seconds):
        self.ns += round(seconds * 1e9)


@pytest.fixture
def args(tmp_path):
    config = tmp_path / 'duck_config.json'
    config.write_text((ROOT / 'example_config.json').read_text())
    return diag.preflight(['--config', str(config), '--support', 'feet', '--duration', '.1',
                           '--frequency', '50', '--output', str(tmp_path / 'telemetry')])


class FakeHardware:
    def __init__(self, args, clock, fault=None):
        self.args, self.clock, self.fault = args, clock, fault
        self.calls, self.reads = [], 0
        self.positions = [args.home[n] + .01 for n in args.layout]
        self.velocities = [0.] * 14
        self.h = SimpleNamespace(joints=args.layout, init_pos=args.home, joints_offsets=args.offsets,
                                 turn_on=self.turn_on, get_present_positions=self.positions_read,
                                 get_present_velocities=self.velocities_read,
                                 set_position_all=self.forbidden, turn_off=self.forbidden)
        self.i = SimpleNamespace(get_data=self.sample, wait_ready=lambda: self.calls.append('ready'),
                                 stop=lambda: self.calls.append('imu_stop'))
        self.f = SimpleNamespace(get=self.contacts, stop=lambda: self.calls.append('feet_stop'))

    def forbidden(self, *a, **kw):
        pytest.fail('Position writes or torque disable outside preparation')

    def config(self, args):
        self.calls.append('config')
        return SimpleNamespace(imu_upside_down=False)

    def imu(self, args, config):
        self.calls.append('imu_init')
        return self.i

    def hwi(self, args, config):
        self.calls.append('hwi_init')
        return self.h

    def feet(self):
        self.calls.append('feet_init')
        return self.f

    def turn_on(self):
        self.calls.append('turn_on')
        if self.fault == 'turn_on':
            raise OSError('serial during preparation')

    def sample(self):
        self.reads += 1
        self.calls.append('imu_read')
        if self.reads == 3:
            if self.fault == 'interrupt':
                raise KeyboardInterrupt
            if self.fault == 'imu':
                raise RuntimeError('I2C disconnected')
        now = self.clock.now()
        sample = transform_sample(dict(gyro=[.01, .02, .03], accelero=[-1., 2., 9.],
                                       sample_index=self.reads,
                                       sample_start_monotonic_ns=now-1_000_000,
                                       sample_end_monotonic_ns=now), self.args.imu_frame)
        if self.fault == 'stale':
            sample['sample_start_monotonic_ns'] -= 100_000_000
        elif self.fault == 'incomplete':
            sample['gyro'] = [0, 0]
        elif self.fault == 'duplicate':
            sample['sample_index'] = 1
        elif self.fault == 'double_rotation':
            sample['accelero'] = [-sample['accelero'][1], sample['accelero'][0], sample['accelero'][2]]
        elif self.fault == 'future':
            sample['sample_end_monotonic_ns'] += 10
        elif self.fault == 'identity':
            sample['sample_index'] = None
        return sample

    def positions_read(self):
        self.calls.append('positions_read')
        self.clock.sleep(.001)
        if self.fault == 'serial' and self.reads == 3:
            return None  # Actual HWI swallows transport errors and returns None.
        if self.fault == 'short_positions':
            return [0.] * 13
        if self.fault == 'position_drift':
            return np.array(self.positions) + self.reads * .03
        return np.array(self.positions)

    def velocities_read(self):
        self.calls.append('velocities_read')
        self.clock.sleep(.001)
        return np.array(self.velocities)

    def contacts(self):
        self.calls.append('contacts_read')
        self.clock.sleep(.001)
        return [True, False] if self.fault != 'contacts' else [True]


def execute(args, fault=None, mutate=None, answer='ENERGIZAR'):
    clock = Clock()
    hw = FakeHardware(args, clock, fault)
    if mutate:
        mutate(hw)
    def confirm(prompt):
        hw.calls.append('confirm')
        return answer
    code = diag.run(args, hardware=hw, confirm=confirm, now_ns=clock.now, sleep=clock.sleep)
    folder = next(args.output.iterdir())
    summary = json.loads((folder / 'summary.json').read_text())
    rows = [json.loads(line) for chunk in sorted(folder.glob('chunk-*.jsonl')) for line in chunk.read_text().splitlines()]
    return code, hw, summary, rows, folder


@pytest.mark.parametrize('option,value', [('--duration', '0'), ('--duration', 'nan'), ('--duration', '601'),
    ('--frequency', '-1'), ('--frequency', 'inf'), ('--frequency', '101'),
    ('--settle-seconds', '-1'), ('--settle-seconds', '61'), ('--imu-i2c-bus', '-1'),
    ('--imu-i2c-bus', '256'), ('--label', ''), ('--serial-port', ' '), ('--imu-frame', 'wrong')])
def test_invalid_args_before_hardware(args, option, value):
    with pytest.raises(SystemExit) as exc:
        diag.preflight(['--support', 'feet', '--config', str(args.config), option, value])
    assert exc.value.code == 2


def test_missing_support_and_invalid_combination(args):
    with pytest.raises(SystemExit):
        diag.preflight(['--config', str(args.config), '--dry-run'])
    with pytest.raises(SystemExit):
        diag.preflight(['--support', 'feet', '--duration', '.01', '--frequency', '1'])


@pytest.mark.parametrize('bad', [{'joints_offsets': {}}, {'imu_upside_down': 'false'}, [],
                                 {'joints_offsets': {'left_knee': float('nan')}}])
def test_invalid_config(args, bad):
    args.config.write_text(json.dumps(bad))
    with pytest.raises(SystemExit):
        diag.preflight(['--config', str(args.config), '--support', 'feet'])


def test_paths_calibration_and_output(args, tmp_path, monkeypatch):
    with pytest.raises(SystemExit):
        diag.preflight(['--support', 'feet', '--config', str(tmp_path / 'absent')])
    target = tmp_path / 'file'
    target.write_text('not a directory')
    with pytest.raises(SystemExit):
        diag.preflight(['--support', 'feet', '--config', str(args.config), '--output', str(target / 'child')])
    monkeypatch.setattr(diag, 'SCRIPTS', tmp_path)
    with pytest.raises(SystemExit):
        diag.preflight(['--support', 'feet', '--config', str(args.config)])
    (tmp_path / 'imu_calib_data.pkl').write_bytes(pickle.dumps({'offsets_accelerometer': [1, 2]}))
    with pytest.raises(SystemExit):
        diag.preflight(['--support', 'feet', '--config', str(args.config)])


def test_dry_run_in_clean_process_without_numpy_hardware_or_policy(args):
    # python -S cannot load site packages, including numpy and hardware drivers.
    result = subprocess.run([sys.executable, '-S', str(ROOT / 'scripts/diagnose_static_pose.py'),
                             '--support', 'stand', '--config', str(args.config),
                             '--output', str(args.output), '--dry-run'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'DRY-RUN' in result.stdout and 'ENERGIZAR' in result.stdout
    assert not args.output.exists()


@pytest.mark.parametrize('acceleration,expected', [([0, 0, 9.81], (0, 0, 0)),
    ([1, 0, 1], (45, 0, 45)), ([-1, 0, 1], (-45, 0, 45)),
    ([0, 1, 1], (0, 45, 45)), ([0, -1, 1], (0, -45, 45)), ([0, 0, -1], (0, 0, 0))])
def test_known_angles(acceleration, expected):
    assert list(diag.angles(acceleration).values()) == pytest.approx(expected)
    with pytest.raises(ValueError):
        diag.angles([0, 0, 0])


def test_complete_capture_offsets_order_timing_and_summary(args, monkeypatch):
    for i, name in enumerate(args.offsets):
        args.offsets[name] = i * .01
    # Protect against importing any inference/controller path, including transitive imports.
    original_import = builtins.__import__
    def guarded(name, *a, **kw):
        assert not any(token in name.lower() for token in ('onnx', 'rl_walk', 'web_controller', 'onnx_infer'))
        return original_import(name, *a, **kw)
    monkeypatch.setattr(builtins, '__import__', guarded)
    code, hw, summary, rows, folder = execute(args)
    assert code == 0 and summary['state'] == 'completed'
    assert summary['valid_samples'] == 5 and summary['rejected_samples'] == 0
    assert summary['effective_frequency_hz'] == pytest.approx(50)
    assert summary['posture_stable'] and summary['validity_passed']
    assert hw.calls.count('turn_on') == 1
    assert hw.calls.index('confirm') < hw.calls.index('turn_on') < hw.calls.index('imu_read')
    assert hw.calls[-2:] == ['feet_stop', 'imu_stop']
    expected_names = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
                      'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'right_hip_yaw',
                      'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle']
    assert list(args.layout) == expected_names
    assert list(args.layout.values()) == [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]
    samples = [r for r in rows if r['kind'] == 'static_pose_sample']
    for sample in samples:
        assert sample['joints']['names'] == expected_names
        assert sample['imu']['accel_yaw_minus_90_m_s2'] == [2, 1, 9]
        assert sample['imu']['accel_selected_m_s2'] == [2, 1, 9]
        assert sample['joints']['raw_estimated_position_rad'] == pytest.approx(np.array(hw.positions)+list(args.offsets.values()))
        assert sample['joints']['measured_minus_home_rad'] == pytest.approx([.01]*14)
        t = sample['timing']
        assert t['positions_start_monotonic_ns'] < t['velocities_start_monotonic_ns'] < t['feet_start_monotonic_ns']
        assert t['joints_read_ms'] == pytest.approx(2)
        assert t['total_acquisition_ms'] == pytest.approx(3)
    assert summary['contacts']['left'] == dict(readings=5, contact=5, no_contact=0)
    assert summary['contacts']['right']['contact'] == 0
    assert summary['principal_angles_from_mean_acceleration'] == pytest.approx(diag.angles([2, 1, 9]))
    assert summary['motors'][3]['configured_offset_rad'] == .03
    assert summary['motors'][3]['measured_minus_home_rad']['mean'] == pytest.approx(.01)
    assert summary['leg_groups']['right']['knee']['name'] == 'right_knee'
    assert (folder / 'writer_timing.json').is_file()
    status = json.loads((folder / 'status.json').read_text())
    assert status['state'] == 'closed' and status['dropped_records'] == 0
    metadata = json.loads((folder / 'metadata.json').read_text())['settings']
    assert metadata['initial_motion_commanded'] and metadata['initial_motion_attempted']
    assert metadata['onnx_policy_executed'] is False and metadata['operator_declared_support'] == 'feet'
    assert metadata['calibration_sha256'] == args.calibration_sha256
    assert {'static_pose_start', 'static_pose_end', 'static_pose_preparation'} <= {r['kind'] for r in rows}


@pytest.mark.parametrize('fault', ['stale', 'incomplete', 'future', 'identity', 'double_rotation',
                                  'short_positions', 'contacts', 'serial', 'imu'])
def test_reject_and_abort_invalid_or_failed_sensor(args, fault):
    code, hw, summary, rows, _ = execute(args, fault)
    assert code == 1 and summary['state'] == 'failed'
    assert summary['rejected_samples'] == 1 and not summary['validity_passed']
    bad = next(r for r in rows if r['kind'] == 'static_pose_sample' and not r['valid'])
    assert bad['rejection_reason']
    assert any(r['kind'] == 'static_pose_failure' for r in rows)
    assert hw.calls.count('turn_on') == 1 and hw.calls[-2:] == ['feet_stop', 'imu_stop']


def test_repeated_imu_is_logged_and_not_counted_as_independent_valid_sample(args):
    code, _, summary, rows, _ = execute(args, 'duplicate')
    assert code == 0 and summary['valid_samples'] == 1 and summary['rejected_samples'] == 4
    assert not summary['validity_passed']
    assert sum(r['kind'] == 'static_pose_sample' for r in rows) == 5


@pytest.mark.parametrize('fault', [None, 'position_drift'])
def test_residual_velocity_or_drift_marks_unstable_without_discarding_samples(args, fault):
    def moving(hw):
        if fault is None:
            hw.velocities[2] = .2
    code, _, summary, rows, _ = execute(args, fault, moving)
    assert code == 0 and summary['valid_samples'] == 5
    assert summary['significant_joint_motion'] and summary['posture_stable'] is False
    assert not summary['validity_passed']
    assert sum(r['kind'] == 'static_pose_sample' for r in rows) == 5


def test_interrupt_preserves_committed_data_and_warns_about_torque(args, capsys):
    code, hw, summary, rows, _ = execute(args, 'interrupt')
    assert code == 130 and summary['state'] == 'interrupted'
    assert summary['valid_samples'] == 2 and summary['failures']
    assert sum(r['kind'] == 'static_pose_sample' for r in rows) == 2
    assert hw.calls[-2:] == ['feet_stop', 'imu_stop']
    assert 'torque permanece aplicado' in capsys.readouterr().out


def test_declining_confirmation_never_energizes(args):
    code, hw, summary, _, folder = execute(args, answer='yes')
    assert code == 2 and summary['state'] == 'cancelled' and summary['valid_samples'] == 0
    assert 'turn_on' not in hw.calls and 'positions_read' not in hw.calls
    assert hw.calls[-2:] == ['feet_stop', 'imu_stop']
    assert not json.loads((folder / 'metadata.json').read_text())['settings']['initial_motion_attempted']


def test_preparation_failure_has_unknown_motion_evidence_and_never_recovers(args, capsys):
    code, hw, summary, _, folder = execute(args, 'turn_on')
    assert code == 1 and summary['total_samples'] == 0 and hw.calls.count('turn_on') == 1
    metadata = json.loads((folder / 'metadata.json').read_text())['settings']
    assert metadata['initial_motion_attempted'] and not metadata['initial_motion_commanded']
    assert 'torque permanece aplicado' in capsys.readouterr().out


def test_imu_initialization_error_is_persisted(args):
    clock = Clock()
    hardware = FakeHardware(args, clock)
    def fail():
        raise OSError('IMU readiness failed')
    hardware.i.wait_ready = fail
    assert diag.run(args, hardware, now_ns=clock.now, sleep=clock.sleep) == 1
    assert hardware.calls[-1] == 'imu_stop' and 'hwi_init' not in hardware.calls
    summary = json.loads(next(args.output.glob('*/summary.json')).read_text())
    assert summary['failures'] and not summary['validity_passed']


def test_mean_vector_angle_is_not_average_of_angles(args):
    clock = Clock()
    hardware = FakeHardware(args, clock)
    samples = []
    with TelemetryRecorder(args.output) as recorder:
        diag.acquire(hardware.i, hardware.h, hardware.f, recorder, args, samples, clock.now, clock.sleep)
    for i, sample in enumerate(samples):
        value = [9, 0, 1] if i == 0 else [0, 0, 10]
        sample['imu']['accel_yaw_minus_90_m_s2'] = value
        sample['imu']['angles'] = diag.angles(value)
    summary = diag.summarize(samples, args, 'completed', [], .1)
    expected = diag.angles([1.8, 0, 8.2])
    assert summary['principal_angles_from_mean_acceleration'] == pytest.approx(expected)
    assert summary['angles_deg']['pitch_deg']['mean'] != pytest.approx(expected['pitch_deg'])


def test_hardware_factories_reuse_runtime_calibration_and_parameters(args, monkeypatch, tmp_path):
    captured = {}
    class Imu:
        def __init__(self, **kwargs):
            captured.update(kwargs)
            captured['cwd'] = Path.cwd()
    monkeypatch.setitem(sys.modules, 'mini_bdx_runtime.raw_imu', SimpleNamespace(Imu=Imu))
    monkeypatch.chdir(tmp_path)
    assert diag.Hardware().imu(args, SimpleNamespace(imu_upside_down=True))
    assert captured['cwd'] == diag.SCRIPTS and Path.cwd() == tmp_path
    assert captured['calibrate'] is False and captured['imu_frame'] == 'yaw-minus-90'
    assert captured['i2c_bus'] == 8 and captured['upside_down'] is True
    assert captured['max_age_s'] == .05
    assert 'user_pitch_bias' not in captured


def test_real_hwi_turn_on_is_only_position_write(args, monkeypatch):
    writes, kps = [], []
    io = SimpleNamespace(set_kps=lambda *a: kps.append(a),
                         write_goal_position=lambda *a: writes.append(a),
                         read_present_position=lambda ids: [args.home[n]+args.offsets[n] for n in args.layout],
                         read_present_velocity=lambda ids: [0.]*14)
    monkeypatch.setitem(sys.modules, 'rustypot', SimpleNamespace(feetech=lambda *a: io))
    hspec = importlib.util.spec_from_file_location('hwi_static_fake_transport',
        ROOT / 'mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py')
    module = importlib.util.module_from_spec(hspec)
    hspec.loader.exec_module(module)
    monkeypatch.setattr(module, 'time', SimpleNamespace(sleep=lambda s: None))
    clock = Clock()
    hardware = FakeHardware(args, clock)
    hwi = module.HWI(SimpleNamespace(joints_offset=args.offsets))
    hardware.hwi = lambda args, config: hwi
    assert diag.run(args, hardware, confirm=lambda p: 'ENERGIZAR', now_ns=clock.now, sleep=clock.sleep) == 0
    assert len(writes) == 1 and len(kps) == 2
    assert writes[0][0] == list(args.layout.values())
    assert writes[0][1] == pytest.approx([args.home[n]+args.offsets[n] for n in args.layout])


def test_slow_serial_ages_sample_and_preserves_timings(args):
    def slow(hw):
        original = hw.positions_read
        def read():
            hw.clock.sleep(.06)
            return original()
        hw.h.get_present_positions = read
    code, _, summary, rows, _ = execute(args, mutate=slow)
    assert code == 1 and summary['rejected_samples'] == 1
    sample = next(r for r in rows if r['kind'] == 'static_pose_sample')
    assert sample['timing']['positions_read_ms'] >= 60
    assert sample['imu']['age_at_cycle_end_ms'] > 50


def test_telemetry_loss_is_failure(args, monkeypatch):
    original = TelemetryRecorder.record
    def record(self, kind, **fields):
        if kind == 'static_pose_sample':
            return False
        return original(self, kind, **fields)
    monkeypatch.setattr(TelemetryRecorder, 'record', record)
    code, _, summary, _, _ = execute(args)
    assert code == 1 and any('Telemetria' in f for f in summary['failures'])


def test_configuration_change_between_validation_and_run_aborts(args):
    args.config.write_text('{}')
    code, hw, summary, _, _ = execute(args)
    assert code == 1 and 'imu_init' not in hw.calls and summary['valid_samples'] == 0


def test_residual_motion_at_end_of_settle_is_not_erased_by_quiet_capture(args):
    def moving_initially(hw):
        original = hw.velocities_read
        calls = [0]
        def velocities():
            calls[0] += 1
            return [.1]*14 if calls[0] == 1 else original()
        hw.h.get_present_velocities = velocities
    code, _, summary, _, _ = execute(args, mutate=moving_initially)
    assert code == 0 and summary['valid_samples'] == 5
    assert not summary['posture_stable'] and summary['significant_joint_motion']
    assert summary['motors'][0]['max_abs_velocity_rad_s'] == 0
    assert summary['settle_velocity_rad_s'][0] == .1


def test_partial_gpio_constructor_failure_releases_allocated_pin(monkeypatch):
    closed = []
    class Contacts:
        def __init__(self):
            self.left_foot = SimpleNamespace(deinit=lambda: closed.append('left'))
            raise OSError('right GPIO failed')
    monkeypatch.setitem(sys.modules, 'mini_bdx_runtime.feet_contacts', SimpleNamespace(FeetContacts=Contacts))
    with pytest.raises(OSError, match='right GPIO'):
        diag.Hardware().feet()
    assert closed == ['left']


@pytest.mark.parametrize('failure', ['feet', 'imu', 'imu_thread'])
def test_cleanup_failure_still_preserves_summary_and_attempts_all_stops(args, failure):
    def mutate(hw):
        if failure == 'imu_thread':
            hw.i._thread = SimpleNamespace(is_alive=lambda: True)
        else:
            def stop():
                hw.calls.append(failure + '_stop')
                raise OSError('stop failed')
            (hw.f if failure == 'feet' else hw.i).stop = stop
    code, hw, summary, rows, _ = execute(args, mutate=mutate)
    assert code == 1 and summary['state'] == 'failed' and summary['valid_samples'] == 5
    assert summary['failures'] and not summary['validity_passed']
    assert hw.calls[-2:] == ['feet_stop', 'imu_stop']
    assert any(r['kind'] == 'static_pose_failure' for r in rows)


def test_cached_stale_exception_preserves_identity_and_age(args):
    from mini_bdx_runtime.imu_safety import ImuSampleStaleError
    def mutate(hw):
        def stale():
            raise ImuSampleStaleError(80, 50, 42)
        hw.i.get_data = stale
    code, _, summary, rows, _ = execute(args, mutate=mutate)
    assert code == 1 and summary['rejected_samples'] == 1
    sample = next(r for r in rows if r['kind'] == 'static_pose_sample')
    assert sample['imu']['rejected_sample_index'] == 42 and sample['imu']['rejected_age_ms'] == 80
