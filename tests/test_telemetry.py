import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from threading import Event
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mini_bdx_runtime'))
from mini_bdx_runtime.telemetry import TelemetryRecorder
from mini_bdx_runtime.imu_safety import ImuDataError, check_sample, LatestImuSample

spec = importlib.util.spec_from_file_location('summary', ROOT/'scripts/summarize_telemetry.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def rows(folder):
    return [json.loads(line) for p in sorted(folder.glob('chunk-*.jsonl'))
            for line in p.read_text(encoding='utf-8').splitlines()]


def test_flush_snapshot_and_exception_close(tmp_path):
    original = np.array([1., np.nan])
    with pytest.raises(ValueError):
        with TelemetryRecorder(tmp_path, flush_seconds=.02) as logger:
            logger.record('sample', values=original)
            original[0] = 99
            deadline = time.monotonic()+2
            while not list(logger.folder.glob('chunk-*.jsonl')) and time.monotonic() < deadline:
                time.sleep(.01)
            assert list(logger.folder.glob('chunk-*.jsonl')), 'No incremental flush before close'
            raise ValueError('simulated control failure')
    records = rows(logger.folder)
    assert records[0]['values'] == [1., None]
    assert records[-2]['kind'] == 'session_error'
    status = json.loads((logger.folder/'status.json').read_text())
    assert status['state'] == 'closed' and status['close_reason'] == 'error'
    assert status['written_records'] == len(records)
    assert status['unwritten_records'] == 0
    with TelemetryRecorder(tmp_path) as another:
        assert another.folder != logger.folder


def test_slow_storage_drops_without_waiting(tmp_path, monkeypatch):
    entered, release = Event(), Event()
    publish = TelemetryRecorder._publish
    def slow(self, records):
        entered.set()
        assert release.wait(2)
        publish(self, records)
    monkeypatch.setattr(TelemetryRecorder, '_publish', slow)
    logger = TelemetryRecorder(tmp_path, flush_seconds=.01, queue_size=2)
    try:
        logger.record('first')
        assert entered.wait(1)
        for i in range(25):
            logger.record('overflow', number=i)
        assert logger.dropped > 0
    finally:
        release.set()
        logger.close()
    status = json.loads((logger.folder/'status.json').read_text())
    assert status['state'] == 'closed_with_loss'
    assert status['attempted_records'] == status['written_records']+status['dropped_records']


def test_disk_failure_visible_without_control_exception(tmp_path, monkeypatch):
    failed = Event()
    def fail(self, records):
        failed.set()
        raise OSError('simulated disk full')
    monkeypatch.setattr(TelemetryRecorder, '_publish', fail)
    logger = TelemetryRecorder(tmp_path, flush_seconds=.01)
    logger.record('sample')
    assert failed.wait(1)
    logger.thread.join(1)
    assert not logger.record('after_failure')
    logger.close()
    status = json.loads((logger.folder/'status.json').read_text())
    assert status['state'] == 'failed'
    assert 'disk full' in status['writer_error']
    assert status['unwritten_records'] > 0


def load_walk(source):
    # Execute only the class definition. No hardware module is imported, and
    # neither RLWalk.__init__ nor HWI.turn_on can run in these tests.
    tree = ast.parse(source)
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RLWalk')
    namespace = {'np': np, 'HOME_DIR': '', 'time': SimpleNamespace(
        time=time.time, monotonic_ns=time.monotonic_ns, sleep=lambda seconds: None),
        'make_action_dict': lambda targets, names: dict(zip(names, targets)),
        'ImuDataError': ImuDataError, 'check_sample': check_sample}
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<walk-class-only>', 'exec'), namespace)
    return namespace['RLWalk']


def make_walk(cls, telemetry=None, fail_first_read=False):
    walk = cls.__new__(cls)
    walk.telemetry = telemetry
    walk.telemetry_observation = None
    walk.imu_fault = None
    walk.imu_fault_acknowledged = False
    walk.imu_max_age_s = .05
    walk.observation_imu = None
    walk.start = lambda: None  # Hardware start is covered separately with stubs.
    walk.num_dofs = 14
    walk.commands = False
    walk.controller = None
    walk.paused = False
    walk.control_freq = 50
    walk.duck_config = SimpleNamespace(antennas=False, eyes=False, projector=False)
    walk.save_obs = False
    walk.replay_obs = None
    walk.action_scale = .25
    walk.action_filter = None
    walk.init_pos = list(np.arange(14)*.01)
    walk.motor_targets = np.array(walk.init_pos)
    walk.prev_motor_targets = walk.motor_targets.copy()
    walk.last_commands = [.05, 0., .2, .01, .02, .03, .04]
    walk.last_action = np.zeros(14)
    walk.last_last_action = np.zeros(14)
    walk.last_last_last_action = np.zeros(14)
    walk.imitation_i = 0
    walk.imitation_phase = np.array([0., 0.])
    walk.PRM = SimpleNamespace(nb_steps_in_period=27)
    walk.phase_frequency_factor = 1.
    walk.phase_frequency_factor_offset = .05
    walk.feet_contacts = SimpleNamespace(get=lambda: [True, False], stop=lambda: None)
    def sample():
        stamp = time.monotonic_ns()
        return {
        'gyro': np.array([.1,.2,.3]), 'accelero': np.array([1.,0.,9.81]),
        'sample_start_monotonic_ns': stamp-1000000,
        'sample_end_monotonic_ns': stamp, 'sample_index': 4}
    walk.imu = SimpleNamespace(get_data=sample, wait_ready=lambda **kwargs: sample(), stop=lambda: None)
    writes, inputs = [], []
    def infer(obs):
        if len(inputs) == 3:
            raise KeyboardInterrupt
        inputs.append(obs.copy())
        return np.linspace(-.3,.3,14)+len(inputs)*.01
    walk.policy = SimpleNamespace(infer=infer)
    reads = [0]
    def read_positions(ignore):
        reads[0] += 1
        if fail_first_read and reads[0] == 1:
            return None
        return np.array(walk.init_pos)+reads[0]*.001
    names = [f'joint_{i}' for i in range(14)]
    walk.hwi = SimpleNamespace(
        joints=dict(zip(names, range(14))), joints_offsets=dict.fromkeys(names, .05),
        get_present_positions=read_positions,
        get_present_velocities=lambda ignore: np.full(14,.1),
        set_position_all=lambda targets: writes.append(dict(targets)))
    return walk, inputs, writes


def test_logged_control_matches_original_runtime_and_summary(tmp_path):
    original = subprocess.check_output(
        ['git', '-C', str(ROOT), 'show', 'HEAD:scripts/v2_rl_walk_mujoco.py'], text=True)
    baseline, old_inputs, old_writes = make_walk(load_walk(original))
    baseline.run()
    source = (ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8')
    with TelemetryRecorder(tmp_path, metadata={'control_freq_hz': 50}) as logger:
        walk, inputs, writes = make_walk(load_walk(source), logger)
        logger.record('runtime_ready', joint_names=list(walk.hwi.joints))
        walk.run()
    np.testing.assert_array_equal(inputs, old_inputs)
    assert writes == old_writes
    cycles = [r for r in rows(logger.folder) if r['kind'] == 'cycle']
    assert len(cycles) == 3
    for cycle, obs, targets in zip(cycles, inputs, writes):
        np.testing.assert_array_equal(cycle['policy_obs'], obs)
        np.testing.assert_array_equal(cycle['observed_obs'], obs)
        np.testing.assert_array_equal(cycle['motor_targets_rad'], list(targets.values()))
        np.testing.assert_array_equal(cycle['servo_goal_rad'], np.array(list(targets.values()))+.05)
        np.testing.assert_array_equal(cycle['sensors']['previous_motor_targets_rad'], obs[83:97])
        assert cycle['inference_ms'] >= 0 and cycle['motor_write_ms'] >= 0
        assert cycle['sensors']['imu_sample_index'] == 4
        assert cycle['sensors']['imu_age_ms'] >= 0
    result = summary.summarize(logger.folder)
    assert result['records_by_kind']['cycle'] == 3
    assert result['sequence_gaps_in_committed_chunks'] == 0
    assert len(result['joint_previous_target_rmse_rad']) == 14
    # An interrupted temporary chunk is not read as committed data.
    (logger.folder/'chunk-999999.jsonl.tmp').write_text('{incomplete', encoding='utf-8')
    assert summary.summarize(logger.folder)['temporary_chunks_ignored'] == 1


def test_sensor_failure_is_recorded(tmp_path):
    source = (ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8')
    with TelemetryRecorder(tmp_path) as logger:
        walk, _, _ = make_walk(load_walk(source), logger, fail_first_read=True)
        walk.run()
    records = rows(logger.folder)
    assert any(r['kind'] == 'sensor_failure' for r in records)
    assert sum(r['kind'] == 'cycle' for r in records) == 3


def test_imu_adds_timestamp_without_changing_readings():
    from mini_bdx_runtime.raw_imu import Imu  # Lazy imports: safe on a PC.
    imu = Imu.__new__(Imu)
    imu.imu = SimpleNamespace(gyro=[1.,2.,3.], acceleration=[4.,5.,6.])
    imu.x_offset = .5
    imu.sampling_freq = 50
    samples = []
    imu._stop_event = Event()
    def publish(sample):
        samples.append(sample)
        imu._stop_event.set()
    imu.samples = SimpleNamespace(publish=publish, fail=lambda error: None)
    imu.bus = SimpleNamespace(deinit=lambda: None)
    imu.imu_worker()
    np.testing.assert_array_equal(samples[0]['gyro'], [1.,2.,3.])
    np.testing.assert_array_equal(samples[0]['accelero'], [3.5,5.,6.])
    assert samples[0]['sample_start_monotonic_ns'] <= samples[0]['sample_end_monotonic_ns']
    assert samples[0]['sample_index'] == 0


def test_real_web_state_pause_timeout_and_disconnect(tmp_path):
    import dataclasses
    import threading
    from mini_bdx_runtime.buttons import Buttons
    source = (ROOT/'mini_bdx_runtime/mini_bdx_runtime/web_controller.py').read_text()
    tree = ast.parse(source)
    selected = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
    future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
    namespace = {'np': np, 'time': time, 'threading': threading, 'dataclass': dataclasses.dataclass,
                 'field': dataclasses.field, 'X_RANGE': [-.15,.15], 'Y_RANGE': [-.2,.2], 'YAW_RANGE': [-1,1]}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future]+selected, type_ignores=[])),
                 '<web-classes-only>', 'exec'), namespace)
    controller = namespace['WebController'].__new__(namespace['WebController'])
    controller.state = namespace['WebCommandState']()
    controller.buttons = Buttons()
    get_command = controller.get_last_command
    count = [0]
    def commands():
        count[0] += 1
        if count[0] == 1:
            controller.state.update({'left_y': 1., 'paused': True})
        elif count[0] == 2:
            controller.state.update({'left_y': 1., 'paused': False})
        elif count[0] == 3:
            controller.state._last_update -= 1.
        elif count[0] == 4:
            controller.state.disconnect()
        else:
            raise KeyboardInterrupt
        return get_command()
    controller.get_last_command = commands
    walk_source = (ROOT/'scripts/v2_rl_walk_mujoco.py').read_text()
    with TelemetryRecorder(tmp_path) as logger:
        walk, inputs, _ = make_walk(load_walk(walk_source), logger)
        walk.commands = True
        walk.controller = controller
        walk.run()
    records = rows(logger.folder)
    cycles = [r for r in records if r['kind'] == 'cycle']
    assert [r['paused'] for r in records if r['kind'] == 'pause_changed'] == [True, False]
    assert sum(r['kind'] == 'paused' for r in records) == 1
    assert len(cycles) == 3
    assert cycles[0]['commands'][0] == .15
    assert cycles[0]['command_source']['command_fresh'] is True
    assert cycles[1]['commands'] == [0.]*7
    assert cycles[1]['command_source']['command_fresh'] is False
    assert cycles[1]['command_source']['connected'] is True
    assert cycles[2]['commands'] == [0.]*7
    assert cycles[2]['command_source']['connected'] is False
    np.testing.assert_array_equal(inputs[0][6:13], cycles[0]['commands'])
    assert summary.summarize(logger.folder)['web_command_cycles'] == {
        'fresh': 1, 'timed_out': 1, 'disconnected': 1}
