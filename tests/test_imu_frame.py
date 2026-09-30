"""Rotation geometry, reader and policy integration with hardware stubs only."""
from threading import Event
from types import SimpleNamespace
import time
import numpy as np
import pytest
from test_telemetry import ROOT, load_walk, make_walk
from mini_bdx_runtime.imu_frame import rotate_vector, transform_sample
from mini_bdx_runtime.imu_safety import check_sample, ImuDataError
from mini_bdx_runtime.raw_imu import Imu


def test_rotation_is_proper_and_preserves_norm_and_vertical():
    basis = np.column_stack([rotate_vector(v, 'yaw-plus-90') for v in np.eye(3)])
    np.testing.assert_array_equal(basis.T @ basis, np.eye(3))
    assert np.linalg.det(basis) == 1
    np.testing.assert_array_equal(rotate_vector([0, -3, 9], 'yaw-plus-90'), [3, 0, 9])
    np.testing.assert_array_equal(rotate_vector([-3, 0, 9], 'yaw-plus-90'), [0, -3, 9])


def test_minus_rotation_matches_guided_front_and_right_tilts():
    basis = np.column_stack([rotate_vector(v, 'yaw-minus-90') for v in np.eye(3)])
    np.testing.assert_array_equal(basis.T @ basis, np.eye(3))
    assert np.linalg.det(basis) == 1
    # Original horizontal frame: nose down has negative Y; right down negative X.
    np.testing.assert_array_equal(rotate_vector([0, -3, 9], 'yaw-minus-90'), [-3, 0, 9])
    np.testing.assert_array_equal(rotate_vector([-3, 0, 9], 'yaw-minus-90'), [0, 3, 9])
    # Z yaw must stay unchanged; the two candidate rotations are inverses.
    np.testing.assert_array_equal(rotate_vector([0, 0, 2], 'yaw-minus-90'), [0, 0, 2])
    np.testing.assert_array_equal(rotate_vector(rotate_vector([1, 2, 3], 'yaw-plus-90'), 'yaw-minus-90'), [1, 2, 3])


@pytest.mark.parametrize('vector', [[1, 2], [1, np.nan, 3], [np.inf, 0, 0]])
def test_invalid_data_is_rejected(vector):
    with pytest.raises(ValueError): rotate_vector(vector, 'yaw-plus-90')


def test_invalid_frame_fails_before_hardware_access():
    with pytest.raises(ValueError, match='Unknown IMU frame'):
        Imu(50, imu_frame='typo')


def test_rotation_cannot_refresh_timestamp_or_hide_staleness():
    now = time.monotonic_ns()
    source = dict(gyro=[1, 2, 3], accelero=[4, 5, 6], sample_index=7,
                  sample_start_monotonic_ns=now-100_000_000, sample_end_monotonic_ns=now-90_000_000)
    changed = transform_sample(source, 'yaw-plus-90')
    assert changed['sample_start_monotonic_ns'] == source['sample_start_monotonic_ns']
    assert changed['sample_index'] == 7 and source['gyro'] == [1, 2, 3]
    with pytest.raises(ImuDataError): check_sample(changed, .05)
    with pytest.raises(ValueError, match='already'): transform_sample(changed, 'yaw-plus-90')


def test_worker_rotates_both_vectors_once_and_keeps_originals():
    reader = Imu.__new__(Imu)
    reader.imu = SimpleNamespace(gyro=[1, 2, 3], acceleration=[4, 5, 6])
    reader.imu_frame = 'yaw-plus-90'
    reader.x_offset = 0
    reader.sampling_freq = 50
    reader._stop_event = Event()
    captured = []
    def publish(sample):
        captured.append(sample)
        reader._stop_event.set()
    reader.samples = SimpleNamespace(publish=publish, fail=lambda error: None)
    reader.bus = SimpleNamespace(deinit=lambda: None)
    reader.imu_worker()
    assert len(captured) == 1
    np.testing.assert_array_equal(captured[0]['gyro'], [-2, 1, 3])
    np.testing.assert_array_equal(captured[0]['accelero'], [-5, 4, 6])
    assert captured[0]['gyro_native_rad_s'] == (1, 2, 3)
    assert captured[0]['accel_native_m_s2'] == (4, 5, 6)


@pytest.mark.parametrize('frame,expected_gyro_x,expected_accel_x', [('yaw-plus-90', -.02, 2), ('yaw-minus-90', .02, -2)])
def test_refreshed_corrected_sample_reaches_policy_and_matching_telemetry(frame, expected_gyro_x, expected_accel_x):
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    events = []
    recorder = SimpleNamespace(record=lambda kind, **fields: events.append(dict(kind=kind, **fields)))
    walk, inputs, writes = make_walk(cls, recorder)
    original = walk.imu.get_data
    calls = [0]
    def read():
        calls[0] += 1
        sample = original()
        sample['gyro'] = [0, calls[0]/100, 0]
        sample['accelero'] = [0, -2, 9.81]
        return transform_sample(sample, frame)
    walk.imu.get_data = read
    walk.run()
    cycle = next(r for r in events if r['kind'] == 'cycle')
    assert len(writes) == 3 and inputs[0][0] == expected_gyro_x
    np.testing.assert_array_equal(inputs[0][3:6], [expected_accel_x, 0, 9.81])
    np.testing.assert_array_equal(cycle['sensors']['gyro_rad_s'], inputs[0][:3])
    assert cycle['sensors']['gyro_native_rad_s'] == (0, .02, 0)
    assert cycle['sensors']['imu_frame'] == frame


def test_diagnostic_preview_preserves_original_readings(monkeypatch):
    from test_imu_diagnostic import diagnostic
    clock = [0.0]
    monkeypatch.setattr(diagnostic, 'time', SimpleNamespace(
        monotonic=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0]+s)))
    original = {'abnormal': False, 'gyro': {'driver_values': [1, 2, 3]},
                'acceleration': {'driver_values': [4, 5, 6]}}
    monkeypatch.setattr(diagnostic, 'read_pair', lambda *args: original)
    events = []
    recorder = SimpleNamespace(error=None, dropped=0,
        record=lambda kind, **data: events.append(data))
    diagnostic.acquire(None, None, recorder, .01, 50, imu_frame='yaw-plus-90')
    assert len(events) == 1 and events[0]['primary'] is original
    assert original['gyro']['driver_values'] == [1, 2, 3]
    np.testing.assert_array_equal(events[0]['frame_preview']['gyro_rad_s'], [-2, 1, 3])
    np.testing.assert_array_equal(events[0]['frame_preview']['accelerometer_m_s2'], [-5, 4, 6])
