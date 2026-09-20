import importlib.util
import json
from pathlib import Path
import struct
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'mini_bdx_runtime'))
spec = importlib.util.spec_from_file_location('diagnose_imu', ROOT/'scripts/diagnose_imu.py')
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


class FakeDevice:
    def __init__(self, acceleration=(50,-80,980), gyro=(0,0,0), fail=False):
        self.values = {0x08: acceleration, 0x14: gyro}
        self.fail = fail
        self.locked = False
        self.calls = 0

    def __enter__(self):
        assert not self.locked
        self.locked = True
        return self

    def __exit__(self, *_):
        self.locked = False

    def write_then_readinto(self, out_buffer, in_buffer, **kwargs):
        assert self.locked
        self.calls += 1
        register = out_buffer[kwargs.get('out_start',0)]
        if self.fail:
            raise OSError('simulated I2C fault')
        raw = struct.pack('<hhh', *self.values[register])
        in_buffer[kwargs.get('in_start',0):kwargs.get('in_end')] = raw


class FakeSensor:
    def __init__(self, device, wrong_scale=False):
        self.i2c_device = diagnostic.CapturedI2CDevice(device)
        self.wrong_scale = wrong_scale

    def _read(self, register, scale):
        # Same-buffer register+payload layout used by adafruit_register.Struct.
        buffer = bytearray(7)
        buffer[0] = register
        with self.i2c_device as device:
            device.write_then_readinto(buffer, buffer, out_end=1, in_start=1)
        return tuple(v*scale for v in struct.unpack('<hhh', buffer[1:]))

    @property
    def gyro(self):
        return self._read(0x14, diagnostic.GYRO_SCALE)

    @property
    def acceleration(self):
        return self._read(0x08, .02 if self.wrong_scale else .01)


def test_same_transaction_capture_no_extra_reads():
    device = FakeDevice()
    sensor = FakeSensor(device)
    row = diagnostic.read_pair(sensor, sensor.i2c_device)
    assert not row['abnormal']
    assert row['acceleration']['driver_values'] == (.5,-.8,9.8)
    assert row['acceleration']['decoded_same_bytes'] == [.5,-.8,9.8]
    assert device.calls == 2 and not device.locked
    assert row['transactions'][1]['request_hex'] == '08'
    assert row['transactions'][1]['response_hex'] == struct.pack('<hhh',50,-80,980).hex()


def test_recorded_sign_bit_pattern_is_preserved_not_repaired():
    sensor = FakeSensor(FakeDevice(acceleration=(50,-80,-31790)))
    row = diagnostic.read_pair(sensor, sensor.i2c_device)
    vector = row['acceleration']
    assert row['abnormal'] and vector['extreme_axes'] == [2]
    assert not vector['conversion_mismatch']
    assert vector['driver_values'][2] == pytest.approx(-317.90)
    assert vector['raw_int16'][2] == -31790
    assert vector['sign_bit_hypothesis_only'][0]['hypothetical_sign_bit_flip_value'] == pytest.approx(9.78)


def test_conversion_failure_distinguished_from_bus_bytes():
    sensor = FakeSensor(FakeDevice(), wrong_scale=True)
    row = diagnostic.read_pair(sensor, sensor.i2c_device)
    assert row['abnormal'] and row['acceleration']['conversion_mismatch']
    assert row['acceleration']['raw_int16'] == (50,-80,980)


def test_io_exception_keeps_failed_transaction_and_releases_lock():
    device = FakeDevice(fail=True)
    sensor = FakeSensor(device)
    row = diagnostic.read_pair(sensor, sensor.i2c_device)
    assert len(row['errors']) == 2 and row['abnormal']
    assert all(not t['success'] for t in row['transactions'])
    assert row['gyro']['raw_capture_missing']
    assert not device.locked


def test_full_acquisition_and_summary_with_immediate_reread(tmp_path):
    sensor = FakeSensor(FakeDevice(acceleration=(50,-80,-31790)))
    with diagnostic.TelemetryRecorder(tmp_path, label='synthetic-no-hardware') as recorder:
        result = diagnostic.acquire(sensor, sensor.i2c_device, recorder, duration=.06, frequency=100)
    report = diagnostic.summarize(recorder.folder)
    assert result['samples'] >= 1
    assert report['counts']['samples'] == result['samples']
    assert report['counts']['acceleration_extreme_samples'] == result['samples']
    assert report['counts']['acceleration_conversion_mismatch'] == 0
    assert report['status']['dropped_records'] == 0
    assert report['sequence_gaps'] == 0
    sample = report['first_abnormal_examples'][0]
    assert sample['reread'] is not None
    assert sample['primary']['end_monotonic_ns'] <= sample['reread']['start_monotonic_ns']
    assert report['status']['state'] == 'closed'


def test_nonfinite_or_missing_values_do_not_look_normal():
    result = diagnostic.inspect_vector((float('nan'),0,0), [], 0x14, diagnostic.GYRO_SCALE,20)
    assert not result['valid_values'] and result['raw_capture_missing']
    assert not diagnostic.inspect_vector(None, [], 0x08,.01,160)['valid_values']
