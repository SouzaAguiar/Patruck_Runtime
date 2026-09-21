"""IMU transport and sample checks. Importing this module does not open hardware."""
from threading import Condition
import time

import numpy as np


class ImuDataError(RuntimeError):
    pass


def open_i2c(bus_number=None):
    if bus_number is not None:
        if bus_number < 0:
            raise ValueError('I2C bus must be nonnegative')
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus_number)  # Never fall back to another controller.
    import board
    import busio
    return busio.I2C(board.SCL, board.SDA)


class CheckedModeDevice:
    """Check the very mode byte consumed by the driver, without extra reads."""
    def __init__(self, device):
        self.device = device

    def __enter__(self):
        self.device.__enter__()
        return self

    def __exit__(self, *args):
        return self.device.__exit__(*args)

    def __getattr__(self, name):
        return getattr(self.device, name)

    def write_then_readinto(self, outgoing, incoming, **kwargs):
        request = bytes(outgoing[kwargs.get('out_start', 0):kwargs.get('out_end')])
        result = self.device.write_then_readinto(outgoing, incoming, **kwargs)
        if request == b'\x3d':
            raw = bytes(incoming[kwargs.get('in_start', 0):kwargs.get('in_end')])
            if raw != b'\x0c':
                raise ImuDataError(f'IMU: modo recebido 0x{raw.hex()}, esperado 0x0c (NDOF)')
        return result


def check_sample(sample, max_age_s, now_ns=None):
    if sample is None:
        raise ImuDataError('IMU: nenhuma amostra disponivel')
    now = time.monotonic_ns() if now_ns is None else now_ns
    start, end = (sample.get(k) for k in ('sample_start_monotonic_ns', 'sample_end_monotonic_ns'))
    if not isinstance(start, int) or not isinstance(end, int) or not 0 <= start <= end <= now:
        raise ImuDataError('IMU: timestamps invalidos')
    # Gyro is read first. Age from start conservatively covers both measurements,
    # including time blocked inside I2C, not only time since delivery.
    if (now-start)/1e9 > max_age_s:
        raise ImuDataError(f'IMU: amostra antiga ({(now-start)/1e6:.1f} ms; limite {max_age_s*1000:.1f} ms)')
    for name, limit in (('gyro', 35.0), ('accelero', 160.0)):
        try:
            vector = np.asarray(sample.get(name), dtype=float)
        except (ValueError, TypeError) as exc:
            raise ImuDataError(f'IMU: vetor {name} invalido') from exc
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ImuDataError(f'IMU: vetor {name} ausente ou nao finito')
        if (np.abs(vector) > limit).any():
            raise ImuDataError(f'IMU: {name} fora da faixa ({vector.tolist()})')
    return sample


class LatestImuSample:
    """Single latest sample; producer never waits for the policy or its pause."""
    def __init__(self, max_age_s):
        if not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('IMU maximum age must be positive and finite')
        self.max_age_s = max_age_s
        self.condition = Condition()
        self.sample = None
        self.error = 'IMU: aguardando primeira amostra'
        self.valid_run = 0

    def publish(self, sample):
        check_sample(sample, self.max_age_s)
        snapshot = dict(sample)
        snapshot['gyro'] = np.array(sample['gyro'], dtype=float, copy=True)
        snapshot['accelero'] = np.array(sample['accelero'], dtype=float, copy=True)
        with self.condition:
            self.sample = snapshot
            self.error = None
            self.valid_run += 1
            self.condition.notify_all()

    def fail(self, error):
        with self.condition:
            self.error = str(error)
            self.valid_run = 0
            self.condition.notify_all()

    def get(self):
        with self.condition:
            if self.error:
                raise ImuDataError(self.error)
            sample = check_sample(self.sample, self.max_age_s)
            return {**sample, 'gyro': sample['gyro'].copy(), 'accelero': sample['accelero'].copy()}

    def wait_ready(self, timeout_s=3):
        deadline = time.monotonic()+timeout_s
        with self.condition:
            while True:
                try:
                    sample = self.get()
                    if self.valid_run >= 3:
                        return sample
                except ImuDataError:
                    pass
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise ImuDataError('IMU: inicializacao sem 3 amostras validas recentes; '+str(self.error or 'amostra antiga'))
                self.condition.wait(min(remaining, .05))
