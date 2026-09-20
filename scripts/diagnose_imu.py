"""BNO055 investigation: capture exact driver I2C transactions, never drive motors.

Hardware imports are lazy; --help and --summarize work on a regular computer.
Stop other IMU consumers before acquisition: the Adafruit constructor resets
and configures the sensor. No readings are filtered or repaired.
"""
import argparse
from collections import Counter
from importlib import metadata
import json
import math
from pathlib import Path
import pickle
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'mini_bdx_runtime'))
from mini_bdx_runtime.telemetry import TelemetryRecorder, atomic_json, file_sha256

GYRO_SCALE = 0.001090830782496456
REGISTERS = {'chip_id': 0x00, 'page': 0x07, 'calibration_status': 0x35,
             'system_status': 0x39, 'system_error': 0x3A, 'unit_selection': 0x3B,
             'operation_mode': 0x3D, 'power_mode': 0x3E, 'axis_map': 0x41, 'axis_sign': 0x42}


class CapturedI2CDevice:
    """Wrap the driver's existing I2CDevice, preserving locks and transactions."""
    def __init__(self, device):
        self.device = device
        self.transactions = []

    def __enter__(self):
        self.device.__enter__()
        return self

    def __exit__(self, *args):
        return self.device.__exit__(*args)

    def __getattr__(self, name):
        return getattr(self.device, name)

    def write_then_readinto(self, out_buffer, in_buffer, **kwargs):
        outgoing = bytes(out_buffer[kwargs.get('out_start', 0):kwargs.get('out_end')])
        event = {'register': outgoing[0] if len(outgoing) == 1 else None,
                 'request_hex': outgoing.hex(), 'start_monotonic_ns': time.monotonic_ns()}
        try:
            result = self.device.write_then_readinto(out_buffer, in_buffer, **kwargs)
            received = bytes(in_buffer[kwargs.get('in_start', 0):kwargs.get('in_end')])
            event.update(response_hex=received.hex(), response_bytes=list(received), success=True)
            return result
        except Exception as exc:
            event.update(success=False, error=f'{type(exc).__name__}: {exc}')
            raise
        finally:
            event['end_monotonic_ns'] = time.monotonic_ns()
            self.transactions.append(event)


def inspect_vector(values, transactions, register, scale, threshold):
    reads = [r for r in transactions if r['register'] == register and r['success']
             and len(r['response_bytes']) == 6]
    raw = struct.unpack('<hhh', bytes(reads[-1]['response_bytes'])) if reads else None
    decoded = [v*scale for v in raw] if raw is not None else None
    valid = isinstance(values, (list, tuple)) and len(values) == 3 and all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in values)
    mismatch = valid and decoded is not None and any(abs(a-b)>1e-9 for a,b in zip(values, decoded))
    extremes = [i for i,v in enumerate(values) if abs(v)>threshold] if valid else []
    hypotheses = []
    if raw is not None:
        for i in extremes:
            toggled = (raw[i] & 0xffff) ^ 0x8000
            toggled = toggled-65536 if toggled >= 32768 else toggled
            hypotheses.append({'axis': 'xyz'[i], 'recorded_int16': raw[i],
                               'hypothetical_sign_bit_flip_value': toggled*scale})
    return {'driver_values': values, 'valid_values': valid, 'raw_int16': raw,
            'decoded_same_bytes': decoded, 'conversion_mismatch': mismatch,
            'raw_capture_missing': raw is None, 'extreme_axes': extremes,
            'sign_bit_hypothesis_only': hypotheses}


def read_pair(sensor, capture):
    capture.transactions.clear()
    started = time.monotonic_ns()
    values, errors = {}, []
    # Match the ordering used by raw_imu.Imu. No second independent read is
    # used to obtain the bytes: they are intercepted in these property calls.
    for name in ('gyro', 'acceleration'):
        try:
            values[name] = getattr(sensor, name)
        except Exception as exc:
            values[name] = None
            errors.append(f'{name}: {type(exc).__name__}: {exc}')
    transactions = list(capture.transactions)
    gyro = inspect_vector(values['gyro'], transactions, 0x14, GYRO_SCALE, 20)
    accel = inspect_vector(values['acceleration'], transactions, 0x08, .01, 160)
    abnormal = bool(errors) or any(
        not v['valid_values'] or v['raw_capture_missing'] or v['conversion_mismatch'] or v['extreme_axes']
        for v in (gyro, accel))
    return {'start_monotonic_ns': started, 'end_monotonic_ns': time.monotonic_ns(),
            'gyro': gyro, 'acceleration': accel, 'transactions': transactions,
            'errors': errors, 'abnormal': bool(abnormal)}


def invalid_mode_reads(transactions):
    """Audit bytes already captured; preserve the original reread trigger."""
    return [t for t in transactions if t.get('register') == 0x3D and t.get('success')
            and (len(t.get('response_bytes', [])) != 1 or t['response_bytes'][0] > 0x0C)]


def open_i2c(bus_number=None):
    """Explicit Linux bus selection has no fallback to the hardware bus."""
    if bus_number is not None:
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus_number)
    import board
    import busio
    return busio.I2C(board.SCL, board.SDA)


def register_snapshot(sensor):
    result = {}
    for name, register in REGISTERS.items():
        try:
            result[name] = sensor._read_register(register)
        except Exception as exc:
            result[name] = {'error': f'{type(exc).__name__}: {exc}'}
    return result


def startup_issues(registers, mode):
    issues = []
    for name, expected in (('chip_id', 0xA0), ('page', 0), ('operation_mode', mode)):
        actual = registers.get(name)
        if actual != expected:
            value = f'0x{actual:02X}' if isinstance(actual, int) else str(actual)
            issues.append(f'{name}={value} (esperado 0x{expected:02X})')
    units = registers.get('unit_selection')
    if not isinstance(units, int) or units & 0x03:
        issues.append(f'unit_selection={units!r} incompativel com a conversao do driver')
    return issues


def probe_startup(sensor, capture, recorder, mode, attempts=10, required_good=3):
    """Retry reads only, retaining every failure. Never force page/unit changes."""
    consecutive = 0
    failures = 0
    last_issues = []
    for attempt in range(1, attempts+1):
        capture.transactions.clear()
        registers = register_snapshot(sensor)
        last_issues = startup_issues(registers, mode)
        consecutive = consecutive+1 if not last_issues else 0
        failures += int(bool(last_issues))
        recorder.record('imu_registers', stage='startup_probe', attempt=attempt,
                        registers=registers, issues=last_issues,
                        transactions=list(capture.transactions), consecutive_valid=consecutive)
        if last_issues:
            print(f'Conferencia IMU {attempt}/{attempts}: '+ '; '.join(last_issues), flush=True)
        else:
            print(f'Conferencia IMU {attempt}/{attempts}: ID=0xA0, pagina=0, '
                  f'modo=0x{mode:02X}; valida {consecutive}/{required_good}', flush=True)
        if consecutive >= required_good:
            recorder.record('startup_probe_result', passed=True, attempts=attempt,
                            invalid_snapshots=failures)
            return registers
        if attempt < attempts:
            time.sleep(.1)
    recorder.record('startup_probe_result', passed=False, attempts=attempts,
                    invalid_snapshots=failures)
    detail = '; '.join(last_issues) or 'leituras validas consecutivas insuficientes'
    raise RuntimeError('IMU nao confirmou configuracao estavel: '+detail+
                       '. Bytes e tentativas preservados na pasta da sessao.')


def acquire(sensor, capture, recorder, duration, frequency, reread=True):
    started = time.monotonic()
    count = anomalies = overruns = 0
    last_report = started
    while time.monotonic()-started < duration:
        tick = time.monotonic()
        primary = read_pair(sensor, capture)
        second = read_pair(sensor, capture) if primary['abnormal'] and reread else None
        count += 1
        anomalies += int(primary['abnormal'])
        elapsed = time.monotonic()-tick
        overruns += int(elapsed > 1/frequency)
        recorder.record('imu_sample', index=count-1, elapsed_s=tick-started,
                        primary=primary, reread=second, acquisition_ms=elapsed*1000)
        if recorder.error is not None or recorder.dropped:
            raise RuntimeError('Diagnostic recording failed or dropped samples; inspect status.json')
        if time.monotonic()-last_report >= 10:
            print(f'{count} amostras; {anomalies} anomalias; {overruns} ciclos acima do periodo', flush=True)
            last_report = time.monotonic()
        time.sleep(max(0, 1/frequency-(time.monotonic()-tick)))
    return {'samples': count, 'abnormal_samples': anomalies, 'acquisition_overruns': overruns}


def summarize(folder):
    folder = Path(folder)
    counts = Counter()
    examples = []
    max_age = 0.
    first = last = None
    seq_previous = -1
    sequence_gaps = 0
    max_abs = {'gyro': 0., 'acceleration': 0.}
    registers = []
    startup_results = []
    errors = []
    for path in sorted(folder.glob('chunk-*.jsonl')):
        for line in path.read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            sequence_gaps += max(0, row['sequence']-seq_previous-1)
            seq_previous = row['sequence']
            if row['kind'] == 'imu_registers':
                registers.append(row['registers'])
            if row['kind'] == 'startup_probe_result':
                startup_results.append({k:v for k,v in row.items() if k not in (
                    'kind','sequence','recorded_monotonic_ns','recorded_unix_ns')})
            if row['kind'] == 'session_error':
                errors.append({'type': row.get('error_type'), 'message': row.get('message')})
            if row['kind'] != 'imu_sample':
                continue
            counts['samples'] += 1
            first = row['elapsed_s'] if first is None else first
            last = row['elapsed_s']
            p = row['primary']
            counts['abnormal_samples'] += int(p['abnormal'])
            bad_mode = bool(invalid_mode_reads(p.get('transactions', [])))
            counts['samples_with_invalid_mode_byte'] += int(bad_mode)
            counts['samples_with_any_detected_anomaly'] += int(p['abnormal'] or bad_mode)
            counts['samples_with_io_error'] += int(bool(p['errors']))
            max_age = max(max_age, row['acquisition_ms'])
            for name in max_abs:
                vector = p[name]
                counts[name+'_extreme_samples'] += int(bool(vector['extreme_axes']))
                counts[name+'_conversion_mismatch'] += int(vector['conversion_mismatch'])
                counts[name+'_missing_raw_capture'] += int(vector['raw_capture_missing'])
                counts[name+'_invalid_values'] += int(not vector['valid_values'])
                if vector['valid_values']:
                    max_abs[name] = max(max_abs[name], *[abs(v) for v in vector['driver_values']])
            if p['abnormal']:
                if row['reread'] is not None:
                    counts['abnormal_followed_by_normal_reread'] += int(not row['reread']['abnormal'])
                if len(examples) < 20:
                    examples.append(row)
    metadata_json = json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
    status = json.loads((folder/'status.json').read_text(encoding='utf-8'))
    return {'session': str(folder.resolve()), 'label': metadata_json['label'], 'status': status,
            'counts': dict(counts), 'sequence_gaps': sequence_gaps,
            'observed_span_s': last-first if first is not None else 0,
            'effective_frequency_hz': (counts['samples']-1)/(last-first)
            if first is not None and last > first else None,
            'max_abs_readings': max_abs, 'max_acquisition_ms': max_age,
            'register_snapshots': registers, 'first_abnormal_examples': examples,
            'startup_probe_results': startup_results, 'session_errors': errors,
            'interpretation': [
                'Raw capture is the same transaction consumed by the driver, not a later read.',
                'A reread is a new acquisition, not proof of the original true physical value.',
                'Normal readings in this short test do not exclude intermittent failures.',
                'Gyro >20 rad/s is a diagnostic threshold, not the sensor full-scale limit.',
                'A sign-bit flip is a hypothesis only; original readings are never repaired.',
                'Invalid mode bytes are counted separately without adding rereads, to preserve timing comparability.',
            ]}


def package_versions():
    versions = {}
    for name in ('adafruit-circuitpython-bno055', 'adafruit-circuitpython-busdevice',
                 'adafruit-circuitpython-register', 'Adafruit-Blinka', 'adafruit-extended-bus'):
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def host_i2c_info():
    """Read effective Linux device-tree hints; never edit boot configuration."""
    result = {'raspberry_model': None, 'device_tree_i2c_clock_hz': {}, 'adapters': {}}
    model = Path('/proc/device-tree/model')
    if model.exists():
        try:
            result['raspberry_model'] = model.read_bytes().rstrip(b'\0').decode(errors='replace')
        except OSError:
            pass
    tree = Path('/proc/device-tree')
    if tree.exists():
        for path in tree.rglob('clock-frequency'):
            if 'i2c' not in str(path).lower():
                continue
            try:
                data = path.read_bytes()
                if len(data) == 4:
                    result['device_tree_i2c_clock_hz'][str(path)] = int.from_bytes(data, 'big')
            except OSError:
                pass
    # Linux commonly exposes these under /sys/bus, not /sys/class/i2c-adapter.
    for path in Path('/sys/bus/i2c/devices').glob('i2c-*/name'):
        try:
            result['adapters'][path.parent.name] = path.read_text().strip()
        except OSError:
            pass
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summarize', type=Path, help='Read a recorded folder; no hardware access')
    parser.add_argument('--output', type=Path, default=Path('../imu_diagnostics'))
    parser.add_argument('--label', default='stationary-motors-off')
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--frequency', type=float, default=50)
    parser.add_argument('--address', type=lambda s: int(s, 0), default=0x29)
    parser.add_argument('--i2c-bus', type=int,
                        help='Explicit Linux /dev/i2c-N via adafruit-extended-bus; no fallback')
    parser.add_argument('--config', type=Path, default=Path.home()/'duck_config.json')
    parser.add_argument('--calibration', type=Path, default=Path(__file__).with_name('imu_calib_data.pkl'))
    parser.add_argument('--no-calibration', action='store_true')
    parser.add_argument('--mode', choices=['ndof', 'imuplus', 'accgyro'], default='ndof')
    parser.add_argument('--no-reread', action='store_true')
    args = parser.parse_args()
    if args.summarize:
        print(json.dumps(summarize(args.summarize), indent=2, ensure_ascii=False))
        return 0
    if not (math.isfinite(args.duration) and 0 < args.duration <= 3600 and
            math.isfinite(args.frequency) and 0 < args.frequency <= 100):
        parser.error('duration must be in (0,3600]; frequency in (0,100]')
    if not 0x08 <= args.address <= 0x77:
        parser.error('invalid I2C address')
    if args.i2c_bus is not None and args.i2c_bus < 0:
        parser.error('i2c-bus must be nonnegative')
    # Refuse an implicit change of mounting/calibration from the runtime setup.
    config = json.loads(args.config.read_text(encoding='utf-8'))
    calibration = None
    if not args.no_calibration:
        # Same trusted local calibration file already used by the runtime.
        with args.calibration.open('rb') as stream:
            calibration = pickle.load(stream)
        for name in ('offsets_accelerometer','offsets_gyroscope','offsets_magnetometer'):
            values = calibration[name]
            if len(values) != 3 or not all(isinstance(v, int) and -32768 <= v <= 32767 for v in values):
                raise ValueError('Invalid local calibration: '+name)
    print('Diagnostico isolado: encerre o runtime e outros leitores da IMU. Nenhum motor sera acionado.')
    # Do not import raw_imu or the walking runtime: both initialize hardware.
    import adafruit_bno055
    bus = None
    exit_code = 0
    recorder = TelemetryRecorder(args.output, label=args.label, metadata={
        'diagnostic_schema': 2, 'duration_s': args.duration, 'frequency_hz': args.frequency,
        'i2c_bus': args.i2c_bus,
        'i2c_backend': 'extended_bus' if args.i2c_bus is not None else 'board_default',
        'address': args.address, 'mode': args.mode, 'imu_upside_down': config.get('imu_upside_down', False),
        'config_sha256': file_sha256(args.config),
        'calibration_sha256': file_sha256(args.calibration) if calibration else None,
        'driver_sha256': file_sha256(adafruit_bno055.__file__), 'packages': package_versions(),
        'script_sha256': file_sha256(__file__), 'reread_on_anomaly': not args.no_reread,
        'motors_controlled_by_script': False,
        'host_i2c': host_i2c_info(),
    })
    try:
        with recorder:
            bus = open_i2c(args.i2c_bus)
            sensor = adafruit_bno055.BNO055_I2C(bus, address=args.address)
            mode = {'ndof': adafruit_bno055.NDOF_MODE, 'imuplus': adafruit_bno055.IMUPLUS_MODE,
                    'accgyro': adafruit_bno055.ACCGYRO_MODE}[args.mode]
            sensor.mode = mode
            negative, positive = adafruit_bno055.AXIS_REMAP_NEGATIVE, adafruit_bno055.AXIS_REMAP_POSITIVE
            sensor.axis_remap = (adafruit_bno055.AXIS_REMAP_Y, adafruit_bno055.AXIS_REMAP_X,
                                 adafruit_bno055.AXIS_REMAP_Z, negative,
                                 negative if config.get('imu_upside_down', False) else positive,
                                 negative if config.get('imu_upside_down', False) else positive)
            if calibration:
                sensor.mode = adafruit_bno055.CONFIG_MODE
                time.sleep(.1)
                for name in ('offsets_accelerometer','offsets_gyroscope','offsets_magnetometer'):
                    setattr(sensor, name, tuple(calibration[name]))
                sensor.mode = mode
            time.sleep(.1)
            capture = CapturedI2CDevice(sensor.i2c_device)
            sensor.i2c_device = capture
            probe_startup(sensor, capture, recorder, mode)
            result = acquire(sensor, capture, recorder, args.duration, args.frequency, not args.no_reread)
            recorder.record('acquisition_complete', **result)
            recorder.record('imu_registers', stage='final', registers=register_snapshot(sensor))
    except KeyboardInterrupt:
        print('Coleta interrompida; blocos pendentes finalizados.')
        exit_code = 130
    except Exception as exc:
        print(f'DIAGNOSTICO INTERROMPIDO: {type(exc).__name__}: {exc}', file=sys.stderr)
        exit_code = 2
    finally:
        if bus is not None:
            try:
                bus.deinit()
            except Exception as exc:
                print(f'Nao foi possivel liberar o barramento: {exc}', file=sys.stderr)
                exit_code = 2
    result = summarize(recorder.folder)
    atomic_json(recorder.folder/'summary.json', result)
    print(json.dumps(result['counts'], indent=2))
    print('Resultado:', (recorder.folder/'summary.json').resolve())
    return exit_code if result['status']['state'] == 'closed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
