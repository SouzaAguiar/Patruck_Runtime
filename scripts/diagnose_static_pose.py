"""Static home-pose acquisition. Only HWI.turn_on() may command motion."""
import argparse
import ast
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import statistics
import sys
import time
from uuid import uuid4

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
sys.path.insert(0, str(ROOT / 'mini_bdx_runtime'))
VERSION = 1
MAX_IMU_AGE_S = .05
MOTION_RAD_S = .05
POSITION_SPAN_RAD = .02
FRAMES = ('native', 'yaw-plus-90', 'yaw-minus-90')


def hwi_layout():
    """Read literal HWI configuration without importing/opening a serial driver."""
    tree = ast.parse((ROOT / 'mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HWI')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    values = {}
    for node in init.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == 'self':
                    if target.attr in ('joints', 'init_pos'):
                        values[target.attr] = ast.literal_eval(node.value)
    joints, home = values['joints'], values['init_pos']
    if len(joints) != 14 or list(joints) != list(home) or len(set(joints.values())) != 14:
        raise ValueError('HWI precisa definir 14 juntas e home na mesma ordem')
    return joints, home


class CalibrationUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        raise ValueError('Calibracao deve conter somente dicionario e triplas de inteiros')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def preflight(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--frequency', type=float, default=50)
    parser.add_argument('--settle-seconds', type=float, default=3)
    parser.add_argument('--imu-i2c-bus', type=int, default=8)
    parser.add_argument('--imu-frame', choices=FRAMES, default='yaw-minus-90')
    parser.add_argument('--config', type=Path, default=Path.home() / 'duck_config.json')
    parser.add_argument('--serial-port', default='/dev/ttyACM0')
    parser.add_argument('--output', type=Path, default=Path('../telemetry'))
    parser.add_argument('--label', default='static-pose-feet')
    parser.add_argument('--support', choices=('feet', 'stand'), required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        for name, low, high, inclusive in (
            ('duration', 0, 600, False), ('frequency', 0, 100, False),
            ('settle_seconds', 0, 60, True),
        ):
            value = getattr(args, name)
            if not math.isfinite(value) or value > high or (value < low if inclusive else value <= low):
                raise ValueError(f'{name}: valor fora dos limites ({low} a {high})')
        if args.duration * args.frequency < 1:
            raise ValueError('A janela deve comportar pelo menos uma amostra nominal')
        if not 0 <= args.imu_i2c_bus <= 255:
            raise ValueError('Barramento I2C deve estar entre 0 e 255')
        if not args.serial_port.strip() or '\x00' in args.serial_port:
            raise ValueError('Porta serial invalida')
        if not args.label.strip() or len(args.label) > 128 or any(ord(c) < 32 for c in args.label):
            raise ValueError('Rotulo deve conter 1 a 128 caracteres imprimiveis')
        args.config = args.config.expanduser().resolve()
        args.output = args.output.expanduser().resolve()
        # raw_imu resolves this filename against cwd; runtime is launched from scripts/.
        args.calibration = SCRIPTS / 'imu_calib_data.pkl'
        for path in (args.config, args.calibration):
            if not path.is_file():
                raise ValueError('Arquivo necessario nao encontrado: ' + str(path))
        parent = args.output
        while not parent.exists():
            parent = parent.parent
        if not parent.is_dir() or not os.access(parent, os.W_OK):
            raise ValueError('Destino precisa ser um diretorio gravavel: ' + str(args.output))
        config = json.loads(args.config.read_text(encoding='utf-8'))
        if not isinstance(config, dict) or not isinstance(config.get('imu_upside_down', False), bool):
            raise ValueError('Configuracao invalida (imu_upside_down deve ser booleano)')
        joints, home = hwi_layout()
        offsets = config.get('joints_offsets', dict.fromkeys(joints, 0.0))
        if not isinstance(offsets, dict) or set(offsets) != set(joints):
            raise ValueError('joints_offsets deve conter exatamente as 14 juntas do HWI')
        if any(type(v) not in (float, int) or not math.isfinite(v) or abs(v) > math.tau for v in offsets.values()):
            raise ValueError('Offsets devem ser finitos, em radianos, entre -2*pi e +2*pi')
        with args.calibration.open('rb') as stream:
            calibration = CalibrationUnpickler(stream).load()
        for key in ('offsets_accelerometer', 'offsets_gyroscope', 'offsets_magnetometer'):
            v = calibration.get(key) if isinstance(calibration, dict) else None
            if not isinstance(v, (tuple, list)) or len(v) != 3 or any(type(x) is not int or not -32768 <= x <= 32767 for x in v):
                raise ValueError('Calibracao invalida: ' + key)
        args.config_sha256, args.calibration_sha256 = sha256(args.config), sha256(args.calibration)
        args.layout, args.home = joints, home
        args.offsets = {name: float(offsets[name]) for name in joints}
        return args
    except (OSError, ValueError, KeyError, StopIteration, SyntaxError, EOFError, pickle.UnpicklingError) as exc:
        parser.error(str(exc))


@contextmanager
def calibration_directory():
    previous = Path.cwd()
    try:
        os.chdir(SCRIPTS)
        yield
    finally:
        os.chdir(previous)


class Hardware:
    """Lazy factories; dry-run does not import hardware or inference modules."""
    def config(self, args):
        from mini_bdx_runtime.duck_config import DuckConfig
        return DuckConfig(config_json_path=str(args.config))

    def imu(self, args, config):
        from mini_bdx_runtime.raw_imu import Imu
        with calibration_directory():
            return Imu(sampling_freq=max(50, args.frequency), calibrate=False,
                       upside_down=config.imu_upside_down, i2c_bus=args.imu_i2c_bus,
                       max_age_s=MAX_IMU_AGE_S, imu_frame=args.imu_frame)

    def hwi(self, args, config):
        from mini_bdx_runtime.rustypot_position_hwi import HWI
        return HWI(config, args.serial_port)

    def feet(self):
        from mini_bdx_runtime.feet_contacts import FeetContacts
        # The existing constructor can fail after allocating just one GPIO pin.
        reader = FeetContacts.__new__(FeetContacts)
        try:
            FeetContacts.__init__(reader)
        except BaseException:
            for name in ('left_foot', 'right_foot'):
                pin = getattr(reader, name, None)
                if pin is not None:
                    try:
                        pin.deinit()
                    except Exception:
                        pass  # Preserve the constructor's original failure.
            raise
        return reader


def vector(value, count, name):
    if value is None:
        raise ValueError(name + ': leitura ausente')
    values = list(value)
    if len(values) != count or any(v is None or isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
        raise ValueError(name + ': leitura incompleta ou nao finita')
    return [float(v) for v in values]


def angles(acceleration):
    x, y, z = acceleration
    if math.hypot(x, y, z) < 1e-6:
        raise ValueError('Aceleracao nula nao define inclinacao')
    return dict(pitch_deg=math.degrees(math.atan2(x, math.hypot(y, z))),
                roll_deg=math.degrees(math.atan2(y, math.hypot(x, z))),
                tilt_deg=math.degrees(math.atan2(math.hypot(x, y), abs(z))))


def stats(values):
    values = list(values)
    if not values:
        return dict(count=0, mean=None, std=None, min=None, max=None)
    return dict(count=len(values), mean=statistics.fmean(values), std=statistics.pstdev(values),
                min=min(values), max=max(values))


def timed_read(row, name, read, now_ns):
    timing = row['timing']
    timing[name + '_start_monotonic_ns'] = now_ns()
    try:
        return read()
    finally:
        end = now_ns()
        timing[name + '_end_monotonic_ns'] = end
        timing[name + '_read_ms'] = (end - timing[name + '_start_monotonic_ns']) / 1e6


def read_sample(imu, hwi, feet, args, index, previous_imu_index, now_ns):
    from mini_bdx_runtime.imu_frame import rotate_vector
    from mini_bdx_runtime.imu_safety import check_sample
    row = dict(index=index, valid=False, rejection_reason=None, timing={}, imu={}, joints={}, contacts=None)
    started = now_ns()
    fatal = None
    try:
        sample = timed_read(row, 'imu', imu.get_data, now_ns)
        # Retain available evidence even if validation below rejects the sample.
        row['imu'] = dict(sample)
        check_sample(sample, MAX_IMU_AGE_S, now_ns=now_ns())
        sample_index = sample.get('sample_index')
        if type(sample_index) is not int or sample_index < 0 or sample.get('imu_frame') != args.imu_frame:
            raise ValueError('IMU: identidade ou referencial invalido')
        native = vector(sample.get('accel_native_m_s2'), 3, 'aceleracao native')
        gyro_native = vector(sample.get('gyro_native_rad_s'), 3, 'gyro native')
        selected = vector(sample.get('accelero'), 3, 'aceleracao selecionada')
        gyro = vector(sample.get('gyro'), 3, 'gyro selecionado')
        # Validate rotation against native, never rotate the already rotated vector.
        expected = rotate_vector(native, args.imu_frame)
        if any(abs(a-b) > 1e-9 for a, b in zip(selected, expected)) or any(
            abs(a-b) > 1e-9 for a, b in zip(gyro, rotate_vector(gyro_native, args.imu_frame))
        ):
            raise ValueError('IMU: transformacao inconsistente')
        corrected = rotate_vector(native, 'yaw-minus-90').tolist()
        row['imu'].update(accel_native_m_s2=native, accel_yaw_minus_90_m_s2=corrected,
                          accel_selected_m_s2=selected, gyro_selected_rad_s=gyro,
                          age_at_read_ms=(now_ns()-sample['sample_start_monotonic_ns'])/1e6,
                          angles=angles(corrected))
        joints_start = now_ns()
        try:
            positions = timed_read(row, 'positions', hwi.get_present_positions, now_ns)
            row['joints']['logical_position_rad'] = positions
            positions = vector(positions, 14, 'posicoes')
            velocities = timed_read(row, 'velocities', hwi.get_present_velocities, now_ns)
            row['joints']['velocity_rad_s'] = velocities
            velocities = vector(velocities, 14, 'velocidades')
        finally:
            row['timing']['joints_start_monotonic_ns'] = joints_start
            row['timing']['joints_end_monotonic_ns'] = now_ns()
            row['timing']['joints_read_ms'] = (row['timing']['joints_end_monotonic_ns']-joints_start)/1e6
        row['joints'].update(
            names=list(args.layout), ids=list(args.layout.values()),
            logical_position_rad=positions, velocity_rad_s=velocities,
            configured_offset_rad=list(args.offsets.values()),
            raw_estimated_position_rad=[p+o for p, o in zip(positions, args.offsets.values())],
            measured_minus_home_rad=[p-h for p, h in zip(positions, args.home.values())])
        contacts = list(timed_read(row, 'feet', feet.get, now_ns))
        row['contacts'] = contacts
        if len(contacts) != 2 or any(type(c) is not bool for c in contacts):
            raise ValueError('Contatos: esperado par de booleanos esquerdo/direito')
        row['imu']['age_at_cycle_end_ms'] = (now_ns()-sample['sample_start_monotonic_ns'])/1e6
        check_sample(sample, MAX_IMU_AGE_S, now_ns=now_ns())
        if previous_imu_index is not None and sample_index <= previous_imu_index:
            row['rejection_reason'] = 'IMU: amostra repetida ou fora de ordem'
        else:
            row['valid'] = True
    except Exception as exc:
        row['rejection_reason'] = f'{type(exc).__name__}: {exc}'
        fatal = exc
        if hasattr(exc, 'age_ms'):
            row['imu'].update(rejected_age_ms=exc.age_ms, rejected_sample_index=exc.sample_index)
        if not row['imu'] and hasattr(imu, 'diagnostic_snapshot'):
            row['imu']['rejected_snapshot'] = imu.diagnostic_snapshot()
    finally:
        row['cycle_start_monotonic_ns'] = started
        row['cycle_end_monotonic_ns'] = now_ns()
        row['timing']['total_acquisition_ms'] = (row['cycle_end_monotonic_ns']-started)/1e6
    return row, fatal


def acquire(imu, hwi, feet, recorder, args, samples, now_ns, sleep):
    start = now_ns()
    deadline = start + round(args.duration * 1e9)
    period = round(1e9 / args.frequency)
    due = start
    previous = None
    recorder.record('static_pose_start', start_monotonic_ns=start, deadline_monotonic_ns=deadline)
    while now_ns() < deadline:
        row, fatal = read_sample(imu, hwi, feet, args, len(samples), previous, now_ns)
        row['scheduled_monotonic_ns'] = due
        samples.append(row)
        if not recorder.record('static_pose_sample', **row):
            raise RuntimeError('Telemetria perdeu uma amostra; coleta abortada')
        current = row['imu'].get('sample_index')
        if type(current) is int:
            previous = max(previous, current) if previous is not None else current
        if fatal:
            raise fatal
        due += period
        # Skip missed deadlines instead of sending catch-up read bursts.
        now = now_ns()
        if due < now:
            due += ((now-due)//period+1)*period
        sleep(max(0, min(due, deadline)-now)/1e9)
    return (now_ns()-start)/1e9


def summarize(samples, args, state, failures, elapsed_s, settle_velocity=None):
    valid = [s for s in samples if s['valid']]
    def axes(key):
        return {axis: stats(s['imu'][key][i] for s in valid) for i, axis in enumerate(('x', 'y', 'z'))}
    mean_accel = [statistics.fmean(s['imu']['accel_yaw_minus_90_m_s2'][i] for s in valid) for i in range(3)] if valid else None
    principal = angles(mean_accel) if mean_accel and math.hypot(*mean_accel) > 1e-6 else None
    motors = []
    # Include complete joint readings from rejected cycles in movement detection.
    joint_samples = []
    for sample in samples:
        try:
            vector(sample['joints'].get('logical_position_rad'), 14, 'positions')
            vector(sample['joints'].get('velocity_rad_s'), 14, 'velocities')
            joint_samples.append(sample)
        except (ValueError, TypeError):
            pass
    moving = bool(settle_velocity and max(abs(v) for v in settle_velocity) > MOTION_RAD_S)
    for i, (name, motor_id) in enumerate(args.layout.items()):
        positions = stats(s['joints']['logical_position_rad'][i] for s in joint_samples)
        speeds = [abs(s['joints']['velocity_rad_s'][i]) for s in joint_samples]
        peak = max(speeds) if speeds else None
        span = positions['max']-positions['min'] if speeds else None
        moving |= bool(peak is not None and (peak > MOTION_RAD_S or span > POSITION_SPAN_RAD))
        offset = args.offsets[name]
        motors.append(dict(name=name, id=motor_id, logical_position_rad=positions,
                           home_rad=args.home[name], configured_offset_rad=offset,
                           raw_estimated_position_rad=stats(s['joints']['logical_position_rad'][i]+offset for s in joint_samples),
                           measured_minus_home_rad=stats(s['joints']['logical_position_rad'][i]-args.home[name] for s in joint_samples),
                           max_abs_velocity_rad_s=peak, position_span_rad=span))
    counts = {}
    for i, side in enumerate(('left', 'right')):
        contacts = [s['contacts'][i] for s in samples if isinstance(s['contacts'], list) and len(s['contacts']) == 2 and all(type(c) is bool for c in s['contacts'])]
        counts[side] = dict(readings=len(contacts), contact=sum(contacts), no_contact=len(contacts)-sum(contacts))
    groups = {side: {function: next(m for m in motors if m['name'] == f'{side}_{function}')
                     for function in ('hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle')}
              for side in ('left', 'right')}
    stable = not moving if joint_samples else None
    return dict(version=VERSION, state=state, failures=failures, total_samples=len(samples),
                valid_samples=len(valid), rejected_samples=len(samples)-len(valid), elapsed_s=elapsed_s,
                effective_frequency_hz=len(samples)/elapsed_s if elapsed_s else 0,
                valid_frequency_hz=len(valid)/elapsed_s if elapsed_s else 0,
                nominal_coverage=len(valid)/(args.duration*args.frequency),
                principal_angles_from_mean_acceleration=principal,
                angles_deg={key: stats(s['imu']['angles'][key] for s in valid) for key in ('pitch_deg', 'roll_deg', 'tilt_deg')},
                acceleration_native_m_s2=axes('accel_native_m_s2'),
                acceleration_yaw_minus_90_m_s2=axes('accel_yaw_minus_90_m_s2'),
                gyro_native_rad_s=axes('gyro_native_rad_s'),
                gyro_selected_rad_s=axes('gyro_selected_rad_s'), motors=motors, leg_groups=groups,
                contacts=counts, posture_stable=stable, significant_joint_motion=moving,
                settle_velocity_rad_s=settle_velocity,
                validity_passed=bool(state == 'completed' and valid and len(valid) == len(samples)
                                     and len(valid)/(args.duration*args.frequency) >= .8 and stable and principal),
                reference_20261019_deg=dict(pitch=.447, roll=.106, applied_as_correction=False),
                difference_from_reference_deg={key: principal[key]-ref for key, ref in [('pitch_deg', .447), ('roll_deg', .106)]} if principal else None)


def run(args, hardware=None, confirm=None, now_ns=None, sleep=None):
    from mini_bdx_runtime.telemetry import TelemetryRecorder, atomic_json
    hardware = hardware or Hardware()
    confirm, now_ns, sleep = confirm or input, now_ns or time.monotonic_ns, sleep or time.sleep
    metadata = dict(diagnostic='static_pose', version=VERSION, session_id=uuid4().hex,
                    date_utc=datetime.now(timezone.utc).isoformat(),
                    parameters={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                    config_sha256=args.config_sha256, calibration_sha256=args.calibration_sha256,
                    angle_frame='yaw-minus-90', axis_transform='x=native_y; y=-native_x; z=native_z',
                    joint_names=list(args.layout), joint_ids=list(args.layout.values()),
                    home_rad=args.home, offsets_rad=args.offsets, onnx_policy_executed=False,
                    initial_motion_commanded=False, initial_motion_attempted=False,
                    operator_declared_support=args.support,
                    thresholds=dict(max_imu_age_ms=50, motion_rad_s=MOTION_RAD_S, position_span_rad=POSITION_SPAN_RAD),
                    raw_position_is_estimate=True, acquisition='sequential reads, independent monotonic timestamps')
    recorder = TelemetryRecorder(args.output, metadata=metadata, label=args.label)
    imu = hwi = feet = None
    samples, failures = [], []
    state, code, elapsed, settle_velocity, collection_start = 'failed', 1, 0, None, None
    attempted = False
    def save_metadata():
        path = recorder.folder / 'metadata.json'
        document = json.loads(path.read_text(encoding='utf-8'))
        document['settings'] = metadata
        atomic_json(path, document)
    try:
        config = hardware.config(args)
        if sha256(args.config) != args.config_sha256 or sha256(args.calibration) != args.calibration_sha256:
            raise RuntimeError('Configuracao ou calibracao alterada depois do preflight')
        imu = hardware.imu(args, config)
        imu.wait_ready()
        hwi = hardware.hwi(args, config)
        if list(hwi.joints.items()) != list(args.layout.items()) or hwi.init_pos != args.home or hwi.joints_offsets != args.offsets:
            raise RuntimeError('HWI diverge da configuracao validada')
        feet = hardware.feet()
        print('ATENCAO: os 14 servos serao energizados e poderao se mover ate a postura home do runtime.')
        print('Proteja o robo contra quedas. A espera nao garante imobilidade nem seguranca mecanica.')
        if confirm('Com o robo protegido, digite ENERGIZAR para confirmar: ').strip() != 'ENERGIZAR':
            state, code = 'cancelled', 2
            recorder.record('static_pose_cancelled', reason='operator_declined')
        else:
            attempted = True
            metadata['initial_motion_attempted'] = True
            save_metadata()
            recorder.record('static_pose_preparation', action='HWI.turn_on')
            hwi.turn_on()
            metadata['initial_motion_commanded'] = True
            save_metadata()
            sleep(args.settle_seconds)
            settle_velocity = vector(hwi.get_present_velocities(), 14, 'velocidade apos estabilizacao')
            recorder.record('static_pose_settled', velocity_rad_s=settle_velocity,
                            significant_motion=max(abs(v) for v in settle_velocity) > MOTION_RAD_S)
            print('Coleta iniciada; apenas leituras. Velocidade maxima apos espera:',
                  max(abs(v) for v in settle_velocity), 'rad/s', flush=True)
            collection_start = now_ns()
            elapsed = acquire(imu, hwi, feet, recorder, args, samples, now_ns, sleep)
            state, code = 'completed', 0
    except KeyboardInterrupt:
        state, code = 'interrupted', 130
        failures.append('KeyboardInterrupt: interrompido pelo operador')
        recorder.record('static_pose_failure', reason=failures[-1])
    except Exception as exc:
        failures.append(f'{type(exc).__name__}: {exc}')
        recorder.record('static_pose_failure', reason=failures[-1])
        print(failures[-1], file=sys.stderr)
    finally:
        if collection_start is not None and state != 'completed':
            elapsed = (now_ns()-collection_start)/1e9
        for name, resource in (('feet', feet), ('imu', imu)):
            if resource is not None:
                try:
                    resource.stop()
                    if name == 'imu' and getattr(resource, '_thread', None) is not None and resource._thread.is_alive():
                        raise RuntimeError('Leitor IMU ainda ativo apos stop()')
                except Exception as exc:
                    failures.append(f'cleanup {name}: {type(exc).__name__}: {exc}')
                    recorder.record('static_pose_failure', reason=failures[-1])
        # HWI has no close method. Release its serial binding without turn_off().
        hwi = None
        if failures and state == 'completed':
            state, code = 'failed', 1
        recorder.record('static_pose_end', state=state, failures=failures, elapsed_s=elapsed)
        recorder.close(state)
        if recorder.error or recorder.dropped or recorder.thread.is_alive():
            failures.append(f'Telemetria incompleta: {recorder.error}; perdidos={recorder.dropped}; writer_alive={recorder.thread.is_alive()}')
            if state != 'interrupted':
                state, code = 'failed', 1
        try:
            atomic_json(recorder.folder / 'summary.json', summarize(samples, args, state, failures, elapsed, settle_velocity))
        finally:
            if attempted:
                print('O torque permanece aplicado (sem comando de desligamento). Assegure o apoio antes de desligar os servos.')
            print('Sessao:', recorder.folder)
    return code


def main(argv=None):
    args = preflight(argv)
    if args.dry_run:
        print('DRY-RUN: arquivos/configuracao validados; nenhum hardware inicializado.')
        print('Config:', args.config, 'SHA-256:', args.config_sha256)
        print('Calibracao do runtime:', args.calibration, 'SHA-256:', args.calibration_sha256)
        print('Juntas/IDs:', list(args.layout.items()))
        print('Home (rad):', args.home, '\nOffsets (rad):', args.offsets)
        print(f'Apoio={args.support}; IMU I2C={args.imu_i2c_bus}; frame={args.imu_frame}; destino={args.output}')
        print(f'Procedimento: confirmar ENERGIZAR, HWI.turn_on(), aguardar {args.settle_seconds}s, ler {args.duration}s a {args.frequency}Hz.')
        print('Sem politica, tara ou escrita de posicao durante a coleta; torque permanece aplicado ao encerrar.')
        return 0
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
