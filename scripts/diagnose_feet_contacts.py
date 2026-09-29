"""Teste guiado dos contatos GPIO22/27. Nao importa nem aciona motores."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

STAGES = [
    ('livres', [False, False], 'Deixe os DOIS sensores livres, sem pressionar.'),
    ('esquerdo', [True, False], 'Pressione SOMENTE o sensor do pe ESQUERDO do robo.'),
    ('livres_apos_esquerdo', [False, False], 'Solte os DOIS sensores.'),
    ('direito', [False, True], 'Pressione SOMENTE o sensor do pe DIREITO do robo.'),
    ('ambos', [True, True], 'Pressione os DOIS sensores.'),
    ('livres_final', [False, False], 'Solte os DOIS sensores novamente.'),
]


class ContactReader:
    def __init__(self):
        # Lazy imports: help, offline analysis and tests never initialize GPIO.
        import board
        import digitalio
        self.pins = []
        try:
            for pin in (board.D22, board.D27):
                item = digitalio.DigitalInOut(pin)
                self.pins.append(item)
                item.direction = digitalio.Direction.INPUT
                item.pull = digitalio.Pull.UP
        except BaseException:
            self.close()
            raise

    def read(self):
        raw = [bool(pin.value) for pin in self.pins]
        return raw, [not value for value in raw]

    def close(self):
        errors = []
        for pin in self.pins:
            try:
                pin.deinit()
            except Exception as exc:
                errors.append(str(exc))
        self.pins = []
        if errors:
            raise RuntimeError('Falha ao liberar GPIO: ' + '; '.join(errors))


def collect_stage(reader, duration, frequency, emit, clock=time.monotonic, sleep=time.sleep):
    start = clock()
    next_sample = start
    samples = []
    while clock() - start < duration:
        raw, contacts = reader.read()
        record = {'raw_gpio_high_left_right': raw, 'contacts_left_right': contacts,
                  'stage_elapsed_s': clock() - start}
        emit('sample', **record)
        samples.append(record)
        next_sample += 1 / frequency
        sleep(max(0, next_sample - clock()))
    return samples


def evaluate(samples, expected, duration, frequency):
    counts = Counter(tuple(s['contacts_left_right']) for s in samples)
    matching = counts.get(tuple(expected), 0)
    coverage = len(samples) / (duration * frequency)
    ratio = matching / len(samples) if samples else 0
    return {'samples': len(samples), 'expected_left_right': expected,
            'counts': {str(list(k)): v for k, v in counts.items()},
            'matching_fraction': ratio, 'nominal_sample_coverage': coverage,
            'passed': bool(coverage >= .8 and ratio >= .95),
            'criterion': '>=95% expected state, >=80% nominal samples; diagnostic triage only'}


def atomic_json(path, value):
    temp = path.with_suffix('.json.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'feet_diagnostics')
    parser.add_argument('--duration', type=float, default=3, help='Segundos por etapa (padrao 3)')
    parser.add_argument('--frequency', type=float, default=20, help='Amostras/s (padrao 20)')
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 30:
        parser.error('duration deve estar entre 1 e 30')
    if not math.isfinite(args.frequency) or not 1 <= args.frequency <= 100:
        parser.error('frequency deve estar entre 1 e 100')
    if not 1 <= args.repeats <= 10:
        parser.error('repeats deve estar entre 1 e 10')
    folder = args.output / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '-' + uuid.uuid4().hex[:8])
    folder.mkdir(parents=True)
    atomic_json(folder/'metadata.json', {
        'schema': 1, 'hardware': True, 'motors_controlled_by_script': False,
        'gpio_bcm_left_right': [22, 27], 'pull': 'UP', 'active_low': True,
        'duration_s': args.duration, 'frequency_hz': args.frequency, 'repeats': args.repeats,
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'stages': STAGES,
    })
    result = {'state': 'running', 'stages': [], 'passed': False}
    atomic_json(folder/'summary.json', result)
    print('Resultado:', folder.resolve(), flush=True)
    print('Encerre o runtime. Apoie o robo com seguranca e mantenha os servos sem alimentacao.')
    print('Este script configura apenas entradas GPIO22/27. Nao desliga motores ja ligados.')
    print('Esquerdo/direito sao os lados do ROBO, nao os de quem olha de frente.')
    reader = None
    code = 2
    with (folder/'samples.jsonl').open('w', encoding='utf-8', buffering=1) as stream:
        def emit(kind, **data):
            record = {'kind': kind, 'monotonic_ns': time.monotonic_ns(),
                      'unix_ns': time.time_ns(), **data}
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        try:
            input('Depois de preparar o robo e encerrar o runtime, pressione ENTER: ')
            reader = ContactReader()
            for repetition in range(1, args.repeats + 1):
                for name, expected, instruction in STAGES:
                    print(f'\nRepeticao {repetition}/{args.repeats} — {name}: {instruction}', flush=True)
                    input('Mantenha esta condicao e pressione ENTER. A coleta comeca em 1 segundo: ')
                    time.sleep(1)
                    emit('stage_start', name=name, repetition=repetition, expected=expected)
                    print('COLETANDO: mantenha a condicao...', flush=True)
                    samples = collect_stage(reader, args.duration, args.frequency, emit)
                    stage = {'name': name, 'repetition': repetition,
                             **evaluate(samples, expected, args.duration, args.frequency)}
                    result['stages'].append(stage)
                    emit('stage_end', **stage)
                    atomic_json(folder/'summary.json', result)
                    print('OK' if stage['passed'] else 'DIVERGENCIA: guarde o resultado para analise', stage['counts'])
            result['state'] = 'completed'
            result['passed'] = all(s['passed'] for s in result['stages'])
            code = 0 if result['passed'] else 1
        except (KeyboardInterrupt, EOFError):
            result['state'] = 'interrupted'
            code = 130
        except Exception as exc:
            result.update(state='error', error=f'{type(exc).__name__}: {exc}')
            print(result['error'])
        finally:
            if reader is not None:
                try:
                    reader.close()
                except Exception as exc:
                    result.update(state='error', error=f'GPIO cleanup: {exc}', passed=False)
                    code = 2
            emit('session_end', state=result['state'], exit_code=code)
            atomic_json(folder/'summary.json', result)
    print('Estado:', result['state'], '| Resultado:', folder.resolve())
    return code


if __name__ == '__main__':
    raise SystemExit(main())
