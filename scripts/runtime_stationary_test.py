"""Preflight and launch a bounded real-runtime stationary timing test on the Pi."""
import argparse
import ast
import hashlib
from pathlib import Path
import shlex
import subprocess
import sys

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
EXPECTED_TEACHER = '3c606f9381a1710cc8fecdb7442787dcbfce3ee9bc02a6f1224774ab2b3a1067'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--onnx-model', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path.home()/'duck_config.json')
    parser.add_argument('--label', default='imu-runtime-refresh-a')
    parser.add_argument('--output', type=Path, default=ROOT/'telemetry')
    parser.add_argument('--serial-port', default='/dev/ttyACM0')
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--web-token', default='duck-test')
    parser.add_argument('--dry-run', action='store_true', help='Validate files and print command; no hardware imports')
    args = parser.parse_args(argv)
    model, config = args.onnx_model.expanduser().resolve(), args.config.expanduser().resolve()
    required = [model, config, SCRIPTS/'imu_calib_data.pkl', SCRIPTS/'polynomial_coefficients.pkl',
                SCRIPTS/'v2_rl_walk_mujoco.py', ROOT/'mini_bdx_runtime/mini_bdx_runtime/runtime_trace.py',
                ROOT/'mini_bdx_runtime/mini_bdx_runtime/imu_safety.py']
    for path in required:
        if not path.is_file():
            parser.error('Arquivo necessario nao encontrado: '+str(path))
    tree = ast.parse((SCRIPTS/'v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    version = next((ast.literal_eval(node.value) for node in tree.body if isinstance(node, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id=='IMU_SELECTION_VERSION' for t in node.targets)), None)
    if version != 3:
        parser.error('Runtime precisa da selecao IMU versao 3; extraia o pacote atualizado')
    safety = ast.parse((ROOT/'mini_bdx_runtime/mini_bdx_runtime/imu_safety.py').read_text(encoding='utf-8'))
    if not any(isinstance(node, ast.ClassDef) and node.name=='ImuSampleStaleError' for node in safety.body):
        parser.error('imu_safety.py precisa da mesma revisao v3; extraia o pacote inteiro')
    if digest(model) != EXPECTED_TEACHER:
        parser.error('ONNX diferente do teacher validado; confira o caminho do modelo')
    command = [sys.executable, str(SCRIPTS/'v2_rl_walk_mujoco.py'),
               '--onnx_model_path', str(model), '--duck_config_path', str(config),
               '--commands', '--control-source', 'web', '--serial-port', args.serial_port,
               '--web-port', str(args.web_port), '--web-token', args.web_token,
               '-c', '50', '-a', '0.25', '-p', '30', '-i', '0', '-d', '0',
               '--imu-i2c-bus', '8', '--imu-max-age-ms', '50', '--imu-frame', 'yaw-minus-90',
               '--start-paused', '--runtime-gc', 'bounded', '--active-window-s', '10',
               '--telemetry-writer-yield-ms', '1', '--telemetry-dir', str(args.output.expanduser().resolve()),
               '--telemetry-label', args.label, '--runtime-trace-seconds', '120',
               '--stationary-test', '--test-duration-s', '90']
    print('Teacher e selecao IMU v3 verificados. Config SHA-256: '+digest(config), flush=True)
    if args.dry_run:
        print('PREPARACAO SOMENTE: nenhum hardware sera inicializado.')
        print(shlex.join(command))
        return 0
    print('TESTE REAL: os servos serao ativados e a postura inicial sera enviada.', flush=True)
    print('Controle web; comandos zero; janelas de 10 s; encerramento apos 90 s de coleta.', flush=True)
    process = subprocess.Popen(command, cwd=SCRIPTS)
    try:
        return process.wait()
    except KeyboardInterrupt:
        # Terminal SIGINT also reaches the child. Give its finally/export time.
        try:
            return process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print('O runtime ainda esta encerrando; aguarde a exportacao dos dados.', file=sys.stderr)
            return 130


if __name__ == '__main__':
    raise SystemExit(main())
