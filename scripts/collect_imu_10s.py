"""Coleta 10 segundos da IMU usando o diagnostico existente do Patruck.

Execute no ambiente Python do runtime, com os demais leitores da IMU parados.
Grava aceleracao, giroscopio, bytes I2C e falhas; nao aciona motores.
"""
import argparse
from pathlib import Path
import subprocess
import sys


def main(argv=None):
    scripts = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--i2c-bus', type=int, default=8)
    parser.add_argument('--config', type=Path, default=Path.home() / 'duck_config.json')
    parser.add_argument('--calibration', type=Path, default=scripts / 'imu_calib_data.pkl')
    parser.add_argument('--telemetry-dir', '--output', dest='telemetry_dir', type=Path,
                        default=scripts.parent / 'imu_diagnostics',
                        help='Diretorio onde sera criada uma nova sessao de coleta')
    parser.add_argument('--telemetry-label', '--label', dest='telemetry_label',
                        default='coleta-10s', help='Rotulo da sessao de coleta')
    args = parser.parse_args(argv)
    if args.i2c_bus < 0:
        parser.error('--i2c-bus deve ser maior ou igual a zero')
    diagnostic = scripts / 'diagnose_imu.py'
    if not diagnostic.is_file():
        parser.error('Mantenha este script na mesma pasta de diagnose_imu.py')
    command = [
        sys.executable, str(diagnostic),
        '--duration', '10', '--frequency', '50',
        '--i2c-bus', str(args.i2c_bus),
        '--config', str(args.config),
        '--calibration', str(args.calibration),
        '--output', str(args.telemetry_dir), '--label', args.telemetry_label,
    ]
    try:
        return subprocess.run(command, check=False).returncode
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
