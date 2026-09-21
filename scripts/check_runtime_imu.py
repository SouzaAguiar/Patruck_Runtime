"""Exercise the runtime IMU reader only. Never imports motor or walking code."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'mini_bdx_runtime'))
from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.imu_safety import ImuDataError
from mini_bdx_runtime.telemetry import TelemetryRecorder, file_sha256


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--i2c-bus', type=int, default=8)
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--max-age-ms', type=float, default=50)
    parser.add_argument('--config', type=Path, default=Path.home()/'duck_config.json')
    parser.add_argument('--output', type=Path, default=ROOT/'imu_runtime_checks')
    args = parser.parse_args()
    if args.i2c_bus < 0 or not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error('Use a nonnegative bus and duration in (0, 3600]')
    if not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0:
        parser.error('max-age-ms must be positive and finite')
    # raw_imu intentionally uses the same calibration path as walking.
    if not Path('imu_calib_data.pkl').is_file():
        parser.error('Run from the runtime scripts directory containing imu_calib_data.pkl')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    imu = None
    errors = reads = 0
    seen = set()
    recorder = TelemetryRecorder(args.output, label='runtime-imu-only', metadata={
        'i2c_bus': args.i2c_bus, 'max_age_ms': args.max_age_ms,
        'motors_controlled_by_script': False,
        'config_sha256': file_sha256(args.config),
        'calibration_sha256': file_sha256('imu_calib_data.pkl'),
        'source_sha256': {name: file_sha256(ROOT/'mini_bdx_runtime'/'mini_bdx_runtime'/name)
                          for name in ('raw_imu.py', 'imu_safety.py')},
    })
    try:
        with recorder:
            try:
                imu = Imu(50, upside_down=config.get('imu_upside_down', False),
                          i2c_bus=args.i2c_bus, max_age_s=args.max_age_ms/1000)
                imu.wait_ready()
                deadline = time.monotonic()+args.duration
                while time.monotonic() < deadline:
                    tick = time.monotonic()
                    try:
                        sample = imu.get_data()
                        reads += 1
                        seen.add(sample['sample_index'])
                        recorder.record('imu_runtime_sample', **sample)
                    except ImuDataError as exc:
                        errors += 1
                        recorder.record('imu_runtime_fault', message=str(exc))
                        print(str(exc), flush=True)
                    if recorder.error or recorder.dropped:
                        raise RuntimeError('Recording failed; inspect status.json')
                    time.sleep(max(0, .02-(time.monotonic()-tick)))
                recorder.record('check_complete', accepted_polls=reads, unique_samples=len(seen),
                                rejected_polls=errors)
            finally:
                if imu is not None:
                    imu.stop()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'Check failed: {exc}', file=sys.stderr)
        return 2
    print(json.dumps({'folder': str(recorder.folder), 'accepted_polls': reads,
                      'unique_samples': len(seen), 'rejected_polls': errors}, indent=2))
    return 2 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
