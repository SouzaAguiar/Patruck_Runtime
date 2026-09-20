"""Read committed telemetry chunks without importing or activating hardware."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def stats(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values:
        return None
    index = (len(values)-1)*.95
    lo = int(index)
    p95 = values[lo] + (values[min(lo+1, len(values)-1)]-values[lo])*(index-lo)
    return {'count': len(values), 'mean': statistics.mean(values),
            'median': statistics.median(values), 'p95': p95, 'max': values[-1]}


def summarize(folder):
    folder = Path(folder)
    metadata = json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
    status_path = folder/'status.json'
    status = json.loads(status_path.read_text(encoding='utf-8')) if status_path.exists() else {}
    kinds = Counter()
    metrics = {key: [] for key in ('inference_ms', 'motor_write_ms', 'previous_cycle_dt_ms',
                                  'loop_work_before_logging_ms', 'imu_age_ms')}
    joint_squares, joint_counts, names = [], [], []
    previous_sequence = -1
    gaps = duplicates = invalid_values = 0
    first_ns = last_ns = None
    over_budget = 0
    frequency = metadata['settings'].get('control_freq_hz', 50)
    command_min, command_max = [float('inf')]*3, [float('-inf')]*3
    null_sensor_cycles = 0
    web_cycles = Counter()
    web_age_ms = []
    for path in sorted(folder.glob('chunk-*.jsonl')):
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                sequence = row['sequence']
                gaps += max(0, sequence-previous_sequence-1)
                duplicates += int(sequence <= previous_sequence)
                previous_sequence = sequence
                kinds[row['kind']] += 1
                timestamp = row['recorded_monotonic_ns']
                first_ns = timestamp if first_ns is None else first_ns
                last_ns = timestamp
                if row['kind'] == 'runtime_ready':
                    names = row['joint_names']
                    joint_squares, joint_counts = [0.]*len(names), [0]*len(names)
                if row['kind'] != 'cycle':
                    continue
                web_state = row.get('command_source') or {}
                if web_state.get('source') == 'web':
                    web_cycles['fresh' if web_state['command_fresh'] else
                               'timed_out' if web_state['connected'] else 'disconnected'] += 1
                    web_age_ms.append(web_state.get('command_age_ms'))
                for key in metrics:
                    metrics[key].append(row['sensors'].get(key) if key == 'imu_age_ms' else row.get(key))
                work_ms = row.get('loop_work_before_logging_ms')
                if work_ms is not None and work_ms > 1000/frequency:
                    over_budget += 1
                for axis, value in enumerate(row['commands'][:3]):
                    if value is not None:
                        command_min[axis] = min(command_min[axis], value)
                        command_max[axis] = max(command_max[axis], value)
                positions = row['sensors']['joint_position_rad']
                targets = row['sensors']['previous_motor_targets_rad']
                sensors = row['sensors']
                arrays = [row['policy_obs'], row['action'], positions, sensors['joint_velocity_rad_s'],
                          sensors['gyro_rad_s'], sensors['accelerometer_m_s2']]
                nulls = sum(v is None for array in arrays for v in array)
                invalid_values += nulls
                null_sensor_cycles += int(nulls > 0)
                for index, (position, target) in enumerate(zip(positions, targets)):
                    if index < len(names) and position is not None and target is not None:
                        joint_squares[index] += (position-target)**2
                        joint_counts[index] += 1
    return {
        'session': str(folder.resolve()), 'label': metadata['label'],
        'model_sha256': metadata['settings'].get('model_sha256'),
        'status': status, 'records_by_kind': dict(kinds),
        'observed_span_s': (last_ns-first_ns)/1e9 if first_ns is not None else 0,
        'sequence_gaps_in_committed_chunks': gaps, 'out_of_order_or_duplicate_records': duplicates,
        'temporary_chunks_ignored': len(list(folder.glob('chunk-*.jsonl.tmp'))),
        'committed_records': sum(kinds.values()),
        'cycles_with_nonfinite_or_missing_array_values': null_sensor_cycles,
        'nonfinite_or_missing_array_values': invalid_values,
        'timings_ms': {k: stats(v) for k, v in metrics.items()},
        'cycles_exceeding_budget_before_logging': over_budget,
        'web_command_cycles': dict(web_cycles), 'web_command_age_ms': stats(web_age_ms),
        'command_min_xyz': [v if math.isfinite(v) else None for v in command_min],
        'command_max_xyz': [v if math.isfinite(v) else None for v in command_max],
        'joint_previous_target_rmse_rad': {
            name: math.sqrt(total/count) if count else None
            for name, total, count in zip(names, joint_squares, joint_counts)
        },
        'notes': [
            'Joint RMSE compares current measured position to the previous sent target; it includes servo lag.',
            'Cycle interval includes sleep and pauses; work timing excludes telemetry enqueue overhead.',
            'A recording/failed status after exit is incomplete. Committed chunks remain readable.',
            'No ground velocity, absolute attitude, torque, current or automatic fall classification is measured.',
        ],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', help='Session folder containing metadata.json and chunks')
    args = parser.parse_args()
    print(json.dumps(summarize(args.session), ensure_ascii=False, allow_nan=False, indent=2))
