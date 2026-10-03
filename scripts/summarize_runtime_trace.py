"""Offline timing summary. Reads saved JSON only; never imports hardware."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), median_ms=statistics.median(values),
                p99_ms=values[math.ceil(.99*len(values))-1], max_ms=values[-1])


def summarize(folder):
    trace = json.loads((folder/'runtime_timing_trace.json').read_text(encoding='utf-8'))
    if trace['schema'] != 1:
        raise ValueError('Unsupported runtime timing trace schema')
    records = [json.loads(line) for file in sorted(folder.glob('chunk-*.jsonl'))
               for line in file.read_text(encoding='utf-8').splitlines()]
    imu = {}
    for timestamp, event, index, start, due in trace['imu']['rows']:
        item = imu.setdefault(index, {'index':index, 'start_ns':start, 'due_ns':due})
        item[trace['imu_events'][str(event)]] = timestamp
    samples = list(imu.values())
    publications = [s['published'] for s in samples if 'published' in s]
    transactions = []
    pending = None
    unpaired = 0
    for timestamp, end, register, failed in trace['i2c']['rows']:
        if not end:
            if pending is not None:
                unpaired += 1
            pending = dict(start_ns=timestamp, register=register)
        elif pending is not None and pending['register'] == register:
            transactions.append(dict(pending, end_ns=timestamp, failed=bool(failed),
                                     duration_ms=(timestamp-pending['start_ns'])/1e6))
            pending = None
        else:
            unpaired += 1
    controls = [dict(time_ns=row[0], event=trace['control_events'][str(row[1])],
                     attempt=row[2], sample_index=row[3]) for row in trace['control']['rows']]
    record_durations, logging_start = [], {}
    for event in controls:
        if event['event'] == 'telemetry_begin':
            logging_start[event['attempt']] = event['time_ns']
        elif event['event'] == 'telemetry_end' and event['attempt'] in logging_start:
            record_durations.append((event['time_ns']-logging_start.pop(event['attempt']))/1e6)
    gc_intervals, gc_pending = [], {}
    for timestamp, stop, generation, collected, uncollectable in trace['gc']['rows']:
        if not stop:
            gc_pending[generation] = timestamp
        elif generation in gc_pending:
            started = gc_pending.pop(generation)
            gc_intervals.append(dict(start_ns=started, end_ns=timestamp,
                                     generation=generation, duration_ms=(timestamp-started)/1e6))
    writer_path = folder/'writer_timing.json'
    writer = json.loads(writer_path.read_text(encoding='utf-8')) if writer_path.is_file() else {}
    batches = writer.get('batches', [])
    faults = []
    for record in records:
        if record['kind'] != 'imu_fault':
            continue
        point = record['detected_monotonic_ns']
        before, after = point-100_000_000, point+100_000_000
        index = (record.get('used_sample') or {}).get('sample_index')
        faults.append(dict(time_ns=point, message=record['message'], diagnostic=record.get('diagnostic'),
                           used_sample_producer=imu.get(index), latest_reader=record.get('latest_reader'),
                           imu_events_near_fault=[s for s in samples if before <= s['start_ns'] <= after],
                           control_events_near_fault=[e for e in controls if before <= e['time_ns'] <= after],
                           i2c_calls_overlapping_fault=[t for t in transactions if t['start_ns'] <= point <= t['end_ns']],
                           gc_overlapping_fault=[g for g in gc_intervals if g['start_ns'] <= point <= g['end_ns']],
                           writer_overlapping_fault=[b for b in batches if b['start_monotonic_ns'] <= point <= b['end_monotonic_ns']]))
    cycles = [r for r in records if r['kind']=='cycle']
    return dict(schema=1, session=folder.name,
                warning='Timing correlations do not prove causality; traces are bounded and may omit older events.',
                overwritten={name:trace[name]['overwritten'] for name in ('imu','i2c','control','gc')},
                allocated_numeric_bytes=trace['allocated_numeric_bytes'],
                status=json.loads((folder/'status.json').read_text(encoding='utf-8')) if (folder/'status.json').is_file() else None,
                cycle_count=len(cycles), nonzero_command_cycles=sum(any(r['commands']) for r in cycles),
                record_kinds=dict(Counter(r['kind'] for r in records)),
                acquisition_ms=stats([(s['read_end']-s['start_ns'])/1e6 for s in samples if 'read_end' in s]),
                publication_after_read_ms=stats([(s['published']-s['read_end'])/1e6 for s in samples if 'published' in s and 'read_end' in s]),
                acquisition_start_lateness_ms=stats([(s['acquisition_start']-s['due_ns'])/1e6 for s in samples if 'acquisition_start' in s and s['due_ns']]),
                publication_interval_ms=stats([(b-a)/1e6 for a,b in zip(publications,publications[1:])]),
                i2c_call_ms=stats([t['duration_ms'] for t in transactions]),
                i2c_failed_calls=sum(t['failed'] for t in transactions), i2c_unpaired_events=unpaired,
                i2c_unfinished_call=pending, producer_failed_samples=sum('failure' in s for s in samples),
                main_telemetry_record_ms=stats(record_durations), gc_intervals=gc_intervals,
                faults=faults)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    result = summarize(args.session)
    text = json.dumps(result, indent=2, ensure_ascii=False)+'\n'
    if args.output:
        args.output.write_text(text, encoding='utf-8')
    else:
        print(text, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
