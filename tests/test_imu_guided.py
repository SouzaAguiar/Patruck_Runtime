"""Verify continuous capture and stage attribution with a fake clock/sensor."""
from types import SimpleNamespace
from test_imu_diagnostic import diagnostic


def test_plan_has_two_rounds_and_no_gaps():
    plan = diagnostic.frame_test_plan()
    assert len(plan) == 31 and plan[0]['start_s'] == 0 and plan[-1]['end_s'] == 95
    for a, b in zip(plan, plan[1:]):
        assert a['end_s'] == b['start_s']
    for repeat in (1, 2):
        for axis in ('pitch', 'roll', 'yaw'):
            assert [s['phase'] for s in plan if s.get('axis') == axis and s.get('repetition') == repeat] == ['ready', 'outbound', 'hold', 'return', 'rest']


def test_one_continuous_acquisition_identifies_every_stage(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(diagnostic, 'time', SimpleNamespace(
        monotonic=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0]+s)))
    pair = {'abnormal': False, 'gyro': {'driver_values': [0, 0, 0]},
            'acceleration': {'driver_values': [0, 0, 9.81]}}
    monkeypatch.setattr(diagnostic, 'read_pair', lambda *args: pair)
    rows = []
    recorder = SimpleNamespace(error=None, dropped=0,
        record=lambda kind, **data: rows.append(dict(kind=kind, **data)))
    plan = diagnostic.frame_test_plan()
    diagnostic.acquire(None, None, recorder, 95, 1, plan=plan, imu_frame='yaw-plus-90')
    samples = [r for r in rows if r['kind'] == 'imu_sample']
    cues = [r for r in rows if r['kind'] == 'guided_stage']
    assert len(samples) == 95 and len(cues) == 31
    assert [r['index'] for r in samples] == list(range(95))
    for sample in samples:
        stage = next(s for s in plan if s['start_s'] <= sample['elapsed_s'] < s['end_s'])
        assert sample['guided_stage'] == stage['name']
