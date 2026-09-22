"""Runtime GC integration with fake clocks/memory and existing motor stubs."""
import gc
import json
from types import SimpleNamespace

import pytest
from test_telemetry import ROOT, load_walk, make_walk
from mini_bdx_runtime import runtime_budget as module
from mini_bdx_runtime.runtime_budget import RuntimeBudget, RuntimeBudgetError
from mini_bdx_runtime.telemetry import TelemetryRecorder


@pytest.fixture
def clock_memory(monkeypatch):
    state = dict(now=0., rss=100*2**20, available=200*2**20)
    monkeypatch.setattr(module, 'memory_snapshot', lambda: (state['rss'], state['available']))
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda:state['now'], monotonic_ns=lambda:int(state['now']*1e9)))
    before = gc.isenabled()
    yield state
    gc.enable() if before else gc.disable()


def test_deadline_and_pause_maintenance_resume(clock_memory):
    before = gc.isenabled()
    guard = RuntimeBudget(active_seconds=30)
    guard.maintain()
    guard.begin()
    assert not gc.isenabled()
    clock_memory['now'] = 29
    guard.check()
    guard.begin()  # Duplicate command cannot extend the window.
    clock_memory['now'] = 30
    with pytest.raises(RuntimeBudgetError, match='Limite'):
        guard.check()
    guard.pause()
    assert gc.isenabled() == before
    guard.maintain()
    guard.begin()
    assert guard.started == 30
    guard.close()
    assert gc.isenabled() == before


def test_gc_maintenance_forbidden_while_active(clock_memory):
    guard = RuntimeBudget()
    guard.maintain(); guard.begin()
    with pytest.raises(RuntimeError, match='paused'):
        guard.maintain()
    guard.close()


def test_memory_baseline_not_reset_on_resume(clock_memory):
    guard = RuntimeBudget()
    guard.maintain(); guard.begin()
    clock_memory['rss'] += 65*2**20
    with pytest.raises(RuntimeBudgetError, match='Crescimento'):
        guard.check()
    guard.pause()
    with pytest.raises(RuntimeBudgetError, match='Crescimento'):
        guard.maintain()
    assert guard.baseline == 100*2**20
    guard.close()


def test_memory_unavailable_or_low_rejects_preflight(monkeypatch):
    monkeypatch.setattr(module, 'memory_snapshot', lambda:(100*2**20, 63*2**20))
    with pytest.raises(RuntimeBudgetError, match='disponivel'):
        RuntimeBudget()


def test_recorder_failure_blocks_begin_and_active_control(clock_memory):
    recorder = SimpleNamespace(error=None, dropped=0, record=lambda *a, **k: None)
    guard = RuntimeBudget(recorder=recorder)
    guard.maintain(); guard.begin()
    recorder.dropped = 1
    with pytest.raises(RuntimeBudgetError, match='telemetria'):
        guard.check()
    guard.pause(); guard.maintain()
    with pytest.raises(RuntimeBudgetError, match='telemetria'):
        guard.begin()
    guard.close()


@pytest.mark.parametrize('fail_at', [1,2])
def test_guard_failure_never_sends_motor_target(fail_at):
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, inputs, writes = make_walk(cls)
    class Guard:
        needs_maintenance = False
        checks = 0
        closed = False
        def check(self):
            self.checks += 1
            if self.checks == fail_at:
                raise RuntimeBudgetError('limit')
        def pause(self):pass
        def close(self):self.closed = True
    guard = Guard(); walk.runtime_budget = guard
    def stop_on_pause(_):
        if walk.paused:raise KeyboardInterrupt
    cls.run.__globals__['time'].sleep = stop_on_pause
    before = walk.last_action.copy()
    walk.run()
    assert not writes and len(inputs) == fail_at-1
    assert (walk.last_action == before).all() and walk.imitation_i == 0
    assert walk.runtime_fault == 'limit' and guard.closed


def test_resume_requires_ack_center_and_fresh_imu(clock_memory):
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, _, _ = make_walk(cls)
    guard = RuntimeBudget(); walk.runtime_budget = guard
    walk._pause_for_runtime(RuntimeBudgetError('time limit'))
    walk._request_pause(False)
    assert walk.paused
    walk._request_pause(True)
    walk._request_pause(False)
    assert walk.paused  # Commands are still nonzero.
    walk.last_commands = [0.]*7
    seen = []
    walk.imu.wait_ready = lambda **kw: seen.append((guard.started, guard.needs_maintenance))
    walk._request_pause(False)
    assert seen == [(None, False)]  # Maintenance precedes readiness, before GC suspension.
    assert not walk.paused and walk.runtime_fault is None and not gc.isenabled()
    old_start = guard.started
    clock_memory['now'] = 20
    walk._request_pause(False)
    assert guard.started == old_start
    guard.close()


def test_gc_restored_if_sensor_stop_fails(clock_memory):
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, _, _ = make_walk(cls)
    guard = RuntimeBudget(); guard.maintain(); guard.begin(); walk.runtime_budget = guard
    before = guard.original_gc
    walk.start = lambda: (_ for _ in ()).throw(KeyboardInterrupt())
    walk.imu.stop = lambda: (_ for _ in ()).throw(RuntimeError('stop failed'))
    with pytest.raises(RuntimeError, match='stop failed'):
        walk.run()
    assert gc.isenabled() == before


def test_cooperative_recorder_keeps_records(tmp_path):
    with TelemetryRecorder(tmp_path, writer_yield_ms=1, flush_seconds=.01) as recorder:
        for n in range(20):recorder.record('example', value=n)
    rows=[json.loads(l) for p in sorted(recorder.folder.glob('chunk-*.jsonl')) for l in p.read_text().splitlines()]
    assert [r['value'] for r in rows if r['kind']=='example'] == list(range(20))
    status=json.loads((recorder.folder/'status.json').read_text())
    assert status['dropped_records']==status['unwritten_records']==0


def test_memory_preflight_precedes_motor_initialization():
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, _, writes = make_walk(cls)
    walk.runtime_budget = SimpleNamespace(preflight=lambda: (_ for _ in ()).throw(RuntimeBudgetError('low memory')))
    with pytest.raises(RuntimeBudgetError, match='low memory'):
        cls.start(walk)
    assert not writes


def test_failed_imu_readiness_does_not_disable_gc_or_resume(clock_memory):
    from mini_bdx_runtime.imu_safety import ImuDataError
    cls = load_walk((ROOT/'scripts/v2_rl_walk_mujoco.py').read_text(encoding='utf-8'))
    walk, _, writes = make_walk(cls)
    walk.paused = True
    walk.last_commands = [0.]*7
    guard = RuntimeBudget(); walk.runtime_budget = guard
    before = gc.isenabled()
    walk.imu.wait_ready = lambda **kw: (_ for _ in ()).throw(ImuDataError('not ready'))
    walk._request_pause(False)
    assert walk.paused and walk.imu_fault == 'not ready' and guard.started is None
    assert gc.isenabled() == before and not writes
    guard.close()
