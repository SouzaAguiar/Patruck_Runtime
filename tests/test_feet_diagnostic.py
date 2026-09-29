"""Offline diagnostic tests; no GPIO or motors are imported."""
import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location('feet_diagnostic', Path(__file__).resolve().parents[1]/'scripts/diagnose_feet_contacts.py')
diag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diag)


def test_expected_states_and_stuck_or_swapped_inputs():
    for _, expected, _ in diag.STAGES:
        samples = [{'contacts_left_right': expected} for _ in range(60)]
        assert diag.evaluate(samples, expected, 3, 20)['passed']
    stuck = [{'contacts_left_right': [True, True]} for _ in range(60)]
    assert not diag.evaluate(stuck, [False, False], 3, 20)['passed']
    swapped = [{'contacts_left_right': [False, True]} for _ in range(60)]
    assert not diag.evaluate(swapped, [True, False], 3, 20)['passed']


def test_empty_or_insufficient_capture_cannot_pass():
    assert not diag.evaluate([], [False, False], 3, 20)['passed']
    assert not diag.evaluate([{'contacts_left_right': [False, False]}], [False, False], 3, 20)['passed']


def test_capture_preserves_raw_and_logical_states():
    class Clock:
        value = 0
        def now(self): return self.value
        def sleep(self, duration): self.value += duration
    class Reader:
        def read(self): return [False, True], [True, False]
    clock = Clock()
    events = []
    samples = diag.collect_stage(Reader(), 1, 4, lambda kind, **data: events.append((kind, data)), clock.now, clock.sleep)
    assert len(samples) == 4
    assert events[0][1]['raw_gpio_high_left_right'] == [False, True]
    assert diag.evaluate(samples, [True, False], 1, 4)['passed']


def test_interrupt_saves_partial_result_and_releases_gpio(tmp_path, monkeypatch):
    import json
    closed = []
    class Reader:
        def close(self): closed.append(True)
    inputs = iter(['', KeyboardInterrupt()])
    def answer(prompt):
        value = next(inputs)
        if isinstance(value, BaseException): raise value
        return value
    monkeypatch.setattr(diag, 'ContactReader', Reader)
    monkeypatch.setattr('builtins.input', answer)
    assert diag.main(['--output', str(tmp_path)]) == 130
    result = json.loads(next(tmp_path.glob('*/summary.json')).read_text())
    assert result['state'] == 'interrupted' and not result['passed']
    assert closed == [True]
