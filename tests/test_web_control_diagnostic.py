"""Exercise diagnostics without web servers, I2C, camera or motor imports."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"mini_bdx_runtime"))
spec = importlib.util.spec_from_file_location("web_diagnostic", ROOT/"scripts/web_control_test.py")
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)
from mini_bdx_runtime.imu_safety import LatestImuSample


class FakeWeb:
    def __init__(self, **kwargs):
        self.last_command_telemetry = dict(command_fresh=True, source="web")
    def get_last_command(self):
        return np.array([.1, 0, 0, 0, 0, 0, 0]), None, None, None
    def consume_desired_paused(self):
        return True  # Stop request must not stop a motor-free diagnostic.


class FakeImu:
    def __init__(self):
        self.samples = LatestImuSample(.05)
        self.calls = 0
        self.stopped = False
    def wait_ready(self):
        pass
    def get_data(self):
        self.calls += 1
        if self.calls == 2:
            self.samples.fail("injected transient fault")
            return self.samples.get()
        now = time.monotonic_ns()
        self.samples.publish(dict(gyro=[0., 0., 0.], accelero=[0., 0., 9.81],
                                  sample_start_monotonic_ns=now-1000000,
                                  sample_end_monotonic_ns=now, sample_index=self.calls))
        return self.samples.get()
    def stop(self):
        self.stopped = True


@pytest.mark.parametrize("gc_policy", ["default", "defer"])
@pytest.mark.parametrize("mode", ["web", "imu", "onnx", "telemetry"])
def test_modes_record_and_continue_after_fault(tmp_path, monkeypatch, mode, gc_policy):
    monkeypatch.setattr(diag, "memory_snapshot", lambda: (100*1024**2, 200*1024**2))
    monkeypatch.setitem(sys.modules, "mini_bdx_runtime.web_controller", SimpleNamespace(WebController=FakeWeb))
    monkeypatch.chdir(tmp_path)
    (tmp_path/"imu_calib_data.pkl").write_bytes(b"unused fake calibration")
    (tmp_path/"config.json").write_text("{}")
    (tmp_path/"fake.onnx").write_bytes(b"fake model")
    imu = FakeImu()
    monkeypatch.setattr(diag, "make_imu", lambda *a: imu)
    calls = []
    class Session:
        def __init__(self, *a, **kw):
            pass
        def get_inputs(self):
            return [SimpleNamespace(name="obs", shape=[1,101], type="tensor(float)")]
        def run(self, _, feed):
            calls.append(feed["obs"].copy())
            return [np.zeros((1,14), np.float32)]
    monkeypatch.setitem(sys.modules, "onnxruntime", SimpleNamespace(
        SessionOptions=SimpleNamespace, InferenceSession=Session))
    output = tmp_path/"output"
    result = diag.main(["--mode", mode, "--duration", "1", "--max-age-ms", "10000", "--output", str(output),
                        "--config", str(tmp_path/"config.json"), "--onnx-model", str(tmp_path/"fake.onnx"),
                        "--gc-policy", gc_policy, "--writer-yield-ms",
                        "1" if mode == "telemetry" and gc_policy == "defer" else "0"])
    folder = next(output.iterdir())
    rows = [json.loads(l) for f in sorted(folder.glob("chunk-*.jsonl")) for l in f.read_text().splitlines()]
    cycles = [r for r in rows if r["kind"] == "diagnostic_cycle"]
    assert len(cycles) >= 3
    assert all(r["desired_paused"] is True for r in cycles)
    assert result == (0 if mode == "web" else 2)
    if mode != "web":
        assert imu.stopped
        assert cycles[1]["faults"][0]["stage"] == "get_data"
        assert cycles[2]["faults"] == []
    if mode in ("onnx", "telemetry"):
        assert len(calls) == len(cycles)-1
        assert calls[0][0,6] == np.float32(.1)
    status = json.loads((folder/"status.json").read_text())
    assert status["dropped_records"] == 0 and status["unwritten_records"] == 0
    assert ("serialization_load_only" in cycles[0]) == (mode in ("onnx", "telemetry"))
    assert (folder/"gc_timing.json").is_file()
    assert (folder/"i2c_timing.json").is_file()
    assert (folder/"writer_timing.json").is_file()


def test_age_rejection_identifies_stage_and_sample():
    imu = FakeImu()
    sample = imu.get_data()
    sample["sample_start_monotonic_ns"] -= 100000000
    used, fault = diag.inspect_sample(imu, sample, .05, "after_inference")
    assert used is sample
    assert fault["stage"] == "after_inference"
    assert fault["used_sample_index"] == sample["sample_index"]


def test_no_sample_yet_does_not_crash():
    sample, fault = diag.inspect_sample(FakeImuEmpty(), None, .05, "get_data")
    assert sample is None and fault["latest"] == {}


class FakeImuEmpty:
    def __init__(self):
        self.samples = LatestImuSample(.05)
    def get_data(self):
        return self.samples.get()


def test_invalid_duration_and_missing_model_fail_before_imports():
    with pytest.raises(SystemExit):
        diag.main(["--duration", "nan"])
    with pytest.raises(SystemExit):
        diag.main(["--mode", "onnx"])


def test_transaction_trace_preserves_buffers_and_errors():
    trace = diag.TimingTrace(4, 4)
    class Device:
        def write_then_readinto(self, outgoing, incoming, **kwargs):
            incoming[0] = 12
            return 42
    device = diag.TimedDevice(Device(), trace)
    aliased = bytearray([61])
    assert device.write_then_readinto(aliased, aliased) == 42
    assert aliased == bytearray([12])
    event = trace.export()["rows"][0]
    assert event[2:] == [61, 0] and event[1] >= event[0]
    class Broken:
        def write_then_readinto(self, *args, **kwargs):
            raise OSError("injected I2C")
    with pytest.raises(OSError):
        diag.TimedDevice(Broken(), trace).write_then_readinto(bytearray([20]), bytearray(6))
    assert trace.export()["rows"][1][2:] == [20, 1]


def test_trace_overflow_is_explicit_and_bounded():
    trace = diag.TimingTrace(1, 2)
    trace.add(1, 2)
    trace.add(3, 4)
    assert trace.export() == dict(count=1, dropped=1, rows=[[1,2]])


def test_gc_observer_is_removed_after_setup_error(tmp_path, monkeypatch):
    import gc
    monkeypatch.setitem(sys.modules, "mini_bdx_runtime.web_controller", SimpleNamespace(WebController=lambda **kw: (_ for _ in ()).throw(RuntimeError("setup"))))
    before = list(gc.callbacks)
    assert diag.main(["--mode", "web", "--duration", ".1", "--output", str(tmp_path)]) == 2
    assert gc.callbacks == before


def test_defer_restores_gc_after_interrupt_and_saves_memory(tmp_path, monkeypatch):
    import gc
    class InterruptedWeb(FakeWeb):
        def get_last_command(self):
            assert not gc.isenabled()
            raise KeyboardInterrupt
    monkeypatch.setitem(sys.modules, "mini_bdx_runtime.web_controller", SimpleNamespace(WebController=InterruptedWeb))
    monkeypatch.setattr(diag, "memory_snapshot", lambda: (100*1024**2, 200*1024**2))
    before, callbacks = gc.isenabled(), list(gc.callbacks)
    result = diag.main(["--mode", "web", "--gc-policy", "defer", "--duration", ".1", "--output", str(tmp_path)])
    assert result == 130 and gc.isenabled() == before and gc.callbacks == callbacks
    folder = next(tmp_path.iterdir())
    assert json.loads((folder/"memory_timing.json").read_text())[0]["rss_bytes"] == 100*1024**2


def test_memory_growth_aborts_and_preserves_preexisting_gc_state(monkeypatch):
    import gc
    before = gc.isenabled()
    try:
        gc.disable()
        monkeypatch.setattr(diag, "memory_snapshot", lambda: (100*1024**2, 200*1024**2))
        guard = diag.MeasurementGuard('defer', 64, 64)
        guard.start()
        monkeypatch.setattr(diag, "memory_snapshot", lambda: (165*1024**2, 200*1024**2))
        with pytest.raises(RuntimeError, match='Memory guard'):
            guard.check(force=True)
        guard.restore()
        assert not gc.isenabled()
    finally:
        if before:gc.enable()


def test_defer_requires_memory_measurements(monkeypatch):
    import gc
    before = gc.isenabled()
    monkeypatch.setattr(diag, "memory_snapshot", lambda: (None, None))
    guard = diag.MeasurementGuard('defer', 64, 64)
    with pytest.raises(RuntimeError, match='requires Linux'):
        guard.start()
    guard.restore()
    assert gc.isenabled() == before


def test_defer_duration_and_yield_validation():
    with pytest.raises(SystemExit):
        diag.main(['--gc-policy', 'defer', '--duration', '121'])
    with pytest.raises(SystemExit):
        diag.main(['--mode', 'onnx', '--writer-yield-ms', '1'])


def test_stop_failure_restores_gc_and_closes_recording(tmp_path, monkeypatch):
    import gc
    class BrokenStop(FakeImu):
        def stop(self):
            raise RuntimeError('stop failure')
    monkeypatch.setitem(sys.modules, "mini_bdx_runtime.web_controller", SimpleNamespace(WebController=FakeWeb))
    monkeypatch.setattr(diag, 'make_imu', lambda *args: BrokenStop())
    monkeypatch.setattr(diag, 'memory_snapshot', lambda: (100*1024**2, 200*1024**2))
    monkeypatch.chdir(tmp_path)
    (tmp_path/'imu_calib_data.pkl').write_bytes(b'fake')
    (tmp_path/'config.json').write_text('{}')
    before, callbacks = gc.isenabled(), list(gc.callbacks)
    result = diag.main(['--mode', 'imu', '--gc-policy', 'defer', '--duration', '.1',
                        '--config', str(tmp_path/'config.json'), '--output', str(tmp_path/'output')])
    assert result == 2 and gc.isenabled() == before and gc.callbacks == callbacks
    folder = next((tmp_path/'output').iterdir())
    status = json.loads((folder/'status.json').read_text())
    assert status['state'] == 'closed' and status['close_reason'] == 'error'
    assert status['dropped_records'] == status['unwritten_records'] == 0
