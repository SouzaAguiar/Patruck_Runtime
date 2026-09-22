"""Web/IMU/ONNX load diagnostics. Never imports walking or motor control."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mini_bdx_runtime"))


def observation(sample, commands):
    import numpy as np
    # Synthetic standing observation: no joint hardware and no feedback of actions.
    obs = np.zeros(101, dtype=np.float32)
    obs[5] = 9.81
    if sample is not None:
        obs[:3], obs[3:6] = sample["gyro"], sample["accelero"]
    obs[6:9] = commands[:3]
    obs[97:99] = 1
    obs[99] = 1
    return obs


def inspect_sample(imu, sample, limit_s, stage):
    from mini_bdx_runtime.imu_safety import ImuDataError, check_sample
    now = time.monotonic_ns()
    try:
        if sample is None:
            sample = imu.get_data()
        else:
            check_sample(sample, limit_s, now_ns=now)
        return sample, None
    except ImuDataError as exc:
        # Snapshot, never use rejected readings for inference.
        with imu.samples.condition:
            latest = imu.samples.sample
            detail = ({k: latest.get(k) for k in (
                "sample_index", "sample_start_monotonic_ns", "sample_end_monotonic_ns")}
                if latest is not None else {})
            error = imu.samples.error
        return sample, dict(stage=stage, message=str(exc), observed_monotonic_ns=now,
                            latest=detail, producer_error=error,
                            used_sample_index=sample.get("sample_index") if sample else None)


def make_imu(bus, limit_s, config, producer_events):
    from mini_bdx_runtime.raw_imu import Imu

    class DiagnosticImu(Imu):
        def imu_worker(self):
            original = self.samples.fail
            def record_failure(error):
                producer_events.append(dict(monotonic_ns=time.monotonic_ns(), message=str(error)))
                original(error)
            self.samples.fail = record_failure
            super().imu_worker()

    return DiagnosticImu(50, upside_down=config.get("imu_upside_down", False),
                         i2c_bus=bus, max_age_s=limit_s)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("web", "imu", "onnx", "telemetry"), default="web")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token", default="duck-test")
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--i2c-bus", type=int, default=8)
    parser.add_argument("--max-age-ms", type=float, default=50)
    parser.add_argument("--config", type=Path, default=Path.home()/"duck_config.json")
    parser.add_argument("--onnx-model", type=Path)
    parser.add_argument("--onnx-threads", type=int, default=0,
                        help="0 preserves ONNX defaults; 1 is a separate comparison")
    parser.add_argument("--output", type=Path, default=ROOT/"web_imu_diagnostics")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 300:
        parser.error("duration must be in (0, 300] (bounded in-memory recording)")
    if not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0 or args.i2c_bus < 0:
        parser.error("Use a positive age and nonnegative I2C bus")
    if args.onnx_threads < 0:
        parser.error("onnx-threads must be nonnegative")
    use_imu = args.mode != "web"
    use_onnx = args.mode in ("onnx", "telemetry")
    if use_onnx and (args.onnx_model is None or not args.onnx_model.is_file()):
        parser.error("onnx and telemetry modes require --onnx-model pointing to an existing file")
    if use_imu and (not args.config.is_file() or not Path("imu_calib_data.pkl").is_file()):
        parser.error("Run from scripts with imu_calib_data.pkl and an existing --config")

    import numpy as np
    from mini_bdx_runtime.web_controller import WebController
    from mini_bdx_runtime.telemetry import TelemetryRecorder, file_sha256, plain
    config = json.loads(args.config.read_text(encoding="utf-8")) if use_imu else {}
    events = deque(maxlen=20000)
    writer_events = deque(maxlen=2000)

    class TimedRecorder(TelemetryRecorder):
        def _publish(self, rows):
            start = time.monotonic_ns()
            try:
                super()._publish(rows)
            finally:
                writer_events.append(dict(start_monotonic_ns=start, end_monotonic_ns=time.monotonic_ns(),
                                          records=len(rows)))

    metadata = dict(mode=args.mode, i2c_bus=args.i2c_bus if use_imu else None,
                    max_age_ms=args.max_age_ms, camera=args.camera, frequency_hz=50,
                    onnx_threads=args.onnx_threads, motors_controlled_by_script=False,
                    observations="synthetic joints/history; real IMU and web commands when enabled",
                    model_sha256=file_sha256(args.onnx_model) if use_onnx else None,
                    config_sha256=file_sha256(args.config) if use_imu else None,
                    calibration_sha256=file_sha256("imu_calib_data.pkl") if use_imu else None,
                    script_sha256=file_sha256(__file__),
                    source_sha256={n:file_sha256(ROOT/"mini_bdx_runtime"/"mini_bdx_runtime"/n)
                                   for n in ("raw_imu.py", "imu_safety.py", "web_controller.py", "telemetry.py")})
    recorder = TimedRecorder(args.output, label="web-diagnostic-"+args.mode, metadata=metadata)
    imu = None
    buffered = []
    polls = faulty_polls = 0
    reason = "completed"
    exit_code = 0
    measurement_end_ns = None
    try:
        session = None
        if use_onnx:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = args.onnx_threads
            session = ort.InferenceSession(str(args.onnx_model), sess_options=options,
                                           providers=["CPUExecutionProvider"])
            inp = session.get_inputs()[0]
            if len(session.get_inputs()) != 1 or inp.shape[-1] != 101 or inp.type != "tensor(float)":
                raise ValueError("Expected one float32 ONNX input with 101 observations")
            if len(inp.shape) not in (1, 2):
                raise ValueError("Expected ONNX input shape [101] or [1,101]")
        if use_imu:
            imu = make_imu(args.i2c_bus, args.max_age_ms/1000, config, events)
            imu.wait_ready()
        controller = WebController(command_freq=20, port=args.port, token=args.token, camera=args.camera)
        print("DIAGNOSTICO SEM MOTORES: INICIAR/PARAR sao registrados; a coleta continua apos falhas.")
        start = previous = time.monotonic_ns()
        deadline = start + int(args.duration*1e9)
        while time.monotonic_ns() < deadline:
            tick = time.monotonic_ns()
            commands, _, _, _ = controller.get_last_command()
            row = dict(cycle=polls, cycle_start_monotonic_ns=tick, interval_ms=(tick-previous)/1e6,
                       commands=commands.tolist(), command_source=controller.last_command_telemetry.copy(),
                       desired_paused=controller.consume_desired_paused(), faults=[])
            previous = tick
            row["command_end_monotonic_ns"] = time.monotonic_ns()
            sample = None
            if imu is not None:
                sample, fault = inspect_sample(imu, None, args.max_age_ms/1000, "get_data")
                if fault:
                    row["faults"].append(fault)
                if sample is not None:
                    row["imu"] = plain(sample)
            row["imu_check_end_monotonic_ns"] = time.monotonic_ns()
            action = None
            if session is not None and not row["faults"]:
                obs = observation(sample, commands)
                feed = obs[None] if len(inp.shape) == 2 else obs
                row["inference_start_monotonic_ns"] = time.monotonic_ns()
                action = np.asarray(session.run(None, {inp.name: feed})[0]).reshape(-1)
                row["inference_end_monotonic_ns"] = time.monotonic_ns()
                if action.shape != (14,) or not np.isfinite(action).all():
                    raise ValueError("Expected 14 finite ONNX actions")
                _, fault = inspect_sample(imu, sample, args.max_age_ms/1000, "after_inference")
                if fault:
                    row["faults"].append(fault)
                if args.mode == "telemetry":
                    # Approximate runtime payload size, explicitly synthetic and never sent to servos.
                    row.update(policy_obs=obs.tolist(), observed_obs=obs.tolist(), action=action.tolist(),
                               synthetic_targets=(action*.25).tolist(), synthetic_previous_targets=[0.]*14,
                               synthetic_positions=[0.]*14, synthetic_velocities=[0.]*14,
                               synthetic_servo_goals=(action*.25).tolist())
            row["work_end_monotonic_ns"] = time.monotonic_ns()
            faulty_polls += bool(row["faults"])
            polls += 1
            if args.mode == "telemetry":
                recorder.record("diagnostic_cycle", **row)
            else:
                buffered.append(row)
            if recorder.error or recorder.dropped:
                raise RuntimeError("Telemetry writer failure or dropped records")
            time.sleep(max(0, .02-(time.monotonic_ns()-tick)/1e9))
    except KeyboardInterrupt:
        reason, exit_code = "interrupted", 130
    except Exception as exc:
        reason, exit_code = "error", 2
        recorder.record("diagnostic_error", message=f"{type(exc).__name__}: {exc}")
        print(f"Diagnostic failed: {exc}", file=sys.stderr)
    finally:
        measurement_end_ns = time.monotonic_ns()
        if imu is not None:
            imu.stop()
        # Non-telemetry stages write only after measurements stop; same JSONL format.
        for row in buffered:
            while recorder.queue.full() and not recorder.error:
                time.sleep(.01)
            recorder.record("diagnostic_cycle", **row)
        while recorder.queue.qsize() > recorder.queue.maxsize-3 and not recorder.error:
            time.sleep(.01)
        producer_faults = sum(e["message"] != "IMU: leitura encerrada" for e in events)
        recorder.record("diagnostic_summary", polls=polls, faulty_polls=faulty_polls,
                        measurement_end_monotonic_ns=measurement_end_ns,
                        producer_faults=producer_faults, producer_events=list(events),
                        producer_event_buffer_full=len(events)==events.maxlen)
        recorder.close(reason)
        (recorder.folder/"writer_timing.json").write_text(json.dumps(list(writer_events), indent=2), encoding="utf-8")
        print(json.dumps(dict(folder=str(recorder.folder), polls=polls, faulty_polls=faulty_polls,
                              mode=args.mode, reason=reason), indent=2))
    if recorder.error or recorder.dropped or recorder.thread.is_alive():
        return 2
    return exit_code or (2 if faulty_polls or producer_faults else 0)


if __name__ == "__main__":
    raise SystemExit(main())
