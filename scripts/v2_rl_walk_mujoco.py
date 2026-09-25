import time
import pickle
from contextlib import nullcontext
from pathlib import Path

import numpy as np
from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer

from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.imu_safety import ImuDataError, check_sample
from mini_bdx_runtime.runtime_budget import RuntimeBudget, RuntimeBudgetError
from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion
from mini_bdx_runtime.feet_contacts import FeetContacts
from mini_bdx_runtime.eyes import Eyes
from mini_bdx_runtime.sounds import Sounds
from mini_bdx_runtime.antennas import Antennas
from mini_bdx_runtime.projector import Projector
from mini_bdx_runtime.rl_utils import make_action_dict, LowPassActionFilter
from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.telemetry import TelemetryRecorder, file_sha256

import os

HOME_DIR = os.path.expanduser("~")


class RLWalk:
    def __init__(
        self,
        onnx_model_path: str,
        duck_config_path: str = f"{HOME_DIR}/duck_config.json",
        serial_port: str = "/dev/ttyACM0",
        control_freq: float = 50,
        pid=[30, 0, 0],
        action_scale=0.25,
        commands=False,
        control_source="xbox",
        web_host="0.0.0.0",
        web_port=8080,
        web_token=None,
        web_command_timeout=0.6,
        allow_head_control=False,
        camera=False,
        camera_size=(640, 480),
        camera_fps=10,
        pitch_bias=0,
        save_obs=False,
        replay_obs=None,
        cutoff_frequency=None,
        telemetry=None,
        imu_i2c_bus=8,
        imu_max_age_ms=50,
        start_paused=None,
        runtime_budget=None,
    ):

        self.runtime_budget = runtime_budget
        self.runtime_fault = None
        self.telemetry = telemetry
        self.telemetry_observation = None
        self.imu_fault = None
        self.imu_fault_acknowledged = False
        self.observation_imu = None
        self.cycle_diagnostics = {}
        self.imu_max_age_s = imu_max_age_ms / 1000
        if not np.isfinite(self.imu_max_age_s) or self.imu_max_age_s <= 0:
            raise ValueError('imu-max-age-ms must be positive and finite')
        self.duck_config = DuckConfig(config_json_path=duck_config_path)

        self.commands = commands
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 14
        self.max_motor_velocity = 5.24  # rad/s

        # Control
        self.control_freq = control_freq
        self.pid = pid

        self.save_obs = save_obs
        if self.save_obs:
            self.saved_obs = []

        self.replay_obs = replay_obs
        if self.replay_obs is not None:
            self.replay_obs = pickle.load(open(self.replay_obs, "rb"))

        self.action_filter = None
        if cutoff_frequency is not None:
            self.action_filter = LowPassActionFilter(
                self.control_freq, cutoff_frequency
            )

        self.imu = Imu(
            sampling_freq=int(self.control_freq),
            user_pitch_bias=self.pitch_bias,
            upside_down=self.duck_config.imu_upside_down,
            i2c_bus=imu_i2c_bus,
            max_age_s=self.imu_max_age_s,
        )
        try:
            self.imu.wait_ready()
        except BaseException:
            self.imu.stop()
            raise

        self.hwi = HWI(self.duck_config, serial_port)

        self.feet_contacts = FeetContacts()

        # Scales
        self.action_scale = action_scale

        self.last_action = np.zeros(self.num_dofs)
        self.last_last_action = np.zeros(self.num_dofs)
        self.last_last_last_action = np.zeros(self.num_dofs)

        self.init_pos = list(self.hwi.init_pos.values())

        self.motor_targets = np.array(self.init_pos.copy())
        self.prev_motor_targets = np.array(self.init_pos.copy())

        self.last_commands = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.paused = True if runtime_budget is not None else (self.duck_config.start_paused if start_paused is None else start_paused)

        self.command_freq = 20  # hz
        self.controller = None
        if self.commands and control_source == "xbox":
            from mini_bdx_runtime.xbox_controller import XBoxController

            self.controller = XBoxController(self.command_freq)
        elif self.commands and control_source == "web":
            from mini_bdx_runtime.web_controller import WebController

            self.controller = WebController(
                self.command_freq,
                host=web_host,
                port=web_port,
                token=web_token,
                command_timeout=web_command_timeout,
                allow_head_control=allow_head_control,
                camera=camera,
                camera_size=camera_size,
                camera_fps=camera_fps,
            )

        # Reference motion, but we only really need the length of one phase
        # TODO
        self.PRM = PolyReferenceMotion("./polynomial_coefficients.pkl")
        self.imitation_i = 0
        self.imitation_phase = np.array([0, 0])
        self.phase_frequency_factor = 1.0
        self.phase_frequency_factor_offset = (
            self.duck_config.phase_frequency_factor_offset
        )

        # Optional expression features
        if self.duck_config.eyes:
            self.eyes = Eyes()
        if self.duck_config.projector:
            self.projector = Projector()
        if self.duck_config.speaker:
            self.sounds = Sounds(
                volume=1.0, sound_directory="../mini_bdx_runtime/assets/"
            )
        if self.duck_config.antennas:
            self.antennas = Antennas()

        if self.telemetry is not None:
            self.telemetry.record(
                'runtime_ready', joint_names=list(self.hwi.joints),
                joint_ids=list(self.hwi.joints.values()), home_rad=self.init_pos,
                offsets_rad=[self.hwi.joints_offsets[n] for n in self.hwi.joints],
                kps=self.hwi.kps, kds=self.hwi.kds, action_scale=self.action_scale,
                control_freq_hz=self.control_freq, start_paused=self.paused,
                imu_upside_down=self.duck_config.imu_upside_down,
                imu_i2c_bus=imu_i2c_bus, imu_max_age_ms=imu_max_age_ms,
                phase_period_steps=self.PRM.nb_steps_in_period,
                phase_frequency_offset=self.phase_frequency_factor_offset,
                cutoff_frequency_hz=cutoff_frequency, replay_obs=replay_obs is not None,
                imu_calibration_sha256=file_sha256('imu_calib_data.pkl'),
                reference_motion_sha256=file_sha256('polynomial_coefficients.pkl'),
            )

    def get_obs(self):

        self.observation_imu = None
        self._mark_imu_stage('get_imu')
        read_start_ns = time.monotonic_ns()
        imu_data = self.imu.get_data()
        self.observation_imu = imu_data
        self._mark_imu_stage('validate_imu')
        check_sample(imu_data, self.imu_max_age_s)
        imu_received_ns = time.monotonic_ns()

        self._mark_imu_stage('read_joint_positions')
        dof_pos = self.hwi.get_present_positions(
            ignore=[
                "left_antenna",
                "right_antenna",
            ]
        )  # rad
        position_end_ns = time.monotonic_ns()

        self._mark_imu_stage('read_joint_velocities', position_end_ns=position_end_ns)
        dof_vel = self.hwi.get_present_velocities(
            ignore=[
                "left_antenna",
                "right_antenna",
            ]
        )  # rad/s
        velocity_end_ns = time.monotonic_ns()

        if dof_pos is None or dof_vel is None:
            if self.telemetry is not None:
                self.telemetry.record('sensor_failure', reason='missing_joint_reading',
                                      positions_missing=dof_pos is None, velocities_missing=dof_vel is None)
            return None

        if len(dof_pos) != self.num_dofs:
            if self.telemetry is not None:
                self.telemetry.record('sensor_failure', reason='position_length', count=len(dof_pos))
            print(f"ERROR len(dof_pos) != {self.num_dofs}")
            return None

        if len(dof_vel) != self.num_dofs:
            if self.telemetry is not None:
                self.telemetry.record('sensor_failure', reason='velocity_length', count=len(dof_vel))
            print(f"ERROR len(dof_vel) != {self.num_dofs}")
            return None

        cmds = self.last_commands

        self._mark_imu_stage('read_contacts', velocity_end_ns=velocity_end_ns)
        feet_contacts = self.feet_contacts.get()
        contacts_end_ns = time.monotonic_ns()
        self._mark_imu_stage('after_joint_reads', contacts_end_ns=contacts_end_ns)
        check_sample(imu_data, self.imu_max_age_s)  # Joint reads may have blocked.

        if self.telemetry is not None:
            sample_end = imu_data.get('sample_end_monotonic_ns')
            self.telemetry_observation = {
                'read_start_monotonic_ns': read_start_ns,
                'imu_received_monotonic_ns': imu_received_ns,
                'position_end_monotonic_ns': position_end_ns,
                'velocity_end_monotonic_ns': velocity_end_ns,
                'contacts_end_monotonic_ns': contacts_end_ns,
                'imu_sample_start_monotonic_ns': imu_data.get('sample_start_monotonic_ns'),
                'imu_sample_end_monotonic_ns': sample_end,
                'imu_sample_index': imu_data.get('sample_index'),
                'imu_age_ms': (imu_received_ns-sample_end)/1e6 if sample_end is not None else None,
                'imu_oldest_age_ms': (contacts_end_ns-imu_data['sample_start_monotonic_ns'])/1e6,
                'gyro_rad_s': np.array(imu_data['gyro']).copy(),
                'accelerometer_m_s2': np.array(imu_data['accelero']).copy(),
                'joint_position_rad': dof_pos.copy(), 'joint_velocity_rad_s': dof_vel.copy(),
                'contacts_left_right': list(feet_contacts),
                'previous_motor_targets_rad': self.motor_targets.copy(),
            }

        obs = np.concatenate(
            [
                imu_data["gyro"],
                imu_data["accelero"],
                cmds,
                dof_pos - self.init_pos,
                dof_vel * 0.05,
                self.last_action,
                self.last_last_action,
                self.last_last_last_action,
                self.motor_targets,
                feet_contacts,
                self.imitation_phase,
            ]
        )

        return obs

    def _select_imu_for_inference(self, obs):
        # Joint reads are already complete. Refresh actual sensor values before
        # inference, never the timestamp of an action computed from older data.
        reserve_ns = min(10_000_000, int(self.imu_max_age_s * 1e9 / 2))
        started_ns = time.monotonic_ns()
        cycle_deadline_ns = (self.cycle_diagnostics['cycle_start_ns']
                             + int(1e9 / self.control_freq))
        deadline_ns = min(started_ns + 5_000_000, cycle_deadline_ns)
        initial_index = self.observation_imu.get('sample_index')
        polls = 0
        self._mark_imu_stage('select_imu_before_inference',
                             selection_initial_sample_index=initial_index,
                             selection_start_ns=started_ns,
                             selection_deadline_ns=deadline_ns,
                             selection_cycle_deadline_ns=cycle_deadline_ns,
                             selection_reserve_ns=reserve_ns)
        while True:
            data = self.imu.get_data()  # Cached snapshot; propagate reader errors.
            self.observation_imu = data
            now_ns = time.monotonic_ns()
            check_sample(data, self.imu_max_age_s, now_ns=now_ns)
            age_ns = now_ns - data['sample_start_monotonic_ns']
            polls += 1
            self.cycle_diagnostics.update(selection_polls=polls,
                selection_end_ns=now_ns, selection_age_ms=age_ns / 1e6,
                selection_sample_index=data.get('sample_index'))
            if now_ns > cycle_deadline_ns:
                raise ImuDataError('IMU: prazo do ciclo na selecao excedido')
            if age_ns <= int(self.imu_max_age_s * 1e9) - reserve_ns:
                # The wait deadline stops polling, not acceptance of a fresh
                # sample returned by the final poll. Still require the cycle
                # deadline, age reserve, runtime budget and final write checks.
                break
            if now_ns >= deadline_ns:
                raise ImuDataError('IMU: sem margem temporal antes da inferencia '
                                   f'({age_ns / 1e6:.1f} ms; reserva {reserve_ns / 1e6:.1f} ms)')
            time.sleep(min(0.0005, (deadline_ns - now_ns) / 1e9))

        obs = obs.copy()
        obs[:3] = data['gyro']
        obs[3:6] = data['accelero']
        if self.telemetry is not None:
            sensors = self.telemetry_observation
            sensors.update(
                imu_initial_sample_index=initial_index,
                imu_received_monotonic_ns=now_ns,
                imu_selected_monotonic_ns=now_ns,
                imu_selection_wait_ms=(now_ns - started_ns) / 1e6,
                imu_selection_polls=polls,
                imu_selection_wait_deadline_overrun_ms=max(0, (now_ns-deadline_ns)/1e6),
                imu_selection_cycle_remaining_ms=(cycle_deadline_ns-now_ns)/1e6,
                imu_reserve_ms=reserve_ns / 1e6,
                imu_sample_start_monotonic_ns=data['sample_start_monotonic_ns'],
                imu_sample_end_monotonic_ns=data['sample_end_monotonic_ns'],
                imu_sample_index=data.get('sample_index'),
                imu_age_ms=(now_ns - data['sample_end_monotonic_ns']) / 1e6,
                imu_oldest_age_ms=age_ns / 1e6,
                gyro_rad_s=np.array(data['gyro']).copy(),
                accelerometer_m_s2=np.array(data['accelero']).copy())
        return obs

    def start(self):
        # Constructor/controller setup can take seconds: validate again before
        # any motor configuration or initial-pose command.
        if self.runtime_budget is not None:
            self.runtime_budget.preflight()
        self.imu.wait_ready()
        kps = [self.pid[0]] * 14
        kds = [self.pid[2]] * 14

        # lower head kps
        kps[5:9] = [8, 8, 8, 8]

        self.hwi.set_kps(kps)
        self.hwi.set_kds(kds)
        self.hwi.turn_on()

        time.sleep(2)

    def _publish_control_status(self):
        if self.controller is not None and hasattr(self.controller, 'set_runtime_status'):
            self.controller.set_runtime_status(
                paused=self.paused, imu_fault=self.imu_fault,
                runtime_fault=self.runtime_fault,
                fault_acknowledged=self.imu_fault_acknowledged,
            )

    def _mark_imu_stage(self, stage, **times):
        if not hasattr(self, 'cycle_diagnostics'):
            self.cycle_diagnostics = {}
        self.cycle_diagnostics.update(times)
        self.cycle_diagnostics['stage'] = stage
        self.cycle_diagnostics[stage+'_ns'] = time.monotonic_ns()

    def _pause_for_imu(self, error):
        # Capture before restoring GC, printing or recording the pause.
        detected_ns = time.monotonic_ns()
        detail = dict(getattr(self, 'cycle_diagnostics', {}))
        sample = self.observation_imu
        used = ({key: sample.get(key) for key in (
            'sample_index', 'sample_start_monotonic_ns', 'sample_end_monotonic_ns')}
            if sample is not None else None)
        latest = self.imu.diagnostic_snapshot() if hasattr(self.imu, 'diagnostic_snapshot') else None
        writer = self.telemetry.writer_snapshot() if self.telemetry is not None and hasattr(self.telemetry, 'writer_snapshot') else None
        self.imu_fault = str(error)
        self.imu_fault_acknowledged = False
        self.paused = True
        if self.runtime_budget is not None:
            self.runtime_budget.pause()
        print('PAUSA IMU:', self.imu_fault, flush=True)
        if self.telemetry is not None:
            self.telemetry.record('imu_fault', message=self.imu_fault,
                                  detected_monotonic_ns=detected_ns, diagnostic=detail,
                                  used_sample=used, latest_reader=latest, writer=writer,
                                  commands=self.last_commands,
                                  response='pause_policy_hold_last_target')
        self._publish_control_status()

    def _pause_for_runtime(self, error):
        self.runtime_fault = str(error)
        self.imu_fault_acknowledged = False
        self.paused = True
        if self.runtime_budget is not None:
            self.runtime_budget.pause()
        if self.telemetry is not None:
            self.telemetry.record('runtime_guard_pause', message=self.runtime_fault,
                                  response='pause_policy_hold_last_target')
        print('PAUSA RUNTIME:', self.runtime_fault, flush=True)
        self._publish_control_status()

    def _check_runtime_budget(self):
        if self.runtime_budget is None:
            return True
        try:
            self.runtime_budget.check()
            return True
        except RuntimeBudgetError as exc:
            self._pause_for_runtime(exc)
            return False

    def _request_pause(self, desired):
        if desired:
            self.paused = True
            if self.runtime_budget is not None:
                self.runtime_budget.pause()
            if self.imu_fault or self.runtime_fault:
                self.imu_fault_acknowledged = True
        else:
            if not self.paused:
                return
            if (self.imu_fault or self.runtime_fault) and not self.imu_fault_acknowledged:
                return
            if (self.runtime_budget is not None or self.imu_fault) and np.any(np.abs(self.last_commands) > 1e-6):
                return
            try:
                if self.runtime_budget is not None:
                    self.runtime_budget.maintain()
                # GC maintenance can age IMU data; require fresh readings afterward.
                self.observation_imu = None
                self._mark_imu_stage('resume_imu_ready')
                self.imu.wait_ready(timeout_s=3 if self.runtime_budget is not None else 0)
                if self.runtime_budget is not None:
                    self.runtime_budget.begin()
            except RuntimeBudgetError as exc:
                self._pause_for_runtime(exc)
                return
            except ImuDataError as exc:
                self._pause_for_imu(exc)
                return
            if self.imu_fault and self.telemetry is not None:
                self.telemetry.record('imu_fault_cleared', response='explicit_resume')
            if self.runtime_fault and self.telemetry is not None:
                self.telemetry.record('runtime_guard_cleared', response='explicit_resume')
            self.imu_fault = None
            self.runtime_fault = None
            self.imu_fault_acknowledged = False
            self.paused = False
        self._publish_control_status()

    def get_phase_frequency_factor(self, x_velocity):

        max_phase_frequency = 1.2
        min_phase_frequency = 1.0

        # Perform linear interpolation
        freq = min_phase_frequency + (abs(x_velocity) / 0.15) * (
            max_phase_frequency - min_phase_frequency
        )

        return freq

    def run(self):
        i = 0
        previous_cycle_ns = None
        previous_paused = None
        attempt = 0
        try:
            self.start()
            print("Starting")
            start_t = time.time()
            while True:
                left_trigger = 0
                right_trigger = 0
                t = time.time()
                cycle_start_ns = time.monotonic_ns()
                cycle_dt_ms = (cycle_start_ns-previous_cycle_ns)/1e6 if previous_cycle_ns is not None else None
                previous_cycle_ns = cycle_start_ns
                self.cycle_diagnostics = dict(attempt=attempt, next_action_cycle=i,
                                              cycle_start_ns=cycle_start_ns, stage='commands')
                self.observation_imu = None
                attempt += 1

                if self.commands:
                    self.last_commands, self.buttons, left_trigger, right_trigger = (
                        self.controller.get_last_command()
                    )
                    if hasattr(self.controller, "consume_desired_paused"):
                        desired_paused = self.controller.consume_desired_paused()
                        if desired_paused is not None:
                            self._request_pause(desired_paused)
                            print("PAUSE" if self.paused else "UNPAUSE")
                    if self.buttons.dpad_up.triggered:
                        self.phase_frequency_factor_offset += 0.05
                        print(
                            f"Phase frequency factor offset {round(self.phase_frequency_factor_offset, 3)}"
                        )

                    if self.buttons.dpad_down.triggered:
                        self.phase_frequency_factor_offset -= 0.05
                        print(
                            f"Phase frequency factor offset {round(self.phase_frequency_factor_offset, 3)}"
                        )

                    if self.buttons.LB.is_pressed:
                        self.phase_frequency_factor = 1.3
                    else:
                        self.phase_frequency_factor = 1.0

                    if self.buttons.X.triggered:
                        if self.duck_config.projector:
                            self.projector.switch()

                    if self.buttons.B.triggered:
                        if self.duck_config.speaker:
                            self.sounds.play_random_sound()

                    if self.duck_config.antennas and not (self.imu_fault or self.runtime_fault):
                        self.antennas.set_position_left(right_trigger)
                        self.antennas.set_position_right(left_trigger)

                    if self.buttons.A.triggered:
                        desired = not self.paused
                        if (self.imu_fault or self.runtime_fault) and not self.imu_fault_acknowledged:
                            desired = True  # First A acknowledges; next A requests resume.
                        self._request_pause(desired)
                        if self.paused:
                            print("PAUSE")
                        else:
                            print("UNPAUSE")

                if self.telemetry is not None and self.paused != previous_paused:
                    self.telemetry.record('pause_changed', paused=self.paused)
                previous_paused = self.paused
                self._publish_control_status()
                if self.paused:
                    if self.runtime_budget is not None and self.runtime_budget.needs_maintenance:
                        try:
                            self.runtime_budget.maintain()
                        except RuntimeBudgetError as exc:
                            if self.runtime_fault != str(exc):
                                self._pause_for_runtime(exc)
                    if self.telemetry is not None:
                        self.telemetry.record('paused', commands=self.last_commands,
                                              command_source=getattr(self.controller, 'last_command_telemetry', None))
                    time.sleep(0.1)
                    continue

                self._mark_imu_stage('runtime_budget_before_observation')
                if not self._check_runtime_budget():
                    continue
                try:
                    obs = self.get_obs()
                except ImuDataError as exc:
                    self._pause_for_imu(exc)
                    continue
                if obs is None:
                    time.sleep(1 / self.control_freq)
                    continue
                try:
                    obs = self._select_imu_for_inference(obs)
                except ImuDataError as exc:
                    self._pause_for_imu(exc)
                    continue
                self._mark_imu_stage('runtime_budget_before_inference')
                if not self._check_runtime_budget():
                    continue
                observed_obs = obs.copy()

                if self.save_obs:
                    self.saved_obs.append(obs)

                if self.replay_obs is not None:
                    if i < len(self.replay_obs):
                        obs = self.replay_obs[i]
                    else:
                        print("BREAKING ")
                        break

                self._mark_imu_stage('inference')
                inference_start_ns = time.monotonic_ns()
                action = self.policy.infer(obs)
                inference_end_ns = time.monotonic_ns()
                self._mark_imu_stage('after_inference_used_sample', inference_start_ns=inference_start_ns,
                                     inference_end_ns=inference_end_ns)
                try:
                    check_sample(self.observation_imu, self.imu_max_age_s)
                    self._mark_imu_stage('after_inference_latest_sample')
                    self.imu.get_data()  # A concurrent reader failure must not be hidden.
                except ImuDataError as exc:
                    self._pause_for_imu(exc)
                    continue

                self._mark_imu_stage('prepare_targets')
                candidate_targets = self.init_pos + action * self.action_scale
                unfiltered_targets = candidate_targets.copy()

                # self.motor_targets = np.clip(
                #     self.motor_targets,
                #     self.prev_motor_targets
                #     - self.max_motor_velocity * (1 / self.control_freq),  # control dt
                #     self.prev_motor_targets
                #     + self.max_motor_velocity * (1 / self.control_freq),  # control dt
                # )

                if self.action_filter is not None:
                    filter_state = (self.action_filter.current_action, self.action_filter.last_action)
                    self.action_filter.push(candidate_targets)
                    filtered_motor_targets = self.action_filter.get_filtered_action()
                    if (
                        time.time() - start_t > 1
                    ):  # give time to the filter to stabilize
                        candidate_targets = filtered_motor_targets.copy()

                candidate_previous = candidate_targets.copy()

                head_motor_targets = self.last_commands[3:] + candidate_targets[5:9]
                candidate_targets[5:9] = head_motor_targets

                action_dict = make_action_dict(
                    candidate_targets, list(self.hwi.joints.keys())
                )

                self._mark_imu_stage('runtime_budget_before_write')
                if not self._check_runtime_budget():
                    if self.action_filter is not None:
                        self.action_filter.current_action, self.action_filter.last_action = filter_state
                    continue
                self._mark_imu_stage('before_motor_write')
                write_start_ns = time.monotonic_ns()
                try:
                    check_sample(self.observation_imu, self.imu_max_age_s, now_ns=write_start_ns)
                except ImuDataError as exc:
                    if self.action_filter is not None:
                        self.action_filter.current_action, self.action_filter.last_action = filter_state
                    self._pause_for_imu(exc)
                    continue
                try:
                    self.hwi.set_position_all(action_dict)
                except Exception as exc:
                    if self.telemetry is not None:
                        self.telemetry.record('motor_write_failure', cycle=i, message=str(exc),
                                              motor_targets_rad=candidate_targets)
                    raise
                write_end_ns = time.monotonic_ns()
                # Only executed actions enter the next observation's history.
                self.last_last_last_action = self.last_last_action.copy()
                self.last_last_action = self.last_action.copy()
                self.last_action = action.copy()
                self.motor_targets = candidate_targets
                self.prev_motor_targets = candidate_previous
                self.imitation_i = (self.imitation_i + 1 * (self.phase_frequency_factor
                                    + self.phase_frequency_factor_offset)) % self.PRM.nb_steps_in_period
                angle = self.imitation_i / self.PRM.nb_steps_in_period * 2 * np.pi
                self.imitation_phase = np.array([np.cos(angle), np.sin(angle)])

                if self.telemetry is not None:
                    self.telemetry.record(
                        'cycle', cycle=i, cycle_start_monotonic_ns=cycle_start_ns,
                        previous_cycle_dt_ms=cycle_dt_ms, commands=self.last_commands,
                        command_source=getattr(self.controller, 'last_command_telemetry', None),
                        sensors=self.telemetry_observation, observed_obs=observed_obs,
                        policy_obs=obs, action=action, unfiltered_targets_rad=unfiltered_targets,
                        motor_targets_rad=self.motor_targets,
                        servo_goal_rad=[action_dict[n]+self.hwi.joints_offsets[n] for n in self.hwi.joints],
                        inference_start_monotonic_ns=inference_start_ns,
                        inference_end_monotonic_ns=inference_end_ns,
                        inference_ms=(inference_end_ns-inference_start_ns)/1e6,
                        motor_write_start_monotonic_ns=write_start_ns,
                        motor_write_end_monotonic_ns=write_end_ns,
                        motor_write_ms=(write_end_ns-write_start_ns)/1e6,
                        loop_work_before_logging_ms=(write_end_ns-cycle_start_ns)/1e6,
                        phase_frequency=self.phase_frequency_factor+self.phase_frequency_factor_offset,
                    )

                i += 1

                took = time.time() - t
                # print("Full loop took", took, "fps : ", np.around(1 / took, 2))
                if (1 / self.control_freq - took) < 0:
                    print(
                        "Policy control budget exceeded by",
                        np.around(took - 1 / self.control_freq, 3),
                    )
                time.sleep(max(0, 1 / self.control_freq - took))

        except KeyboardInterrupt:
            if self.telemetry is not None:
                self.telemetry.record('control_interrupted', cycle=i)
        finally:
            try:
                self.imu.stop()
            finally:
                if self.runtime_budget is not None:
                    self.runtime_budget.close()
            self.paused = True
            self._publish_control_status()
            if self.duck_config.antennas:
                self.antennas.stop()
            if self.duck_config.eyes:
                self.eyes.stop()
            if self.duck_config.projector:
                self.projector.stop()
            self.feet_contacts.stop()

        if self.save_obs:
            pickle.dump(self.saved_obs, open("robot_saved_obs.pkl", "wb"))
        print("Controle encerrado; nenhum novo alvo sera enviado. Torque nao foi alterado.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument(
        "--duck_config_path",
        type=str,
        required=False,
        default=f"{HOME_DIR}/duck_config.json",
    )
    parser.add_argument("-a", "--action_scale", type=float, default=0.25)
    parser.add_argument("-p", type=int, default=30)
    parser.add_argument("-i", type=int, default=0)
    parser.add_argument("-d", type=int, default=0)
    parser.add_argument("-c", "--control_freq", type=int, default=50)
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument(
        "--commands",
        action="store_true",
        default=True,
        help="enable the selected external command source",
    )
    parser.add_argument(
        "--control-source", choices=("xbox", "web"), default="xbox",
        help="command input used by the walking policy",
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--web-token", default=None)
    parser.add_argument("--web-command-timeout", type=float, default=0.6)
    parser.add_argument("--allow-head-control", action="store_true")
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=10)
    parser.add_argument(
        "--save_obs",
        type=str,
        required=False,
        default=False,
        help="save the run's observations",
    )
    parser.add_argument(
        "--replay_obs",
        type=str,
        required=False,
        default=None,
        help="replay the observations from a previous run (can be from the robot or from mujoco)",
    )
    parser.add_argument("--cutoff_frequency", type=float, default=None)
    parser.add_argument('--telemetry-dir', help='Directory for a new, unique telemetry session')
    parser.add_argument('--telemetry-label', default='test', help='Example: student-forward-floor-a')
    parser.add_argument('--imu-i2c-bus', '--i2c-bus', type=int, default=8,
                        help='Linux IMU bus (default 8); no fallback. Use 1 only after restoring hardware I2C.')
    parser.add_argument('--imu-max-age-ms', type=float, default=50,
                        help='Pause policy when IMU age from acquisition start exceeds this limit')
    parser.add_argument('--start-paused', action='store_true', default=None,
                        help='Wait for an explicit start command after initial pose setup')

    parser.add_argument('--runtime-gc', choices=('bounded', 'default'), default='bounded',
                        help='Bounded active windows with GC during pauses (default); default restores legacy GC')
    parser.add_argument('--active-window-s', type=float, default=30,
                        help='Maximum continuous active time, in (0,120]; default 30 seconds')
    parser.add_argument('--max-rss-growth-mb', type=float, default=64)
    parser.add_argument('--min-available-mb', type=float, default=64)
    parser.add_argument('--telemetry-writer-yield-ms', type=float, default=1)
    args = parser.parse_args()
    # Validate resources/settings before constructing the hardware runtime.
    budget = (RuntimeBudget(args.active_window_s, args.max_rss_growth_mb, args.min_available_mb)
              if args.runtime_gc == 'bounded' else None)
    pid = [args.p, args.i, args.d]

    print("Done parsing args")
    recorder = nullcontext(None)
    if args.telemetry_dir:
        recorder = TelemetryRecorder(args.telemetry_dir, label=args.telemetry_label,
                                     writer_yield_ms=args.telemetry_writer_yield_ms, metadata={
            'runtime_gc': args.runtime_gc, 'active_window_s': args.active_window_s,
            'imu_fault_diagnostics_version': 1,
            'imu_selection_version': 2,
            'imu_selection_max_wait_ms': 5,
            'imu_selection_reserve_ms': min(10, args.imu_max_age_ms / 2),
            'max_rss_growth_mb': args.max_rss_growth_mb, 'min_available_mb': args.min_available_mb,
            'model_name': Path(args.onnx_model_path).name,
            'model_sha256': file_sha256(args.onnx_model_path),
            'runtime_sha256': file_sha256(__file__),
            'config_sha256': file_sha256(args.duck_config_path),
            'control_source': args.control_source, 'control_freq_hz': args.control_freq,
            'action_scale': args.action_scale, 'pid_requested': pid,
            'pitch_bias_requested_deg': args.pitch_bias,
            'cutoff_frequency_hz': args.cutoff_frequency,
            'web_command_timeout_s': args.web_command_timeout,
            'imu_i2c_bus': args.imu_i2c_bus, 'imu_max_age_ms': args.imu_max_age_ms,
            'source_sha256': {
                name: file_sha256(Path(__file__).resolve().parents[1]/'mini_bdx_runtime'/'mini_bdx_runtime'/name)
                for name in ('raw_imu.py', 'imu_safety.py', 'web_controller.py', 'telemetry.py', 'runtime_budget.py', 'rustypot_position_hwi.py')
            },
        })
    with recorder as telemetry:
        if budget is not None:
            budget.recorder = telemetry
        rl_walk = RLWalk(
            args.onnx_model_path,
            duck_config_path=args.duck_config_path,
            action_scale=args.action_scale,
            pid=pid,
            control_freq=args.control_freq,
            commands=args.commands,
            control_source=args.control_source,
            serial_port=args.serial_port,
            web_host=args.web_host,
            web_port=args.web_port,
            web_token=args.web_token,
            web_command_timeout=args.web_command_timeout,
            allow_head_control=args.allow_head_control,
            camera=args.camera,
            camera_size=(args.camera_width, args.camera_height),
            camera_fps=args.camera_fps,
            pitch_bias=args.pitch_bias,
            save_obs=args.save_obs,
            replay_obs=args.replay_obs,
            cutoff_frequency=args.cutoff_frequency,
            telemetry=telemetry,
            imu_i2c_bus=args.imu_i2c_bus,
            imu_max_age_ms=args.imu_max_age_ms,
            start_paused=args.start_paused,
            runtime_budget=budget,
        )
        print("Done instantiating RLWalk")
        rl_walk.run()
