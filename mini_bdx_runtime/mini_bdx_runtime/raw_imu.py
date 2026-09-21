import numpy as np
import os
import pickle

from threading import Thread, Event
import time
from mini_bdx_runtime.imu_safety import CheckedModeDevice, ImuDataError, LatestImuSample, open_i2c


class Imu:
    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True,
        i2c_bus=None, max_age_s=.05,
    ):
        self.sampling_freq = sampling_freq
        self.calibrate = calibrate
        if not np.isfinite(sampling_freq) or sampling_freq <= 0:
            raise ValueError('IMU sampling frequency must be positive')
        self.samples = LatestImuSample(max_age_s)
        self.max_age_s = max_age_s
        self.i2c_bus = i2c_bus
        self._stop_event = Event()
        import adafruit_bno055

        self.bus = open_i2c(i2c_bus)
        try:
            self.imu = adafruit_bno055.BNO055_I2C(self.bus,address=0x29)
            self._configure_sensor(upside_down)
            self._thread = Thread(target=self.imu_worker, daemon=True)
            self._thread.start()
        except BaseException:
            self.bus.deinit()
            raise

    def _configure_sensor(self, upside_down):
        import adafruit_bno055
        # self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        # self.imu.mode = adafruit_bno055.ACCGYRO_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        self.imu.mode = adafruit_bno055.NDOF_MODE
        # self.imu.mode = adafruit_bno055.NDOF_FMC_OFF_MODE

        if upside_down:
            self.imu.axis_remap = (
                adafruit_bno055.AXIS_REMAP_Y,
                adafruit_bno055.AXIS_REMAP_X,
                adafruit_bno055.AXIS_REMAP_Z,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
            )

        else:
            self.imu.axis_remap = (
                adafruit_bno055.AXIS_REMAP_Y,
                adafruit_bno055.AXIS_REMAP_X,
                adafruit_bno055.AXIS_REMAP_Z,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_POSITIVE,
                adafruit_bno055.AXIS_REMAP_POSITIVE,
            )

        if self.calibrate:
            self.imu.mode = adafruit_bno055.NDOF_MODE
            calibrated = self.imu.calibrated
            while not calibrated:
                print("Calibration status: ", self.imu.calibration_status)
                print("Calibrated : ", self.imu.calibrated)
                calibrated = self.imu.calibrated
                time.sleep(0.1)
            print("CALIBRATION DONE")
            offsets_accelerometer = self.imu.offsets_accelerometer
            offsets_gyroscope = self.imu.offsets_gyroscope
            offsets_magnetometer = self.imu.offsets_magnetometer

            imu_calib_data = {
                "offsets_accelerometer": offsets_accelerometer,
                "offsets_gyroscope": offsets_gyroscope,
                "offsets_magnetometer": offsets_magnetometer,
            }
            for k, v in imu_calib_data.items():
                print(k, v)

            pickle.dump(imu_calib_data, open("imu_calib_data.pkl", "wb"))

            print("Saved", "imu_calib_data.pkl")
            exit()

        if os.path.exists("imu_calib_data.pkl"):
            imu_calib_data = pickle.load(open("imu_calib_data.pkl", "rb"))
            self.imu.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.1)
            self.imu.offsets_accelerometer = imu_calib_data["offsets_accelerometer"]
            self.imu.offsets_gyroscope = imu_calib_data["offsets_gyroscope"]
            self.imu.offsets_magnetometer = imu_calib_data["offsets_magnetometer"]
            self.imu.mode = adafruit_bno055.NDOF_MODE
            time.sleep(0.1)
        else:
            print("imu_calib_data.pkl not found")
            print("Imu is running uncalibrated")

        self.x_offset = 0

        # self.tare_x()

        for register, expected in ((0x00, 0xA0), (0x07, 0), (0x3D, 0x0C)):
            actual = self.imu._read_register(register)
            if actual != expected:
                raise ImuDataError(f'IMU: registrador 0x{register:02x}=0x{actual:02x}, esperado 0x{expected:02x}')
        if self.imu._read_register(0x3B) & 3:
            raise ImuDataError('IMU: unidades incompativeis com o driver')
        self.imu.i2c_device = CheckedModeDevice(self.imu.i2c_device)

    def tare_x(self):
        print("Taring x ...")
        x_values = []
        num_values = 100
        ok = False
        while not ok:
            x_values.append(np.array(self.imu.acceleration)[0])

            x_values = x_values[-num_values:]

            if len(x_values) == num_values:
                mean = np.mean(x_values)
                std = np.std(x_values)
                if std < 0.05:
                    ok = True
                    self.x_offset = mean
                    print("Tare x done")
                else:
                    print(std)

            time.sleep(0.01)

    def imu_worker(self):
        sample_index = 0
        try:
            while not self._stop_event.is_set():
                started = time.monotonic()
                sample_start_ns = time.monotonic_ns()
                try:
                    gyro = np.asarray(self.imu.gyro, dtype=float)
                    accelero = np.asarray(self.imu.acceleration, dtype=float)
                    if accelero.shape == (3,):
                        accelero = accelero.copy()
                        accelero[0] -= self.x_offset
                    self.samples.publish({
                        'gyro': gyro, 'accelero': accelero,
                        'sample_start_monotonic_ns': sample_start_ns,
                        'sample_end_monotonic_ns': time.monotonic_ns(),
                        'sample_index': sample_index,
                    })
                except Exception as exc:
                    self.samples.fail(f'{type(exc).__name__}: {exc}')
                sample_index += 1
                self._stop_event.wait(max(0, 1/self.sampling_freq-(time.monotonic()-started)))
        finally:
            self.samples.fail('IMU: leitura encerrada')
            self.bus.deinit()

    def get_data(self):
        return self.samples.get()

    def wait_ready(self, timeout_s=3):
        return self.samples.wait_ready(timeout_s)

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1)


if __name__ == "__main__":
    imu = Imu(50, upside_down=False)
    while True:
        data = imu.get_data()
        # print(data)
        print("gyro", np.around(data["gyro"], 3))
        print("accelero", np.around(data["accelero"], 3))
        print("---")
        time.sleep(1 / 25)
