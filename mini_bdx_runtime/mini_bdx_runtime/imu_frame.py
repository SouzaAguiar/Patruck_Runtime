"""Pure frame rotation AFTER the existing BNO055 remap; no hardware access."""
import numpy as np

IMU_FRAMES = ('native', 'yaw-plus-90', 'yaw-minus-90')


def rotate_vector(vector, frame='native'):
    if frame not in IMU_FRAMES:
        raise ValueError('Unknown IMU frame: ' + str(frame))
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('IMU vector must contain three finite values')
    if frame == 'native':
        return value.copy()
    if frame == 'yaw-plus-90':
        return np.array([-value[1], value[0], value[2]])
    return np.array([value[1], -value[0], value[2]])


def transform_sample(sample, frame='native'):
    if 'imu_frame' in sample:
        raise ValueError('IMU sample already has a frame; refusing double rotation')
    return {**sample, 'imu_frame': frame,
            'gyro_native_rad_s': tuple(sample['gyro']),
            'accel_native_m_s2': tuple(sample['accelero']),
            'gyro': rotate_vector(sample['gyro'], frame),
            'accelero': rotate_vector(sample['accelero'], frame)}
