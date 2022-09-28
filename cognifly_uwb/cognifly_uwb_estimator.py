from threading import Lock
import time
from multiprocessing import Process, Pipe
import socket
import struct
import pickle
import math
import socket

from cognifly.cognifly_controller.cognifly_controller import CogniflyController, PoseEstimator
from cognifly.utils.functions import smallest_angle_diff_rad
from cognifly.utils.filters import Simple1DKalman, Simple1DExponentialAverage

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)
from pozyx import ReadyToLocalize


class PoseEstimatorOptitrack(PoseEstimator):
    def __init__(self):

        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()

        remote_id = None  # remote device network ID

        # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
        anchors = [DeviceCoordinates(0x0022, 1, Coordinates(0, 0, 2790)),
                   DeviceCoordinates(0x0024, 1, Coordinates(10490, 0, 2790)),
                   DeviceCoordinates(0x0026, 1, Coordinates(-405, 6000, 2790)),
                   DeviceCoordinates(0x0028, 1, Coordinates(10490, 6500, 2790))]

        # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
        algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
        dimension = PozyxConstants.DIMENSION_3D
        # height of device, required in 2.5D positioning
        height = 1000

        pozyx = PozyxSerial(serial_port)
        self.r = ReadyToLocalize(pozyx, None, anchors, algorithm, dimension, height, remote_id)
        self.r.setup()

        self.vx, self.vy, self.vz, self.w = 0, 0, 0, 0
        self.x, self.y, self.z, self.yaw = 0, 0, 0, 0
        self.filter_vx = Simple1DExponentialAverage(tau=0.2)
        self.filter_vy = Simple1DExponentialAverage(tau=0.2)
        self.filter_vz = Simple1DExponentialAverage(tau=0.2)
        self.filter_w = Simple1DExponentialAverage(tau=0.2)
        print("waiting for first uwb estimate...")
        self.ts = time.monotonic()
        self.ts_ot = None
        self.ts_last_recv_ot = self.ts

        self.x, self.y = None, None

        while self.x is None or self.y is None:
            self.x, self.y, _ = self.r.get_position()
            if self.x is None or self.y is None:
                time.sleep(0.1)
        print("first estimate received!")

    def __del__(self):
        self.close()

    def get(self):
        est_x, est_y, est_z, est_yaw, est_vx, est_vy, est_vz, est_w = self.get_fc_estimates()

        t = time.monotonic()
        step = t - self.ts
        self.ts = t

        if t - self.ts_last_recv_ot > 0.5:
            return (None, None, None, None, None, None, None, None)

        x, y, _ = self.r.get_position()
        self.z, self.yaw, self.vz, self.w = est_z, est_yaw, est_vz, est_w

        y = -y  # aerospace convention

        vx = (x - self.x) / step
        vy = (y - self.y) / step

        self.x, self.y = x, y
        self.vx = self.filter_vx.update_estimate(vx)
        self.vy = self.filter_vy.update_estimate(vy)
        self.ts_last_recv_ot = t

        return self.x, self.y, self.z, self.yaw, self.vx, self.vy, self.vz, self.w

    def close(self):
        pass


if __name__ == '__main__':
    print("creating the uwb estimator...")
    pe = PoseEstimatorOptitrack()
    print("uwb estimator ready!")
    cc = CogniflyController(print_screen=True,
                            pose_estimator=pe,
                            trace_logs=True,
                            vel_x_kp=250.0,
                            vel_x_ki=100.0,
                            vel_x_kd=10.0,
                            vel_y_kp=250.0,
                            vel_y_ki=100.0,
                            vel_y_kd=10.0,
                            vel_z_kp=5.0,
                            vel_z_ki=5.0,
                            vel_z_kd=0.1,
                            vel_w_kp=100.0,
                            vel_w_ki=75.0,
                            vel_w_kd=0.0,
                            pid_limit=200)
    cc.run_curses()
    pe.close()

