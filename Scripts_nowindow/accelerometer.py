import util as ut
import numpy as np

class Accmeter:
    def __init__(self, precision, time) -> None:
        # (Acc, Angular acc) precisions
        self.prec = np.array(precision)
        self.old_speed = 0
        self.old_rot_speed = 0
        self.last_scan_time = time

    def get_acc(self, robot_speed, robot_rot_speed, t):
        dv = robot_speed - self.old_speed
        drot = robot_rot_speed - self.old_rot_speed
        dt = t - self.last_scan_time
        if dt:
            acc_true, ang_acc_true = dv / dt, drot / dt
            # print(robot_speed, robot_rot_speed, dv, drot, dt)
            acc, ang_acc = add_uncertainty(acc_true, ang_acc_true, self.prec)

            self.old_speed = robot_speed
            self.old_rot_speed = robot_rot_speed
            self.last_scan_time = t
        else: return 0, 0

        return acc, ang_acc

def add_uncertainty(acc, ang_acc, sigma):
	mean = np.array([acc, ang_acc])
	covariance = np.diag(sigma ** 2)
	acc, ang_acc = ut.normal_distribution(mean, covariance)
	return acc, ang_acc
