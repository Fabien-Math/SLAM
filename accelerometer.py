import numpy as np

class Accmeter:
    def __init__(self, acc_prec, ang_acc_prec) -> None:
        self.acc_prec = acc_prec
        self.ang_acc_prec = ang_acc_prec
        self.old_speed = 0
        self.old_rot_speed = 0
        self.last_scan_time = 0

    def get_acc(self, robot_speed, robot_rot_speed, t):
        dv = robot_speed - self.old_speed
        drot = robot_rot_speed - self.old_rot_speed
        dt = t - self.last_scan_time
        if dt != 0:
            acc = (dv / dt) * (1 + (self.acc_prec/100)*(np.random.rand() - 0.5))
            rot_acc = (drot / dt) * (1 + (self.ang_acc_prec/100)*(np.random.rand() - 0.5))

            self.old_speed = robot_speed
            self.old_rot_speed = robot_rot_speed
            self.last_scan_time = t
        return acc, rot_acc