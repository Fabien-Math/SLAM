from robot import BeaconRobot

class Accmeter:
    def __init__(self, prec) -> None:
        self.prec = prec
        self.old_speed = 0

    def get_acc(self, robot:BeaconRobot, dt):
        dv = robot.speed - self.old_speed
        acc = dv * dt
        return acc