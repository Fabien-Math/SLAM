from lidar import LIDAR
from util import Vector2
from accelerometer import Accmeter
import pygame


class BeaconRobot:
	def __init__(self, pos, acc, max_speed) -> None:
		self.pos = Vector2(pos[0], pos[1])
		self.pos_calc = Vector2(pos[0], pos[1])
		self.acc = acc
		self.decc = 100
		self.speed = 0
		self.last_dir = [0, 0, 0, 0]
		self.max_speed = max_speed
		self.radius = 5
		self.color = (0, 200, 255)
		self.map = []
		self.lidar = None
		self.accmeter = None

	def move(self, dir, dt):

		if dir == [0, 0, 0, 0]:
			self.speed = max(self.speed - self.decc*dt, 0)
			dir = self.last_dir
		else:
			self.speed = min(self.speed + self.acc*dt, self.max_speed)
			self.last_dir = dir

		if dir[0]:
			self.pos.add(-self.speed*dt,0)
		if dir[1]:
			self.pos.add(self.speed*dt,0)
		if dir[2]:
			self.pos.add(0, -self.speed*dt)
		if dir[3]:
			self.pos.add(0, self.speed*dt)

	def draw(self, window):
		pygame.draw.circle(window, self.color, (self.pos.x, self.pos.y), 10)

	def draw_known_map(self, window):
		for dot in self.map:
			pygame.draw.circle(window, (255, 255, 255), (dot[0], dot[1]), 1)

	def equip_accmeter(self):
		self.accmeter = Accmeter(prec=3)


	def equip_lidar(self, fov, freq, res, prec):
		self.lidar = LIDAR(fov, freq, res, prec)
		self.lidar.pos = self.pos
		
	def scan_environment(self, time, obstacles):
		if self.lidar is None:
			print("No lidar equiped !")
			return
		
		points = self.lidar.scan_environment(time, obstacles)
		if points is not None:
			self.map += points
	