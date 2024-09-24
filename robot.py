from lidar import LIDAR
from util import Vector2
from accelerometer import Accmeter
import pygame

class BeaconRobot:
	def __init__(self, pos, acc) -> None:
		self.pos = Vector2(pos[0], pos[1])
		self.pos_calc = Vector2(pos[0], pos[1])
		self.acc = acc
		self.speed = 50
		self.radius = 5
		self.color = (0, 200, 255)
		self.map = []
		self.lidar = None
		self.accmeter = None

	def move(self, x, y):
		self.pos.add(x,y)

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
	