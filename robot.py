from lidar import LIDAR
from util import Vector2, distance, find_circle_intersection, find_n_nearest, get_angle
from accelerometer import Accmeter
import pygame
from math import cos, sin ,pi
import numpy as np
from scipy.sparse import lil_matrix


class BeaconRobot:
	def __init__(self, pos, acc, rot_acc, max_forward_speed, max_backward_speed, max_rot_speed) -> None:
		# Position
		self.pos = Vector2(pos[0], pos[1])
		self.pos_calc = Vector2(pos[0], pos[1])
		self.pos_calc_lidar = Vector2(pos[0], pos[1])
		self.rot = 0
		self.rot_calc = 0
		
		# Acceleration
		self.acc = acc
		self.rot_acc = rot_acc
		self.acc_calc = 0
		self.rot_acc_calc = 0
		self.decc = 100
		self.rot_decc = 1000

		# Speed
		self.speed = 0
		self.speed_calc = 0
		self.max_forward_speed = max_forward_speed
		self.max_backward_speed = max_backward_speed
		self.rot_speed = 0
		self.rot_speed_calc = 0
		self.max_rot_speed = max_rot_speed


		self.radius = 10
		self.color = (0, 200, 255)
		self.map = []
		self.live_grid_map_centre = self.pos.copy()
		self.live_grid_map_size = 201
		self.live_grid_map = np.zeros((self.live_grid_map_size, self.live_grid_map_size))
		self.saved_grid_map = None

		# Sensors
		self.lidar = None
		self.accmeter = None

	def move(self, dir, dt):
		if dir == 1:
			if self.speed >= 0:
				self.speed = min(self.speed + self.acc*dt, self.max_forward_speed)
			else:
				self.speed = max(self.speed + self.decc*dt, 0)
		elif dir == -1:
			if self.speed <= 0:
				self.speed = max(self.speed - self.acc*dt, self.max_backward_speed)
			else:
				self.speed = min(self.speed + self.decc*dt, 0)
		else:
			if self.speed >= 0:
				self.speed = max(self.speed - self.decc*dt, 0)
			else:
				self.speed = min(self.speed + self.decc*dt, 0)

		x = self.speed * cos(self.rot * pi / 180) * dt
		y = self.speed * sin(self.rot * pi / 180) * dt

		self.pos.add(x, y)


	def rotate(self, dir, dt):
		if dir == 1:
			if self.rot_speed >= 0:
				self.rot_speed = min(self.rot_speed + self.rot_acc*dt, self.max_rot_speed)
			else:
				self.rot_speed = max(self.rot_speed + self.rot_decc*dt, 0)
		elif dir == -1:
			if self.rot_speed <= 0:
				self.rot_speed = max(self.rot_speed - self.rot_acc*dt, -self.max_rot_speed)
			else:
				self.rot_speed = min(self.rot_speed + self.rot_decc*dt, 0)
		else:
			if self.rot_speed >= 0:
				self.rot_speed = max(self.rot_speed - self.rot_decc*dt, 0)
			else:
				self.rot_speed = min(self.rot_speed + self.rot_decc*dt, 0)
		self.rot += self.rot_speed * dt


	def draw(self, window):
		pygame.draw.circle(window, self.color, (self.pos.x, self.pos.y), self.radius)
		pygame.draw.line(window, (255,255,255), (self.pos.x, self.pos.y), (self.pos.x + self.radius * cos(self.rot * pi / 180), self.pos.y  + self.radius * sin(self.rot * pi / 180)), 2)
		pygame.draw.circle(window, (100,100,100), (self.pos_calc.x, self.pos_calc.y), 0.8*self.radius)
		pygame.draw.line(window, (255,255,255), (self.pos_calc.x, self.pos_calc.y), (self.pos_calc.x + self.radius * cos(self.rot_calc * pi / 180), self.pos_calc.y  + self.radius * sin(self.rot_calc * pi / 180)), 2)
		pygame.draw.circle(window, (200,100,100), (self.pos_calc_lidar.x, self.pos_calc_lidar.y), 0.5*self.radius)
	

	def draw_known_map(self, window):
		for dot in self.map:
			pygame.draw.circle(window, (255, 255, 255), (dot[0], dot[1]), 1)


	def draw_live_grid_map(self, window):
		for idy, line in enumerate(self.live_grid_map):
			for idx, value in enumerate(line):
				if value == 0:
					continue
				if value == -1:
					color = (70, 255, 200)
				else:
					gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (gray_scale, gray_scale, gray_scale)
				offset_id = self.live_grid_map_size//2
				top_left_x = (idx - offset_id) * self.radius + self.live_grid_map_centre.y
				top_left_y = (idy - offset_id) * self.radius + self.live_grid_map_centre.x
				rect = pygame.Rect(top_left_y, top_left_x, self.radius, self.radius)
				pygame.draw.rect(window, color, rect)


	def update_live_grid_map(self, points):
		# Robot pos
		idx = int((self.pos_calc.x - self.live_grid_map_centre.x)/self.radius + self.live_grid_map_size//2)
		idy = int((self.pos_calc.y - self.live_grid_map_centre.y)/self.radius + self.live_grid_map_size//2)
		if 0 < idx < self.live_grid_map_size and 0 < idy < self.live_grid_map_size:
			self.live_grid_map[idx, idy] = -1

		if points is None:
			return
		# Lidar points pos
		for point in points:
			idx = int((point[0] - self.live_grid_map_centre.x)/self.radius + self.live_grid_map_size//2)
			idy = int((point[1] - self.live_grid_map_centre.y)/self.radius + self.live_grid_map_size//2)
			if 0 < idx < self.live_grid_map_size and 0 < idy < self.live_grid_map_size:
				self.live_grid_map[idx, idy] = max(self.live_grid_map[idx, idy], 1 - distance(self.pos_calc, point)/500)
			else:
				print("couucccou")


	def equip_accmeter(self, prec):
		self.accmeter = Accmeter(prec)

	def compute_pos_calc(self, t):
		last_time_scan = self.accmeter.last_scan_time
		self.acc_calc, self.rot_acc_calc = self.accmeter.get_acc(self.speed, self.rot_speed, t)

		self.speed_calc += self.acc_calc * (t - last_time_scan)

		self.rot_speed_calc += self.rot_acc_calc * (t - last_time_scan)
		self.rot_calc += self.rot_speed_calc * (t - last_time_scan)
		self.rot_calc %= 360

		x = self.speed_calc * (t - last_time_scan) * cos(self.rot_calc * pi / 180)
		y = self.speed_calc * (t - last_time_scan) * sin(self.rot_calc * pi / 180)
		self.pos_calc.add(x, y)


	def equip_lidar(self, fov, freq, res, prec):
		self.lidar = LIDAR(fov, freq, res, prec)
		self.lidar.pos = self.pos
		
	def scan_environment(self, time, map, window):
		if self.lidar is None:
			print("No lidar equiped !")
			return
		
		points = self.lidar.scan_environment(time, map, self.radius, window)

		# Compute position with lidar data
		if points is not None:
			self.update_live_grid_map(points)
			pos_lidar = self.lidar.correct_pos_with_lidar(points)
			self.map += points
			if pos_lidar is not None:
				self.pos_calc_lidar = pos_lidar
				self.pos_calc = self.pos_calc_lidar.copy()


		
		return True
	
	