from lidar import LIDAR
from util import Vector2, distance
from accelerometer import Accmeter
import pygame
from math import cos, sin ,pi
import numpy as np
from controller import Controller


class BeaconRobot:
	def __init__(self, pos:Vector2, acc:float, rot_acc:float, max_forward_speed:float, max_backward_speed:float, max_rot_speed:float) -> None:
		# Position and rotation
		self.pos = Vector2(pos[0], pos[1])
		self.pos_calc = Vector2(pos[0], pos[1])
		self.pos_calc_lidar = Vector2(pos[0], pos[1])
		self.rot = 0
		self.rot_calc = 0
		
		# Acceleration and decceleration
		self.acc = acc
		self.rot_acc = rot_acc
		self.acc_calc = 0
		self.rot_acc_calc = 0
		self.decc = 100
		self.rot_decc = 1000

		# Speed and angular speed
		self.speed = 0
		self.speed_calc = 0
		self.max_forward_speed = max_forward_speed
		self.max_backward_speed = max_backward_speed
		self.rot_speed = 0
		self.rot_speed_calc = 0
		self.max_rot_speed = max_rot_speed

		# Robot attribute
		self.radius = 10
		self.color = (0, 200, 255)

		# Robot map
		self.map = []
		self.live_grid_map_centre = self.pos.copy()
		self.live_grid_map_list_size = 201
		self.live_grid_map_size = 10
		self.live_grid_map = np.zeros((self.live_grid_map_list_size, self.live_grid_map_list_size))
		self.saved_grid_map = None

		# Sensors
		self.lidar = None
		self.accmeter = None
		self.controller = Controller(self, 0)

	def move(self, dir:int, dt:float):
		"""Move the robot

		Args:
			dir (int): Direction to go
			dt (float): Time step
		"""
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

		# Compute traveled distance
		x = self.speed * cos(self.rot * pi / 180) * dt
		y = self.speed * sin(self.rot * pi / 180) * dt
		
		# Update robot position
		self.pos.add(x, y)


	def rotate(self, dir:int, dt:float):
		"""Rotate the robot

		Args:
			dir (int): Direction to rotate
			dt (float): Time step
		"""
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
		
		# Update rotation
		self.rot += self.rot_speed * dt

	def live_grid_map_coord_to_ids(self, point):
		idx = int((point[0] - self.live_grid_map_centre.x)/self.live_grid_map_size + self.live_grid_map_list_size//2)
		idy = int((point[1] - self.live_grid_map_centre.y)/self.live_grid_map_size + self.live_grid_map_list_size//2)
		return idx, idy

	def live_grid_map_ids_to_rect(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		if 0 <= idx < self.live_grid_map_list_size and 0 <= idy < self.live_grid_map_list_size:
			offset_id = self.live_grid_map_list_size//2
			p1x = (idx - offset_id) * self.live_grid_map_size + self.live_grid_map_centre.x
			p1y = (idy - offset_id) * self.live_grid_map_size + self.live_grid_map_centre.y
			p2x = (idx + 1 - offset_id) * self.live_grid_map_size + self.live_grid_map_centre.x
			p2y = (idy + 1 - offset_id) * self.live_grid_map_size + self.live_grid_map_centre.y

		return p1x, p1y, p2x, p2y 
	
	def update_live_grid_map(self, points):
		"""Update the live grid of the robot

		Args:
			points (list): List of points to be added, if None, update only robot pos
		"""
		# Add robot pos
		idx, idy = self.live_grid_map_coord_to_ids(self.pos_calc.to_tuple())
		# If ids are in the range of the live map
		if 0 < idx < self.live_grid_map_list_size and 0 < idy < self.live_grid_map_list_size:
			# Set to safe path
			self.live_grid_map[idy, idx] = -1

		if points is None:
			return
		
		# Add lidar points pos
		for point in points:
			idx, idy = self.live_grid_map_coord_to_ids(point)
			# If ids are in the range of the live map
			if 0 < idx < self.live_grid_map_list_size and 0 < idy < self.live_grid_map_list_size:
				self.live_grid_map[idy, idx] = max(self.live_grid_map[idy, idx], 1 - distance(self.pos_calc, point)/self.lidar.max_dist)


	def equip_accmeter(self, acc_prec:float, ang_acc_prec:float):
		"""Equip accelerometer to the robot

		Args:
			prec (float): Precision of the accelerometer in % (0 to 100)
		"""
		self.accmeter = Accmeter(acc_prec, ang_acc_prec)


	def compute_pos_calc(self, t:float):
		"""Estimate position with accelerometer data

		Args:
			t (float): Current time
		"""
		last_time_scan = self.accmeter.last_scan_time
		self.acc_calc, self.rot_acc_calc = self.accmeter.get_acc(self.speed, self.rot_speed, t)

		self.speed_calc += self.acc_calc * (t - last_time_scan)

		self.rot_speed_calc += self.rot_acc_calc * (t - last_time_scan)
		self.rot_calc += self.rot_speed_calc * (t - last_time_scan)
		self.rot_calc %= 360

		x = self.speed_calc * (t - last_time_scan) * cos(self.rot_calc * pi / 180)
		y = self.speed_calc * (t - last_time_scan) * sin(self.rot_calc * pi / 180)
		self.pos_calc.add(x, y)


	def equip_lidar(self, fov:float, freq:float, res:float, prec:float, max_dist:float):
		"""Equip a Lidar to the robot

		Args:
			fov (float): Horizontal field of view
			freq (float): Scanning frequency
			res (float): Angular resolution
			prec (float): LIDAR precision in % (0, 100)
			max_dist (float): Max range of the Lidar
		"""
		self.lidar = LIDAR(fov, freq, res, max_dist, prec)
		self.lidar.pos = self.pos
		
		
	def scan_environment(self, time, map, window):
		"""Request a scanning of the environment to the lidar

		Args:
			time (float): Current time
			map (Map): Simulation map
			window (surface): Surface on which to draw

		Returns:
			bool: Return if environment was scanned
		"""
		if self.lidar is None:
			print("No lidar equiped !")
			return False
		
		points = self.lidar.scan_environment(time, map, self.radius, window)
		
		# Compute position with lidar data
		if points is not None:
			self.controller.find_new_direction(points, window)
			self.update_live_grid_map(points)
			pos_lidar = self.lidar.correct_pos_with_lidar(points, window)

			if pos_lidar is not None:
				self.pos_calc_lidar = pos_lidar
				self.pos_calc = self.pos_calc_lidar.copy()


	### DISPLAY
	def draw(self, window):
		"""Draw the robot as a circle

		Args:
			window (surface): Surface on which to draw
		"""
		pygame.draw.circle(window, self.color, (self.pos.x, self.pos.y), self.radius)
		pygame.draw.line(window, (255,255,255), (self.pos.x, self.pos.y), (self.pos.x + self.radius * cos(self.rot * pi / 180), self.pos.y  + self.radius * sin(self.rot * pi / 180)), 2)
		pygame.draw.circle(window, (100,100,100), (self.pos_calc.x, self.pos_calc.y), 0.8*self.radius)
		pygame.draw.line(window, (255,255,255), (self.pos_calc.x, self.pos_calc.y), (self.pos_calc.x + self.radius * cos(self.rot_calc * pi / 180), self.pos_calc.y  + self.radius * sin(self.rot_calc * pi / 180)), 2)
		pygame.draw.circle(window, (200,100,100), (self.pos_calc_lidar.x, self.pos_calc_lidar.y), 0.5*self.radius)
	

	def draw_dot_map(self, window):
		"""Draw point found by the robot

		Args:
			window (surface): Surface on which to draw
		"""
		for dot in self.map:
			pygame.draw.circle(window, (255, 255, 255), (dot[0], dot[1]), 1)


	def draw_live_grid_map(self, window, map_offset):
		"""Draw the live grid map, the color change according to the value of confidence, 1 means it is sure that a wall is there
		and drawn as a black square and 0 the opposite

		Args:
			window (surface): Surface on which to draw
		"""
		for idy in range(self.live_grid_map_list_size):
			for idx in range(self.live_grid_map_list_size):
				value = self.live_grid_map[idy, idx]
				if value == 0:
					continue
				if value == -1:
					color = (70, 255, 200)
				else:
					gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (gray_scale, gray_scale, gray_scale)

				top_left_x, top_left_y, _, _ = self.live_grid_map_ids_to_rect(idx, idy)
				# pygame.draw.circle(window, (255, 0, 0), (top_left_x, top_left_y), 2)
				# pygame.draw.circle(window, (255, 255, 0), self.pos.to_tuple(), 2)
				# pygame.display.update()				


				rect = pygame.Rect(top_left_x, top_left_y, self.live_grid_map_size, self.live_grid_map_size)
				pygame.draw.rect(window, color, rect)