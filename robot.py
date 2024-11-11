from util import Vector2, sign
from math import cos, sin ,pi

from lidar import LIDAR
from accelerometer import Accmeter
from live_grid_map import Live_grid_map
from controller import Controller

import pygame


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
		self.live_grid_map = Live_grid_map(self, 201, 10)

		# Sensors
		self.lidar = None
		self.accmeter = None
		self.controller = None

	def move(self, dt:float, direction:int, coef_max_speed:float = 1, coef_max_acc:float = 1):
		"""Move the robot

		Args:
			dt (float): Time
			coef_max_acc (float): _description_
			coef_max_speed (float, optional): _description_. Defaults to 1.
		"""

		if direction == 1:
			max_speed = coef_max_speed * self.max_forward_speed
		elif direction == -1:
			max_speed = coef_max_speed * self.max_backward_speed
		else:
			max_speed = 0
			
		acc = coef_max_acc * self.acc
		decc = coef_max_acc * self.decc

		abs_speed = abs(self.speed)

		# Accelration
		if abs_speed < max_speed:
			self.speed = direction * min(abs_speed + acc*dt, max_speed)
		
		# Decceleration
		if abs_speed > max_speed:
			self.speed = sign(self.speed) * max(abs_speed - decc*dt, 0)

	
		# Compute traveled distance
		x = self.speed * cos(self.rot * pi / 180) * dt
		y = self.speed * sin(self.rot * pi / 180) * dt
		
		# Update robot position
		self.pos.add(x, y)


	def rotate(self, dt:float, direction:float, coef_max_rot_speed:float = 1, coef_max_rot_acc:float = 1):
		"""Rotate the robot

		Args:
			dir (float): Direction to rotate
			dt (float): Time step
		"""

		if direction == 0:
			max_rot_speed = 0
		else:
			max_rot_speed = coef_max_rot_speed * self.max_rot_speed
		rot_acc = coef_max_rot_acc * self.rot_acc
		rot_decc = coef_max_rot_acc * self.rot_decc

		abs_rot_speed = abs(self.rot_speed)

		# Accelration
		if abs_rot_speed < max_rot_speed:
			self.rot_speed = direction * min(abs_rot_speed + rot_acc*dt, max_rot_speed)
		
		# Decceleration
		if abs_rot_speed >= max_rot_speed:
			self.rot_speed = sign(self.rot_speed) * max(abs_rot_speed - rot_decc*dt, 0)
		
		# Update rotation
		self.rot += self.rot_speed * dt
		self.rot %= 360



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
			# self.controller.find_new_direction(points, window)
			self.live_grid_map.update(self.pos_calc, points, self.lidar.max_dist)
			pos_lidar = self.lidar.correct_pos_with_lidar(points, window)

			if pos_lidar is not None:
				self.pos_calc_lidar = pos_lidar
				self.pos_calc = self.pos_calc_lidar.copy()




	### EQUIP EQUIPEMENT
	def equip_accmeter(self, acc_prec:float, ang_acc_prec:float):
		"""Equip accelerometer to the robot

		Args:
			prec (float): Precision of the accelerometer in % (0 to 100)
		"""
		self.accmeter = Accmeter(acc_prec, ang_acc_prec)


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

	def equip_controller(self, check_safe_path_frequency, mode):
		self.controller = Controller(self, check_safe_path_frequency, mode)
		

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


