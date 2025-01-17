from util import Vector2, sign, distance_tuple, wall_orthogonal_point, point_in_circle, point_in_box, find_id_min, distance, find_circle_intersection
from math import cos, sin ,pi

from lidar import LIDAR
from accelerometer import Accmeter
from live_grid_map import Live_grid_map
from controller import Controller
from map import Map

import pygame


class BeaconRobot:
	def __init__(self, pos:Vector2, acc:float, ang_acc:float, max_forward_speed:float, max_backward_speed:float, max_rot_speed:float) -> None:
		# Position and rotation
		self.pos = Vector2(pos[0], pos[1])
		self.pos_calc = Vector2(pos[0], pos[1])
		self.pos_calc_lidar = Vector2(pos[0], pos[1])
		self.rot = 0
		self.rot_calc = 0
		
		# Acceleration and decceleration
		self.acc = acc
		self.ang_acc = ang_acc
		self.acc_calc = 0
		self.ang_acc_calc = 0
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
		self.crashed_in_wall = False

		# Robot map
		self.map = []
		self.live_grid_map = Live_grid_map(self, 101, 15)

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


	def rotate(self, dt:float, direction:float, coef_max_rot_speed:float = 1, coef_max_ang_acc:float = 1):
		"""Rotate the robot

		Args:
			dir (float): Direction to rotate
			dt (float): Time step
		"""

		if direction == 0:
			max_rot_speed = 0
		else:
			max_rot_speed = coef_max_rot_speed * self.max_rot_speed
		ang_acc = coef_max_ang_acc * self.ang_acc
		rot_decc = coef_max_ang_acc * self.rot_decc

		abs_rot_speed = abs(self.rot_speed)

		# Accelration
		if abs_rot_speed < max_rot_speed:
			self.rot_speed = direction * min(abs_rot_speed + ang_acc*dt, max_rot_speed)
		
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
		if self.accmeter is None:
			print("No accelerometer equiped !")
			return False
		
		last_time_scan = self.accmeter.last_scan_time
		dt = t - last_time_scan
		self.acc_calc, self.ang_acc_calc = self.accmeter.get_acc(self.speed, self.rot_speed, t)

		self.speed_calc += self.acc_calc * dt

		self.rot_speed_calc += self.ang_acc_calc * dt
		self.rot_calc += self.rot_speed_calc * dt
		self.rot_calc %= 360

		x = self.speed_calc * dt * cos(self.rot_calc * pi / 180)
		y = self.speed_calc * dt * sin(self.rot_calc * pi / 180)
		self.pos_calc.add(x, y)

	


	def scan_environment(self, time, map:Map, window):
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
		
		lidar_data = self.lidar.scan_environment(time, map, window)
		if lidar_data is None:
			return None
		
		# Update live grid map with all known and unknown point
		# if no_inter_point is not None:
		# 	self.live_grid_map.update(points, self.lidar.max_dist)

		# Compute position of point given the lidar data
		points = [(self.pos_calc.x + dist*cos(ang), self.pos_calc.y + dist*sin(ang)) for dist, ang in lidar_data]
		collide_points = [point for point in points if distance(self.pos_calc, point) < self.lidar.max_dist*1.5]

		self.live_grid_map.update(points, self.lidar.max_dist, window)
		# self.map += [point for point in points if distance(self.pos_calc, point) < self.lidar.max_dist*1.5]
		if len(collide_points):
			pos_lidar = self.correct_pos_with_lidar(collide_points, window)

			if pos_lidar is not None:
				self.pos_calc_lidar = pos_lidar
				self.pos_calc = self.pos_calc_lidar.copy()
	

	def correct_pos_with_lidar(self, points:list, window):
		"""Correct the position of the robot using lidar scan

		Args:
			points (list): List of the point detected by the lidar
			window (surface): Surface on which to draw the scene

		Returns:
			Vector2: Estimated position of the lidar
		"""
		n = len(points)
		if n < 3:
			return
		
		n_circle = 3
		circles = [0] * n_circle
		for i in range(n_circle):
			p = points[(i*n)//n_circle]
			circles[i] = (p, distance(self.pos, p))

		pts = find_all_circle_intersection(circles)

		if not len(pts):
			return None
		
		idpt = find_n_nearest_from_point(pts, int(n_circle/2), self.pos_calc)

		# Average coordinate
		x, y = pts[idpt[0]]
		
		return Vector2(x, y)



	### EQUIP EQUIPEMENT
	def equip_accmeter(self, precision:tuple, time:float):
		"""Equip accelerometer to the robot

		Args:
			prec (float): Precision of the accelerometer in % (0 to 100)
		"""
		self.accmeter = Accmeter(precision, time)


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

	def equip_controller(self, mode):
		self.controller = Controller(self, mode)
		
	
	### ROBOT COLLISION

	def compute_robot_collision(self, map:Map, window):
		"""Compute collision between the robot and walls
		"""
		idx, idy = map.subdiv_coord_to_ids(self.pos.to_tuple())
		
		if map.subdivision is None:
			return None
		
		nx, ny = map.subdiv_number

		for idxs in range(idx-1, idx+2):
			for idys in range(idy-1, idy+2):
				if idxs > nx - 1 or idxs < 0 or idys > ny - 1 or idys < 0:
					continue
				
				map_subdiv = map.subdivision[idys][idxs]
				if map_subdiv != []:
					walls = [map.walls[i] for i in map_subdiv]
					for wall in walls:
						if point_in_circle(wall.p1.to_tuple(), self.pos.to_tuple(), self.radius):
							self.crashed_in_wall = True
							return
						if point_in_circle(wall.p2.to_tuple(), self.pos.to_tuple(), self.radius):
							self.crashed_in_wall = True
							return

						ortho_point = wall_orthogonal_point(self.pos.to_tuple(), wall, window)

						if point_in_box(ortho_point, wall.p1, wall.p2):
							if point_in_circle(ortho_point, self.pos.to_tuple(), self.radius):
								self.crashed_in_wall = True
								return

		self.crashed_in_wall = False


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
			pygame.draw.circle(window, (255, 255, 255), (dot[0], dot[1]), 2)



###   FUNCTIONS   ###

def find_all_circle_intersection(circles):
	"""Compute all intersection between circles

	Args:
		circles (list): List of all circles, all circles must be different

	Returns:
		list: List of intersecting point
	"""

	points = []
	for i, c1 in enumerate(circles):
		for c2 in circles:
			if c1 == c2:
				continue
			p1, r1 = c1
			p2, r2 = c2

			ps = find_circle_intersection(p1, r1, p2, r2)
			# If intersection found
			if ps is not None:
				pi1, pi2 = ps
				points += [pi1]
				points += [pi2]

	return points


def find_n_nearest_from_point(ps: list, n: int, pos:Vector2):
	"""Find the 'n' nearest point from a position
	"""
	ds = [0]*len(ps)
	for i, p in enumerate(ps):
		ds[i] = distance(pos, p)

	ds_min = [0]*n
	for i in range(n):
		id_min = find_id_min(ds)
		ds_min[i] = id_min + i
		ds.pop(id_min)

	return ds_min