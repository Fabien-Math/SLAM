from util import * #Vector2, sign, distance, find_intersection, point_in_box, compute_line, find_circle_intersection, find_id_min, get_angle_tuple_deg
from math import cos, sin, pi, sqrt
import random
import pygame
import numpy as np

from map import Wall, Map

class LIDAR:
	"""Class to simulate a LIDAR
	"""
	def __init__(self, fov:float, freq:float, res:float, max_dist:float, prec:float) -> None:
		"""Initialize the LIDAR

		Args:
			fov (float): Horizontal field of view
			freq (float): Scanning frequency
			res (float): Angular resolution
			prec (float): LIDAR precision in % (0, 100)
			max_dist (float): Max range of the Lidar
		"""
		
		# Horizontal field of view
		self.fov = fov
		# Angular resolution
		self.resolution = res
		# Scanning frequency
		self.freq = freq
		# LIDAR precision in % (0, 100)
		self.precision = prec
		self.sigma = np.array(prec)
		# Lidar max distance scan
		self.max_dist = max_dist
		# State of the LIDAR
		self.is_scanning = False
		# Sarting angle
		self.angle = 0
		self.nb_angle = int(360 / res) + 1 * ((360 / res)%1 >= 0.5)
		# Last time the lidar scan the environment
		self.last_scan_time = -100
		# Position of the LIDAR
		self.pos = Vector2(0, 0)
		# Lidar data
		self.data = None


	def scan_environment(self, t, map:Map, robot_id, robots, window):
		"""
		Perform a Lidar scan of the environment.

		Args:
			t (float): Current simulation time
			map (Map): Simulation map
			window (surface): Display surface

		Returns:
			list[tuple]: List of intersection points (x, y) where Lidar beams hit walls or obstacles in the environment.
		"""

		# Wait for the lidar to be available
		if t < self.last_scan_time + 1/self.freq:
			return None
		
		# If the map isn't initialized
		if map.subdivision is None:
			return None

		# Initialize intersection point list
		data = []


		while self.angle < 2*pi:
			p_r = Vector2(self.pos.x + self.max_dist*cos(self.angle), self.pos.y + self.max_dist*sin(self.angle))

			# intersection = self.move_on_line(p_r, map, window)

			# From the lidar position to the max range position
			ids1 = map.subdiv_coord_to_ids(self.pos.to_tuple())
			ids2 = map.subdiv_coord_to_ids(p_r.to_tuple())
			ids1_float = map.subdiv_coord_to_ids_float(self.pos.to_tuple())
			ids2_float = map.subdiv_coord_to_ids_float(p_r.to_tuple())

			ids_line = move_on_line(ids1, ids2, ids1_float, ids2_float)
			for ids in ids_line:
				intersection = self.check_wall_collision(map, ids, p_r, window)

				if intersection is not None:
					break;

			robot_intersection = None
			for robot in robots:
				if robot.id != robot_id:
					line = compute_line(self.pos, p_r)
					if orthogonal_projection(robot.pos.to_tuple(), line) < robot.radius:
						robot_intersection = orthogonal_point(robot.pos.to_tuple(), self.pos.to_tuple(), p_r.to_tuple(), line)

			if robot_intersection is not None and intersection is not None:
				d_r = distance_tuple(robot_intersection, self.pos.to_tuple())
				d_i = distance_tuple(intersection, self.pos.to_tuple())
				if d_r < d_i:
					print("Robot intercepted !!")
					intersection = robot_intersection

			if intersection is not None:
				dist, ang = add_uncertainty(distance_tuple(intersection, self.pos.to_tuple()), self.angle, self.sigma)
				data.append((dist, ang))
			else:
				data.append((2*self.max_dist, self.angle))
		
			# Increment angle with resolution
			self.angle += self.resolution * pi / 180.0

		# Reset angle
		self.angle -= 2*pi

		# Update last scan time
		self.last_scan_time = t

		self.data = data
		return data
	
	
	def check_wall_collision(self, map:Map, ids:tuple, p_r, window):
		"""Check if the line shot by the lidar collide with a wall

		Args:
			map (Map): Simulation map
			ids (tuple): idx, idy of the map subdivision
			line (tuple): Line equation parameter a, b and c (ax + by = c)

		Returns:
			tuple: Intersection point found or None if no point found
		"""

		# Check if the ids are in the subdivided map
		if 0 <= ids[0] < map.subdiv_number[0]:
			if 0 <= ids[1] < map.subdiv_number[1]:
				# Get the list of indices of walls in the subdivision box 
				subdiv = map.subdivision[ids[1]][ids[0]]

				# If any wall is in the cell
				if subdiv != []:

					# Go through all walls in the map subdivision
					for k in subdiv:
						wall = map.walls[k]
						intersection = find_segment_intersection(wall.p1, wall.p2, self.pos, p_r)
												
						if intersection:
							rect = map.subdiv_ids_to_rect(ids[0], ids[1])
							if point_in_box_tuple(intersection, (rect[0], rect[1]), (rect[2], rect[3])):
								return intersection
		return None


# FUNCTIONS
def add_uncertainty(distance, angle, sigma):
	mean = np.array([distance, angle])
	covariance = np.diag(sigma ** 2)
	dist, ang = normal_distribution(mean, covariance)
	dist = max(dist, 0)
	ang = max(ang, 0)
	return dist, ang
