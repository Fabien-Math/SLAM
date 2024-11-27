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
		self.last_scan_time = 0
		# Position of the LIDAR
		self.pos = Vector2(0, 0) 


	def scan_environment(self, t, map:Map, window):
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

			intersection = self.move_on_line(p_r, map, window)
			if intersection is not None:
				dist, ang = add_uncertainty(distance_tuple(intersection, self.pos.to_tuple()), self.angle, self.sigma)
				# point = (self.pos.x + dist*cos(ang), self.pos.y + dist*sin(ang))
				# print(point)
				data.append((dist, ang))
		
			# Increment angle with resolution
			self.angle += self.resolution * pi / 180.0

		# Reset angle
		self.angle -= 2*pi

		# Update last scan time
		self.last_scan_time = t

		return np.array(data)
	
		
	def move_on_line(self, p_r:tuple, map:Map, window):
		"""Travel along a line given by tow points to look for all subdivision it cross in map subdivision

			ids1 (tuple[int]): Map subdivision ids of the start point from which to travel along
			ids2 (tuple[int]): Map subdivision ids of the end point
			ids1_float (tuple[float]): Float map subdivision ids of the start point
			ids2_float (tuple[float]): Float map subdivision ids of the en point

		Args:
			p_r (tuple): End point
			map (Map): Map
			window (Surface): Surface on which to draw

		Returns:
			tuple: Intersection point
		"""
		debug = False
		debug_vertical = False

		# From the lidar position to the max range position
		ids1 = map.subdiv_coord_to_ids(self.pos.to_tuple())
		ids2 = map.subdiv_coord_to_ids(p_r.to_tuple())
		ids1_float = map.subdiv_coord_to_ids_float(self.pos.to_tuple())
		ids2_float = map.subdiv_coord_to_ids_float(p_r.to_tuple())

		if debug:
			window.fill((100, 100, 100))
			map.draw_map(window)

		idx, idy = ids1
		idx_float, idy_float = ids1_float
		
		a, b, _ = compute_line_tuple(ids1_float, ids2_float)
		if b:
			m = - a / b
		
		if ids1[0] == ids2[0]:	# Vertical line
			direction =  1 * (ids1[1] < ids2[1]) - 1 * (ids1[1] >= ids2[1])
			for idyi in range(ids1[1], ids2[1] + direction, direction):
				if debug_vertical:
					rect = map.subdiv_ids_to_rect(idx, idyi)
					rect = (rect[0], rect[1], abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
					pygame.draw.rect(window, (255, 0, 0), rect, 2)
					pygame.display.update()
					pygame.time.delay(100)
					print(ids1, ids2, idy, direction)
					print(ids1[1], ids2[1] + direction, direction)
				
				intersection = self.check_wall_collision(map, (idx, idyi), p_r, window)
				if intersection is not None:
					if debug_vertical:
						print("coucou")
						pygame.draw.circle(window, (0, 255, 0), intersection, 2)
						pygame.display.update()
						pygame.time.delay(100)
					return intersection

			return None
		
		if ids1[1] == ids2[1]:	# Horizontal line
			direction = 1 * (ids1[0] < ids2[0]) - 1 * (ids1[0] >= ids2[0])
			for idx in range(ids1[0], ids2[0] + direction, direction):
				intersection = self.check_wall_collision(map, (idx, idy), p_r, window)

				if intersection is not None:
					return intersection
			return None


		direction = sign(ids1[0] - ids2[0])

		n = 0
		if abs(m) < 1:
			while n <= abs(ids1[0] - ids2[0]):
				for idyi in [idy - 1, idy, idy +1]:
					if debug:
						rect = map.subdiv_ids_to_rect(idx, idyi)
						rect = (rect[0], rect[1], abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
						pygame.draw.rect(window, (255, 255, 0), rect, 2)
						pygame.display.update()
						pygame.time.delay(100)
					intersection = self.check_wall_collision(map, (idx, idyi), p_r, window)

					if intersection is not None:
						if debug:
							pygame.draw.circle(window, (255, 255, 0), intersection, 2)
							pygame.display.update()
							pygame.time.delay(100)
						return intersection

				idx_float -= 1 * direction
				idy_float -= m * direction

				idx = int(idx_float)
				idy = int(idy_float)
				n += 1
		else:
			while n <= abs(ids1[1] - ids2[1]):
				for idxi in [idx-1, idx, idx+1]:
					if debug:

						rect = map.subdiv_ids_to_rect(idxi, idy)
						rect = (rect[0], rect[1], abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
						pygame.draw.rect(window, (255, 0, 255), rect, 2)
						pygame.display.update()
						pygame.time.delay(100)


					intersection = self.check_wall_collision(map, (idxi, idy), p_r, window)
					if intersection is not None:
						if debug:
							pygame.draw.circle(window, (255, 255, 0), intersection, 2)
							pygame.display.update()
							pygame.time.delay(100)

						return intersection
					
				idx_float -= abs(1/m) * direction
				idy_float -= sign(m) * direction

				idx = int(idx_float)
				idy = int(idy_float)

				n += 1

		return None


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
				# draw_rect(ids[0], ids[1], window, map)

				# If any wall is in the cell
				if subdiv != []:
					walls = [map.walls[k] for k in subdiv]

					for wall in walls:
						intersection = find_segment_intersection(wall.p1, wall.p2, self.pos, p_r)
												
						if intersection:
							rect = map.subdiv_ids_to_rect(ids[0], ids[1])
							if point_in_box(intersection, Vector2(rect[0], rect[1]), Vector2(rect[2], rect[3])):
								return intersection
		return None
	

	def correct_pos_with_lidar(self, points:list, window):
		"""Correct the position of the robot using lidar scan

		Args:
			points (list): List of the point detected by the lidar
			window (surface): Surface on which to draw the scene

		Returns:
			Vector2: Estimated position of the lidar
		"""
		n = len(points)
		if n == 0:
			return
		
		n_circle = 2
		circles = [0] * n_circle
		for i in range(n_circle):
			p = points[(i*n)//n_circle]
			circles[i] = (p, distance(self.pos, p))
			# pygame.draw.circle(window, (255, 0, 0), p, distance(self.pos, p), 2)
			# pygame.display.update()
			# pygame.time.delay(10000)

		pts = find_all_circle_intersection(circles)
		
		idpts = find_n_nearest_from_point(pts, int(n_circle/2), self.pos)

		# Average coordinate
		x, y = 0, 0
		for id in idpts:
			x += pts[id][0]
			y += pts[id][1]
		if len(idpts):
			x /= len(idpts)
			y /= len(idpts)

		return Vector2(x, y)


# FUNCTIONS
def add_uncertainty(distance, angle, sigma):
	mean = np.array([distance, angle])
	covariance = np.diag(sigma ** 2)
	dist, ang = normal_distribution(mean, covariance)
	dist = max(dist, 0)
	ang = max(ang, 0)
	return dist, ang

def find_all_circle_intersection(circles):
	"""Compute all intersection between circles

	Args:
		circles (list): List of all circles, all circles must be different

	Returns:
		list: List of intersecting point
	"""

	points = []
	for i, c1 in enumerate(circles):
		for c2 in circles[i+1::]:
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