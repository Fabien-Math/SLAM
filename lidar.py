from util import *
import random
from math import cos, sin, pi, sqrt
from map import Wall, Map
import pygame

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
		# Lidar max distance scan
		self.max_dist = max_dist
		# State of the LIDAR
		self.is_scanning = False
		# Sarting angle
		self.angle = 0
		# Last time the lidar scan the environment
		self.last_scan_time = 0
		# Position of the LIDAR
		self.pos = Vector2(0, 0) 


	def compute_line_collision(self, walls:list[Wall], line:tuple, window):
		"""Compute the collision point between a line and a segment

		Args:
			walls (list[Wall]): List of wall
			line (tuple): Line equation parameter a, b and c (ax + by = c)

		Returns:
			tuple: Intersection point between the line and the segment if it exists, else return None
		"""
		nearest_intersections = None
		min_inter_dist = 1e9

		for wall in walls:
			wall_line = wall.line_eq
			p1, p2 = wall.p1, wall.p2

			intersection = find_intersection(line, wall_line)
			if intersection:
				if point_in_box(intersection, p1, p2):
					inter_dist = distance(self.pos, intersection)
					if inter_dist < min_inter_dist:
						# pygame.draw.circle(window, (55, 255, 255), intersection, 3)
						# pygame.time.delay(10)
						# pygame.display.update()
						nearest_intersections = intersection
						min_inter_dist = inter_dist


		if nearest_intersections is not None:
			# pygame.draw.circle(window, (255, 55, 255), nearest_intersections, 3)
			# pygame.time.delay(20)
			# pygame.display.update()
			dist = distance(self.pos, nearest_intersections)
			x = nearest_intersections[0] + self.precision * (random.random()-0.5) * dist/100
			y = nearest_intersections[1] + self.precision * (random.random()-0.5) * dist/100
			# pygame.draw.circle(window, (255, 55, 100), (x, y), 5, 2)
			# pygame.time.delay(10)
			# pygame.display.update()
			return x,y

		return None


	def scan_environment(self, t, map:Map, robot_radius: float, window):
		"""
		Perform a Lidar scan of the environment.

		Args:
			t (float): Current simulation time
			map (Map): Simulation map
			robot_radius (float): Radius of the robot for scan calculations
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
		points = []
		points_mask = []
		
		while self.angle < 2*pi:
			p_r = Vector2(self.pos.x + 100*cos(self.angle), self.pos.y + 100*sin(self.angle))
			line = compute_line(self.pos, p_r)

			# Upward intersection point
			intersection = None

			# Number of point to check according to the lidar max distance
			N = int(sqrt(5 * (self.max_dist - robot_radius)))
			for i in range(N):
				# Upward points
				r = robot_radius + i**(2)/5
				p_r = (r*cos(self.angle) + self.pos.x, r*sin(self.angle) + self.pos.y)
				# pygame.draw.circle(window, (34,134,233), p_r, 2)

				intersection = self.check_wall_collision(map, p_r, line, window)

				if intersection is not None:
					points.append(intersection)
					break

			# Increment angle with resolution
			self.angle += self.resolution * pi / 180.0

		# Reset angle
		self.angle -= 2*pi

		# Update last scan time
		self.last_scan_time = t

		return points
	

	def check_wall_collision(self, map:Map, p:tuple, line, window):
		"""Check if the line shot by the lidar collide with a wall

		Args:
			map (Map): Simulation map
			p (tuple): Lidar point
			line (tuple): Line equation parameter a, b and c (ax + by = c)

		Returns:
			tuple: Intersection point found or None if no point found
		"""

		# Convert the position into the indices of the subdivision map
		p_ids = map.subdiv_coord_to_ids(p)
		rect = map.subdiv_ids_to_rect(p_ids[0], p_ids[1])
		# rect = pygame.Rect(rect[0], rect[1], abs(rect[0] - rect[2]), abs(rect[1] - rect[3]))
		# pygame.draw.rect(window, (255, 0, 0), rect, 2)

		# Check if the ids are in the subdivided map
		if 0 <= p_ids[0] < map.subdiv_number[0]:
			if 0 <= p_ids[1] < map.subdiv_number[1]:
				# Get the list of indices of walls in the subdivision box 
				subdiv = map.subdivision[p_ids[1]][p_ids[0]]

				# If any wall is in the cell
				if subdiv != []:
					walls = [map.walls[k] for k in subdiv]
					# for wall in walls:
						# pygame.draw.line(window, (255, 255, 255), wall.p1.to_tuple(), wall.p2.to_tuple(), 2)

					intersection = self.compute_line_collision(walls, line, window)

					if intersection:
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


def compute_ray_pos(line, r:float, offset:Vector2):
	"""Compute the point distant from r from the lidar

	Args:
		line (tuple): Line equation parameter a, b and c (ax + by = c)
		r (float): Distance of the point to be
		offset (Vector2): Offset position

	Returns:
		tuple: Points intersecting the circle of radius r
	"""
	a, b, _ = line
	ab = sqrt(a**2 + b**2)

	xp = r * b / ab + offset.x
	xm = -r * b / ab + offset.x
	yp = -r * a / ab + offset.y
	ym = r * a / ab + offset.y

	return [xp, yp], [xm, ym]


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