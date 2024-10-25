from util import Vector2, compute_line, find_intersection, point_in_box, distance, find_circle_intersection, find_n_nearest_from_point
import random
from math import cos, sin, pi, sqrt
from map import Wall, Map

class LIDAR:
	def __init__(self, fov, freq, res, prec) -> None:
		self.fov = fov
		self.resolution = res
		self.freq = freq
		self.precision = prec
		self.is_scanning = False
		self.angle = -pi
		self.last_scan_time = 0
		self.pos = Vector2(0, 0) 

	def compute_line_collision(self, walls:list[Wall], line):
			inter_found = False

			for wall in walls:
				wall_line = wall.line_eq
				p1, p2 = wall.p1, wall.p2

				intersection = find_intersection(line, wall_line)

				if intersection:
					if point_in_box(intersection, p1, p2):
						inter_found = True
						break

			if inter_found:
				dist = distance(self.pos, intersection)
				x = intersection[0] + self.precision * (random.random()-0.5) * dist/100
				y = intersection[1] + self.precision * (random.random()-0.5) * dist/100
				return x,y

			return None

	def scan_environment(self, t, map:Map, robot_radius, window):
		"""
		### Scan the environment with the Lidar
		"""
				
		if t < self.last_scan_time + 1/self.freq:
			return None
		
		self.last_scan_time = t
		if map.subdivision is None:
			return None


		points = []
		while self.angle < 0:
			p = Vector2(self.pos.x + 100*cos(self.angle), self.pos.y + 100*sin(self.angle))
			self.angle += self.resolution * pi / 180.0
			line = compute_line(self.pos, p)

			intersection_p = None
			intersection_m = None
			for i in range(51):
				r = robot_radius + i**(2)/5
				pp, pm = compute_ray_pos(line, r, self.pos)

				# Top points
				if intersection_p is None:
					intersection_p = self.check_wall_collision(map, pp, line)
				else:
					points.append(intersection_p)
					
				# Bottom points
				if intersection_m is None:
					intersection_m = self.check_wall_collision(map, pm, line)
				else:
					points.append(intersection_m)

		self.angle -= pi
		
		return points
	
	def check_wall_collision(self, map:Map, p:tuple, line):
		p_ids = map.subdiv_coord_to_ids(p)

		if 0 <= p_ids[0] < map.subdiv_number[0]:
			if 0 <= p_ids[1] < map.subdiv_number[1]:
				subdiv = map.subdivision[p_ids[1]][p_ids[0]]

				# If any wall is in the cell
				if subdiv != []:
					walls = [map.walls[k] for k in subdiv]
					intersection = self.compute_line_collision(walls, line)

					if intersection:
						return intersection
		return None
	

	def correct_pos_with_lidar(self, points, window):
		"""
		### Correct robot position using points from the Lidar
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
		
		idpts = find_n_nearest_from_point(pts, 1, self.pos)

		x, y = 0, 0
		for id in idpts:
			x += pts[id][0]
			y += pts[id][1]
		if len(idpts):
			x /= len(idpts)
			y /= len(idpts)

		return Vector2(x, y)
	
def compute_ray_pos(line, r, offset:Vector2):
		"""
		### Give the two points that intersecti with the circle around the lidar
		"""
		a, b, _ = line
		ab = sqrt(a**2 + b**2)

		xp = r * b / ab + offset.x
		xm = -r * b / ab + offset.x
		yp = -r * a / ab + offset.y
		ym = r * a / ab + offset.y

		return [xp, yp], [xm, ym]


def find_all_circle_intersection(circles):
	points = []
	for i, c1 in enumerate(circles):
		for c2 in circles[i+1::]:
			p1, r1 = c1
			p2, r2 = c2

			pi1, pi2 = find_circle_intersection(p1, r1, p2, r2)
			points += [pi1]
			points += [pi2]

	return points






