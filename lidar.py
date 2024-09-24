from util import Vector2, compute_line, find_intersection, distance
import random
from math import cos, sin, atan2, pi
import pygame

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

	def scan_environment(self, t, obstacles):

		def compute_line_collision(obstacles, line):
			nearest_intersection = None
			min_dist = 1e6
			for obs in obstacles:
				points = [(obs[0], obs[1], obs[0], obs[3]),
			  			  (obs[0], obs[3], obs[2], obs[3]),
			  			  (obs[2], obs[3], obs[2], obs[1]),
			  			  (obs[2], obs[1], obs[0], obs[1])]
				for p1x, p1y, p2x, p2y in points:
					p1 = Vector2(p1x, p1y)
					p2 = Vector2(p2x, p2y)
					line_obs = compute_line(p1, p2)
					intersection = find_intersection(line, line_obs)
					if intersection is not False:
						if abs(atan2(intersection[1] - self.pos.y, intersection[0] - self.pos.x) - self.angle) < 0.1:
							if intersection[0] > min(p1.x, p2.x)-1 and intersection[0] < max(p2.x, p1.x)+1:
								if intersection[1] > min(p1.y, p2.y)-1 and intersection[1] < max(p2.y, p1.y)+1:
									dist = distance(self.pos, intersection)
									if dist < min_dist:
										nearest_intersection = intersection
										min_dist = dist
			if nearest_intersection is not None:
				x = nearest_intersection[0] + self.precision * (random.random()-0.5) * min_dist/100
				y = nearest_intersection[1] + self.precision * (random.random()-0.5) * min_dist/100
				# pygame.draw.line(window, (255,255,255), (self.pos.x, self.pos.y), nearest_intersection, 2)
				return x,y

			return nearest_intersection
						
		if t < self.last_scan_time + 1/self.freq:
			return None
		
		self.last_scan_time = t

		points = []
		while self.angle < pi:
			p = Vector2(self.pos.x + 100*cos(self.angle), self.pos.y + 100*sin(self.angle))
			self.angle += self.resolution * pi / 180.0
			line = compute_line(self.pos, p)
			point = compute_line_collision(obstacles, line)
			if point is not None:
				points.append(point)
		self.angle -= 2*pi
		
		return points

	