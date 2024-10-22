from util import Vector2, compute_line, find_intersection, distance, find_circle_intersection, find_n_nearest
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

	def compute_line_collision(self, obstacles, line):
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

	def scan_environment(self, t, obstacles):
				
		if t < self.last_scan_time + 1/self.freq:
			return None
		
		self.last_scan_time = t

		points = []
		while self.angle < pi:
			p = Vector2(self.pos.x + 100*cos(self.angle), self.pos.y + 100*sin(self.angle))
			self.angle += self.resolution * pi / 180.0
			line = compute_line(self.pos, p)
			point = self.compute_line_collision(obstacles, line)
			if point is not None:
				points.append(point)
		self.angle -= 2*pi
		
		return points

	def compute_lidar_position(self, points):
		n = len(points)
		p1 = points[0]
		r1 = distance(self.pos, p1)
		p2 = points[n//4]
		r2 = distance(self.pos, p2)
		p3 = points[(2*n)//4]
		r3 = distance(self.pos, p3)
		p4 = points[(3*n)//4]
		r4 = distance(self.pos, p4)

		pi12, pi21 = find_circle_intersection(p1, r1, p2, r2)
		pi23, pi32 = find_circle_intersection(p2, r2, p3, r3)
		pi34, pi43 = find_circle_intersection(p3, r3, p4, r4)
		pi41, pi14 = find_circle_intersection(p4, r4, p1, r1)
		pts = [pi12, pi21, pi23, pi32, pi34, pi43, pi41, pi14]

		idpts = find_n_nearest(pts, 4)
		x, y = 0, 0
		for id in idpts:
			x += pts[id][0]
			y += pts[id][1]
		if len(idpts):
			x /= len(idpts)
			y /= len(idpts)

		return Vector2(x, y)



	