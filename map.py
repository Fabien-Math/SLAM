import pygame
from util import Vector2, compute_line, compute_segment_rect_intersection
from cave_generator import compute_contours, create_trigle_grid
import numpy as np

class Wall:
	def __init__(self, p1:Vector2, p2:Vector2, thickness:float):
		self.p1 = p1
		self.p2 = p2
		self.thickness = thickness
		self.line_eq = compute_line(p1, p2)

	def draw(self, window):
		pygame.draw.line(window, (0,0,0), self.p1.to_tuple(), self.p2.to_tuple(), self.thickness)
	
	def get_line(self):
		return compute_line(self.p1, self.p2)

	
class Map:
	def __init__(self, map_size:tuple, map_offset:tuple, subdiv_number:tuple, dx, dy):
		self.walls:list[Wall] = None

		# Map
		self.map_size = map_size
		self.map_offset = map_offset
		self.dx = dx
		self.dy = dy

		tris, pts, vs = create_trigle_grid(map_size, dx, dy)
		self.contours = compute_contours(tris, pts, vs, 0.5)

		# Sub map
		self.subdiv_number = subdiv_number
		self.subdivision = None

		self.initialize_map(map_offset)
	
	def initialize_map(self, map_offset):
		self.walls = [None for _ in range(len(self.contours))]

		for i, contour in enumerate(self.contours):
			contour[0].add(*map_offset)
			contour[1].add(*map_offset)
			self.walls[i] = (Wall(contour[0], contour[1], 2))
		
		self.create_and_fill_subdivision()


	def create_and_fill_subdivision(self):
		# Initialisation of subdivision list
		self.subdivision = [[[] for _ in range(self.subdiv_number[0])] for _ in range(self.subdiv_number[1])]
		points = np.array([[(j * self.map_size[0]/self.subdiv_number[0] + self.map_offset[0], (i-1) * self.map_size[1]/self.subdiv_number[1] + self.map_offset[1]) for j in range(self.subdiv_number[0]+1)] for i in range(self.subdiv_number[1]+1)])
		error_offset = 1e-6
		if self.walls is None:
			print("Walls is not defined !")
			return
		for k, wall in enumerate(self.walls):
			p1_ids = self.subdiv_coord_to_ids(wall.p1.to_tuple())
			p2_ids = self.subdiv_coord_to_ids(wall.p2.to_tuple())
			for i in range(min(p1_ids[1], p2_ids[1]), max(p1_ids[1], p2_ids[1]) + 1):
				for j in range(min(p1_ids[0], p2_ids[0]), max(p1_ids[0], p2_ids[0]) + 1):
					rect = self.subdiv_ids_to_rect(i, j)
					# If the first point is in the box
					if wall.p1.x > min(rect[0], rect[2]) - error_offset and wall.p1.x < max(rect[0], rect[2]) + error_offset:
						if wall.p1.y > min(rect[1], rect[3]) - error_offset and wall.p1.y < max(rect[1], rect[3]) + error_offset:
							self.subdivision[i][j].append(k)
							continue
					# If the second point is in the box
					if wall.p2.x > min(rect[0], rect[2]) - error_offset and wall.p2.x < max(rect[0], rect[2]) + error_offset:
						if wall.p2.y > min(rect[1], rect[3]) - error_offset and wall.p2.y < max(rect[1], rect[3]) + error_offset:
							self.subdivision[i][j].append(k)
							continue
					# If the line touch another box
					is_in_rect = compute_segment_rect_intersection(wall.line_eq, wall.p1, wall.p2, rect)
					if is_in_rect:
						self.subdivision[i][j].append(k)

	def draw_subdivision(self, window):
		dx = self.map_size[0]/self.subdiv_number[0]
		dy = self.map_size[1]/self.subdiv_number[1]

		for i, line in enumerate(self.subdivision):
			for j, walls in enumerate(line):
				if walls != []:
					for wall_id in walls:
						pygame.draw.line(window, (255, 255, 255), self.walls[wall_id].p1.to_tuple(), self.walls[wall_id].p2.to_tuple(), 2)
					# rect = self.subdiv_ids_to_rect(i, j)
					# rect = pygame.Rect(rect[0], rect[1], dx, dy)
					# pygame.draw.rect(window, (255, 0, 0), rect)

	def subdiv_ids_to_rect(self, i, j):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		p1x = j * self.map_size[0]/self.subdiv_number[0] + self.map_offset[0]
		p1y = (i-1) * self.map_size[1]/self.subdiv_number[1] + self.map_offset[1]
		p2x = (j+1) * self.map_size[0]/self.subdiv_number[0] + self.map_offset[0]
		p2y = i * self.map_size[1]/self.subdiv_number[1] + self.map_offset[1]
		
		return p1x, p1y, p2x, p2y 

	def subdiv_coord_to_ids(self, pos:tuple):
		"""
		### Give the subdiv ids from the coordinante where the point is

		#### Return
		ids (tuple[int]): (idx, idy)
		"""
		idx = (pos[0] - self.map_offset[0]) * self.subdiv_number[0] / self.map_size[0]
		idy = (pos[1] - self.map_offset[1]) * self.subdiv_number[1] / self.map_size[1]

		return int(idx), int(idy+1)


	def draw_subdiv_points(self, window):
		for i in range(self.subdiv_number[1]):
			for j in range(self.subdiv_number[0]):
				rect = self.subdiv_ids_to_rect(i, j)
				p = [rect[0], rect[1]]
				c_p = [(rect[0] + rect[2])/2 + 10, (rect[1] + rect[3])/2 + 10]
				pygame.draw.circle(window, (0, 0, 0), p, 2)
				# idx, idy = self.subdiv_coord_to_ids(c_p)
				# if i != idy or j != idx:
				# 	print(j, i, c_p, self.subdiv_coord_to_ids(c_p))
				# pygame.draw.circle(window, (255, 0, 255), c_p, 2)


	
	
	def draw_map(self, window):
		for wall in self.walls:
			wall.draw(window)
