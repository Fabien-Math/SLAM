from util import Vector2, compute_line, compute_segment_rect_intersection
from math import pi, sin

from cave_generator import compute_contours, create_trigle_grid


class Wall:
	def __init__(self, p1:Vector2, p2:Vector2, thickness:float):
		self.p1 = p1
		self.p2 = p2
		self.thickness = thickness
		self.line_eq = compute_line(p1, p2)

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
		self.subdiv_size = (map_size[0] / subdiv_number[0], map_size[1] / subdiv_number[1])
		self.subdivision = None

		self.initialize_map()
	
	def initialize_map(self):
		self.walls = [None for _ in range(len(self.contours) + 4)]

		# Random cave map
		for i, contour in enumerate(self.contours):
			contour[0].add(*(self.map_offset[0], self.map_offset[1] + 0.5*self.dy*sin(pi/3)))
			contour[1].add(*(self.map_offset[0], self.map_offset[1] + 0.5*self.dy*sin(pi/3)))
			self.walls[i] = (Wall(contour[0], contour[1], 2))

		safe_offset = 5
		# Top left
		p1 = Vector2(self.map_offset[0] + safe_offset, self.map_offset[1] + safe_offset)
		# Bottom left
		p2 = Vector2(self.map_offset[0] + safe_offset, self.map_offset[1] + self.map_size[1] - safe_offset)
		# Top right
		p3 = Vector2(self.map_offset[0] + self.map_size[0] - safe_offset, self.map_offset[1] + self.map_size[1] - safe_offset)
		# Bottom right
		p4 = Vector2(self.map_offset[0] + self.map_size[0] - safe_offset, self.map_offset[1] + safe_offset)
		# Make borders
		self.walls[-1] = Wall(p1, p2, 2)
		self.walls[-2] = Wall(p2, p3, 2)
		self.walls[-3] = Wall(p3, p4, 2)
		self.walls[-4] = Wall(p4, p1, 2)

		self.create_and_fill_subdivision()


	def create_and_fill_subdivision(self):
		# Initialisation of subdivision list
		self.subdivision = [[[] for _ in range(self.subdiv_number[0])] for _ in range(self.subdiv_number[1])]
		error_offset = 1e-6
		if self.walls is None:
			print("Walls is not defined !")
			return
		for k, wall in enumerate(self.walls):
			p1_ids = self.subdiv_coord_to_ids(wall.p1.to_tuple())
			p2_ids = self.subdiv_coord_to_ids(wall.p2.to_tuple())
			for i in range(min(p1_ids[1], p2_ids[1]), max(p1_ids[1], p2_ids[1]) + 1):
				for j in range(min(p1_ids[0], p2_ids[0]), max(p1_ids[0], p2_ids[0]) + 1):
					rect = self.subdiv_ids_to_rect(j, i)
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


	def subdiv_ids_to_rect(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		p1x = idx * self.subdiv_size[0] + self.map_offset[0]
		p1y = idy * self.subdiv_size[1] + self.map_offset[1]
		p2x = (idx+1) * self.subdiv_size[0] + self.map_offset[0]
		p2y = (idy+1) * self.subdiv_size[1] + self.map_offset[1]
		
		return p1x, p1y, p2x, p2y 
	
	def subdiv_ids_to_centre(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			centre point (tuple): center point of the subdivision
		"""
		p1x = (idx + 0.5) * self.subdiv_size[0] + self.map_offset[0]
		p1y = (idy + 0.5) * self.subdiv_size[1] + self.map_offset[1]
		
		return p1x, p1y

	def subdiv_coord_to_ids(self, pos:tuple):
		"""
		### Give the subdiv ids from the coordinante where the point is

		#### Return
		ids (tuple[int]): (idx, idy)
		"""
		idx = (pos[0] - self.map_offset[0]) * self.subdiv_number[0] / self.map_size[0]
		idy = (pos[1] - self.map_offset[1]) * self.subdiv_number[1] / self.map_size[1]

		return int(idx), int(idy)

	def subdiv_coord_to_ids_float(self, pos:tuple):
		"""
		### Give the subdiv ids from the coordinante where the point is

		#### Return
		ids (tuple[float]): (idx_float, idy_float)
		"""
		idx = (pos[0] - self.map_offset[0]) / self.subdiv_size[0]
		idy = (pos[1] - self.map_offset[1]) / self.subdiv_size[1]

		return idx, idy
