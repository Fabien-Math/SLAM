import pygame
from util import Vector2, compute_line
from cave_generator import compute_contours, create_trigle_grid


class Wall:
	def __init__(self, p, height, width, color) -> None:
		self.p : Vector2 = p
		self.width = width
		self.height = height
		self.color = color

	def draw(self, window):
		pygame.draw.rect(window, self.color, (self.p[0], self.p[1], self.height, self.width))

	def get_rect(self):
		rectangle = (self.p[0], self.p[1], self.p[0] + self.height, self.p[1] + self.width)
		return rectangle
	

class Wall_Line:
	def __init__(self, p1:Vector2, p2:Vector2, thickness:float):
		self.p1 = p1
		self.p2 = p2
		self.thickness = thickness
		self.line_eq = compute_line(p1, p2)

	def draw(self, window):
		pygame.draw.line(window, (0,0,0), self.p1.to_tuple(), self.p2.to_tuple(), self.thickness)
	
	def get_line(self):
		return compute_line(self.p1, self.p2)

	
class Map_V2:
	def __init__(self, n, m, dx, dy):
		self.walls = None
		tris, pts, vs = create_trigle_grid(n, m, dx, dy)
		self.contours = compute_contours(tris, pts, vs, 0.5)
		self.initialize_map()
	
	def initialize_map(self):
		self.walls = [None for _ in range(len(self.contours))]

		for i, contour in enumerate(self.contours):
			self.walls[i] = (Wall_Line(contour[0], contour[1], 2))
	
	def draw_map(self, window):
		for wall in self.walls:
			wall.draw(window)

class Map:
	def __init__(self) -> None:
		self.walls = None
		self.initialize_map()
	
	def draw_map(self, window):
		for wall in self.walls:
			wall.draw(window)

	def initialize_map(self):
		self.walls = []
		wall_color = (50, 50, 50)

		# Initialize walls
		self.walls.append(Wall((100,100), 340, 10, wall_color))
		self.walls.append(Wall((100,100), 10, 200, wall_color))
		self.walls.append(Wall((100,300), 75, 10, wall_color))
		self.walls.append(Wall((120,300), 10, 150, wall_color))
		self.walls.append(Wall((120,450), 300, 10, wall_color))
		self.walls.append(Wall((300,250), 10, 200, wall_color))
		self.walls.append(Wall((420,410), 10, 50, wall_color))
		self.walls.append(Wall((420,410), 150, 10, wall_color))
		self.walls.append(Wall((570,410), 10, 100, wall_color))
		self.walls.append(Wall((570,510), 100, 10, wall_color))
		self.walls.append(Wall((670,120), 10, 400, wall_color))
		self.walls.append(Wall((530,120), 150, 10, wall_color))
		self.walls.append(Wall((530,120), 10, 100, wall_color))
		self.walls.append(Wall((430,210), 100, 10, wall_color))
		self.walls.append(Wall((430,100), 10, 120, wall_color))
		self.walls.append(Wall((400,300), 200, 10, wall_color))
		self.walls.append(Wall((600,300), 10, 40, wall_color))
		self.walls.append(Wall((200,180), 10, 70, wall_color))
		self.walls.append(Wall((200,180), 50, 10, wall_color))