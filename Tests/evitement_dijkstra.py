import pygame
import math

# Constants
WIDTH, HEIGHT = 1200, 700
GRID_SIZE = 10
OBSTACLE_RADIUS = 200
OBSTACLE_CENTER = (WIDTH // 2, HEIGHT // 2)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Initialize pygame
pygame.init()
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dijkstra's Algorithm")

class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.neighbors = []
		self.g_cost = float('inf')
		self.visited = False
		self.previous = None

	def add_neighbors(self, grid, obstacle_center, obstacle_radius):
		directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
		for dx, dy in directions:
			nx, ny = self.x + dx, self.y + dy
			if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid):
				if math.hypot((nx * GRID_SIZE) - obstacle_center[0], (ny * GRID_SIZE) - obstacle_center[1]) > obstacle_radius:
					self.neighbors.append(grid[ny][nx])


def draw(win, grid, start, end, path):
	win.fill(WHITE)

	# Draw obstacle

	# Draw nodes and path
	for row in grid:
		for node in row:
			color = WHITE
			if node == start:
				color = GREEN
			elif node == end:
				color = RED
			elif node in path:
				color = BLUE
			pygame.draw.rect(win, color, (node.x * GRID_SIZE, node.y * GRID_SIZE, GRID_SIZE, GRID_SIZE))
	pygame.draw.circle(win, (0, 0, 0), (600, 350), 200)

	pygame.display.update()


def reconstruct_path(end_node):
	path = []
	current = end_node
	while current.previous:
		path.append(current)
		current = current.previous
	return path[::-1]


def dijkstra(grid, start, end):
	open_set = []
	start.g_cost = 0
	open_set.append(start)

	while open_set:
		open_set.sort(key=lambda node: node.g_cost)
		current_node = open_set.pop(0)
		current_node.visited = True

		if current_node == end:
			return reconstruct_path(end)

		for neighbor in current_node.neighbors:
			if not neighbor.visited:
				if neighbor.x != current_node.x and neighbor.y != current_node.y:
					tentative_g_cost = current_node.g_cost + math.sqrt(2)
				else:
					tentative_g_cost = current_node.g_cost + 1

				if tentative_g_cost < neighbor.g_cost:
					neighbor.g_cost = tentative_g_cost
					neighbor.previous = current_node
					if neighbor not in open_set:
						open_set.append(neighbor)

	return []


def main():
	cols = WIDTH // GRID_SIZE
	rows = HEIGHT // GRID_SIZE
	grid = [[Node(x, y) for x in range(cols)] for y in range(rows)]
	for row in grid:
		for node in row:
			node.add_neighbors(grid, OBSTACLE_CENTER, OBSTACLE_RADIUS)

	start = grid[35][20]
	end = grid[35][100]
	running = True

	c = False
	while running:
		# draw(WIN, grid, start, end, [])

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

			if pygame.mouse.get_pressed()[0]:  # Left mouse button
				pos = pygame.mouse.get_pos()
				grid_x, grid_y = pos[0] // GRID_SIZE, pos[1] // GRID_SIZE
				if not start:
					start = grid[grid_y][grid_x]
				elif not end:
					end = grid[grid_y][grid_x]

		if c:
			continue

		if start and end:
			path = dijkstra(grid, start, end)
			draw(WIN, grid, start, end, path)
			print("Path length :", GRID_SIZE * len(path))
			c = True



	pygame.quit()

if __name__ == "__main__":
	main()