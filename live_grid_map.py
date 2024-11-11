from util import *
import numpy as np

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from robot import BeaconRobot  # Import only for editor, avoid circular import



class Live_grid_map():
	def __init__(self, robot, list_size, size):
		self.robot:BeaconRobot = robot

		self.centre = robot.pos.copy()
		self.list_size = list_size
		self.size = size
		self.map = np.zeros((self.list_size, self.list_size))
		self.saved_map = None
	
	def vec_to_ids(self, vector2):
		idx = int((vector2.x - self.centre.x)/self.size + self.list_size//2)
		idy = int((vector2.y - self.centre.y)/self.size + self.list_size//2)
		return idx, idy

	def coord_to_ids(self, point):
		idx = int((point[0] - self.centre.x)/self.size + self.list_size//2)
		idy = int((point[1] - self.centre.y)/self.size + self.list_size//2)
		return idx, idy

	def ids_to_rect(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		if 0 <= idx < self.list_size and 0 <= idy < self.list_size:
			offset_id = self.list_size//2
			p1x = (idx - offset_id) * self.size + self.centre.x
			p1y = (idy - offset_id) * self.size + self.centre.y
			p2x = (idx + 1 - offset_id) * self.size + self.centre.x
			p2y = (idy + 1 - offset_id) * self.size + self.centre.y

		return p1x, p1y, p2x, p2y 
	
	
	def update(self, robot_pos, points, max_lidar_distance):
		"""Update the live grid of the robot

		Args:
			points (list): List of points to be added, if None, update only robot pos
		"""
		# Add robot pos
		idx, idy = self.vec_to_ids(robot_pos)
		# If ids are in the range of the live map
		if 0 < idx < self.list_size and 0 < idy < self.list_size:
			# Set to safe path
			self.map[idy, idx] = -1

		if points is None:
			return
		
		# Add lidar points pos
		for point in points:
			idx, idy = self.coord_to_ids(point)
			# If ids are in the range of the live map
			if 0 < idx < self.list_size and 0 < idy < self.list_size:
				self.map[idy, idx] = max(self.map[idy, idx], 1 - distance(robot_pos, point)/max_lidar_distance)

	def draw(self, window, map_offset):
		"""Draw the live grid map, the color change according to the value of confidence, 1 means it is sure that a wall is there
		and drawn as a black square and 0 the opposite

		Args:
			window (surface): Surface on which to draw
		"""
		for idy in range(self.list_size):
			for idx in range(self.list_size):
				value = self.map[idy, idx]
				if value == 0:
					continue
				if value == -1:
					color = (70, 255, 200)
				else:
					gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (gray_scale, gray_scale, gray_scale)

				top_left_x, top_left_y, _, _ = self.ids_to_rect(idx, idy)
				# pygame.draw.circle(window, (255, 0, 0), (top_left_x, top_left_y), 2)
				# pygame.draw.circle(window, (255, 255, 0), self.pos.to_tuple(), 2)
				# pygame.display.update()				


				rect = pygame.Rect(top_left_x, top_left_y, self.size, self.size)
				pygame.draw.rect(window, color, rect)