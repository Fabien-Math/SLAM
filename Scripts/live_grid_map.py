from util import *
import numpy as np
import cv2
from PIL import Image

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
		offset_id = self.list_size//2
		p1x, p1y, p2x, p2y = 0, 0, 0, 0

		if 0 <= idx < self.list_size:
			p1x = (idx - offset_id) * self.size + self.centre.x
			p2x = (idx + 1 - offset_id) * self.size + self.centre.x
		if 0 <= idy < self.list_size:
			p1y = (idy - offset_id) * self.size + self.centre.y
			p2y = (idy + 1 - offset_id) * self.size + self.centre.y

		return p1x, p1y, p2x, p2y
	
	def ids_to_center(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		offset_id = self.list_size//2
		px, py = 0, 0

		if 0 <= idx < self.list_size:
			px = (idx + 0.5 - offset_id) * self.size + self.centre.x
		if 0 <= idy < self.list_size:
			py = (idy + 0.5 - offset_id) * self.size + self.centre.y

		return px, py
	
	def update_robot_path(self):
		# Add robot pos
		idx, idy = self.vec_to_ids(self.robot.pos_calc)
		# If ids are in the range of the live map
		if 0 < idx < self.list_size and 0 < idy < self.list_size:
			# Set to safe path
			self.map[idy, idx] = -1.01
	
	
	def update(self, points, max_lidar_distance, window):
		"""Update the live grid of the robot

		Args:
			points (list): List of points to be added, if None, update only robot pos
		"""
		
		if points is None:
			return
		
		new_cpt = 0

		# Add lidar points pos
		for point in points:
			idx, idy = self.coord_to_ids(point)
			# If ids are in the range of the live map
			if distance(self.robot.pos_calc, point) <= 1.5*max_lidar_distance:
				if 0 < idx < self.list_size and 0 < idy < self.list_size:
					self.map[idy, idx] = max(self.map[idy, idx], 0.5 + (1 - distance(self.robot.pos_calc, point)/max_lidar_distance) / 2)

				p1 = (point[0], point[1])
				new_cpt += self.fill_subdivision(p1, window)
			else:
				p1 = ((self.robot.pos_calc.x + point[0])/2, (self.robot.pos_calc.y + point[1])/2)
				new_cpt += self.fill_subdivision(p1, window)

		
	def points_in_safe_range_to_ids(self, p1, p2, safe_range):
		"""
		Get all list indices inside a rotated rectangle on a grid.

		Parameters:
			p1 (tuple): (x, y) coordinates of the starting point.
			p2 (tuple): (x, y) coordinates of the ending point.
			safe_range (float): Additional buffer around the rectangle.

		Returns:
			list: Indices of grid points inside the rotated rectangle.
		"""
		rotation = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
		line = compute_line_tuple(p1, p2)

		# Define perpendicular vector for width
		ortho_vec = compute_ortho_vec(line)
		# Distance between points
		length = distance_tuple(p1, p2)

		p1, p2 = np.array(p1), np.array(p2)
		half_width = np.array([safe_range * ortho_vec[0], safe_range * ortho_vec[1]])
		# Rectangle corners for safe range
		corners = np.array([
			p1 + half_width,
			p2 + half_width,
			p1 - half_width,
			p2 - half_width,
		])

		rotation_matrix = np.array([
			[np.cos(rotation), -np.sin(rotation)],
			[np.sin(rotation), np.cos(rotation)]
		])
		transposed_rot_matrix = rotation_matrix.T

		# Convert corners coordinates into live grid map coordinates
		ids = np.array([self.coord_to_ids(corner) for corner in corners])
		x_max = np.max(ids[:, 0])
		x_min = np.min(ids[:, 0])
		y_max = np.max(ids[:, 1])
		y_min = np.min(ids[:, 1])

		# Check grid points within the rotated rectangle
		inside_points = []
		for idx in range(x_min, x_max + 1):
			for idy in range(y_min, y_max + 1):
				# print("Point in safe range to ids", idx, idy)
				point = np.array(self.ids_to_center(idx, idy))
				# Transform the point into the local space of the rectangle
				local_point = np.dot(transposed_rot_matrix, point - p1)
				# print("Local point :", local_point)
				# Check if within bounds of the rectangle
				if (
					-safe_range <= local_point[0] <= length + safe_range and
					-safe_range <= local_point[1] <= safe_range
				):
					inside_points.append((idx, idy))

		return inside_points
			

	def fill_subdivision(self, p1, window):
		# Initialisation of subdivision list
		robot_pos = self.robot.pos_calc.to_tuple()
		line = compute_line_tuple(p1, robot_pos)

		new_cpt = 0

		p1_ids = self.coord_to_ids(p1)
		p2_ids = self.coord_to_ids(robot_pos)

		ids_line = move_on_line(p2_ids, p1_ids, p2_ids, p1_ids)
		for ids in ids_line:
			j, i = ids
			rect = self.ids_to_rect(*ids)
			is_in_rect = compute_segment_rect_intersection_tuple(line, p1, robot_pos, rect)

			# If the second point is in the box
			if is_in_rect:
				# If the distance between the robot and the rect wall is lower than the distince between the robot and the obstacle
				if distance_tuple(robot_pos, is_in_rect) < distance_tuple(robot_pos, p1):
					if self.map[i, j] <= 0:
						if self.map[i, j] <= -0.5:
							continue
						# Add known square cpt (allow to compute the discovery)
						new_cpt += 1
						self.map[i, j] = -1
						continue

		return new_cpt


	def find_frontiers(self):
		map_image = np.array((self.map + 1)*120, dtype=np.uint8)
		
		red_contours = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)[1]  # ensure binary
		all_contours = cv2.adaptiveThreshold(map_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 1)

		kernel = np.ones((3, 3), np.uint8)
		img_dilation = cv2.dilate(red_contours, kernel, iterations=1)

		sum_img = img_dilation + all_contours

		cv2.imwrite("Images/frontiers.png", sum_img)

		coords = np.column_stack(np.where(sum_img < 1))
		return coords


	def draw(self, window):
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
				elif value == -1:
					color = (70, 255, 200)
				elif value == -1.01:
					color = (35, 200, 180)
				else:
					gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (gray_scale, gray_scale//2, gray_scale//2)

				top_left_x, top_left_y, _, _ = self.ids_to_rect(idx, idy)
				# pygame.draw.circle(window, (255, 0, 0), (top_left_x, top_left_y), 2)
				# pygame.draw.circle(window, (255, 255, 0), self.pos.to_tuple(), 2)
				# pygame.display.update()				


				rect = pygame.Rect(top_left_x, top_left_y, self.size, self.size)
				pygame.draw.rect(window, color, rect)
			
	def save_map_to_image(self):
		def set_color(value):
			if value == 0:
				color = (0, 0, 0)
			elif value == -1:
				color = (200, 255, 70)
			elif value == -1.01:
				color = (180, 200, 35)
			else:
				gray_scale = min(255, max(0, 255 * (1 - value)))
				color = (gray_scale//2, gray_scale//2, gray_scale)
			return color
				
		map = [[set_color(value) for value in line] for line in self.map]
		map_image = np.array(map, dtype=np.uint8)
		cv2.imwrite("Images/map_explored.png", map_image)
