import util as ut
import numpy as np
import cv2
from PIL import Image
import pygame

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from robot import BeaconRobot  # Import only for editor, avoid circular import



class Live_grid_map():
	def __init__(self, robot, list_size, size):
		self.robot:BeaconRobot = robot

		self.centre = (600, 350)
		self.list_size = list_size
		self.size = size
		self.map = np.zeros((self.list_size, self.list_size), dtype=np.int16)
		self.occurance_map = np.zeros((self.list_size, self.list_size), dtype=np.int16)
		
		self.updated_cells = {}
		self.saved_map = None

	def coord_to_ids(self, point):
		idx = int((point[0] - self.centre[0])/self.size + self.list_size//2)
		idy = int((point[1] - self.centre[1])/self.size + self.list_size//2)
		return idx, idy

	def ids_to_rect(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		offset_id = self.list_size//2
		p1x, p1y, p2x, p2y = 0, 0, 0, 0

		if 0 <= idx < self.list_size:
			p1x = (idx - offset_id) * self.size + self.centre[0]
			p2x = (idx + 1 - offset_id) * self.size + self.centre[0]
		if 0 <= idy < self.list_size:
			p1y = (idy - offset_id) * self.size + self.centre[1]
			p2y = (idy + 1 - offset_id) * self.size + self.centre[1]

		return p1x, p1y, p2x, p2y
	
	def ids_to_center(self, idx, idy):
		"""Give the subdiv rectangle coordinante where the point is

		Returns:
			rect (tuple): (x1, y1, x2, y2) where p1 is top-left and p2 is bottom-right
		"""
		offset_id = self.list_size//2
		px, py = 0, 0

		if 0 <= idx < self.list_size:
			px = (idx + 0.5 - offset_id) * self.size + self.centre[0]
		if 0 <= idy < self.list_size:
			py = (idy + 0.5 - offset_id) * self.size + self.centre[1]

		return px, py
	
	def update_robot_path(self):
		# Add robot pos
		idx, idy = self.coord_to_ids(self.robot.pos_calc)
		# If ids are in the range of the live map
		if 0 < idx < self.list_size and 0 < idy < self.list_size:
			# Set to safe path
			self.map[idy, idx] = 19
			self.updated_cells[(idx, idy)] = 1
	
	
	def update(self, collide_points, no_collide_points, max_lidar_distance, window):
		"""Update the live grid of the robot

		Args:
			points (list): List of points to be added, if None, update only robot pos
		"""
		
		if collide_points is None or no_collide_points is None:
			return
		
		new_cpt = 0
		
		# Add lidar points pos
		for point in no_collide_points:
			p1 = ((self.robot.pos_calc[0] + point[0])/2, (self.robot.pos_calc[1] + point[1])/2)
			new_cpt += self.fill_subdivision(p1, intercept=False, window=window)
		
		for point in collide_points:
			new_cpt += self.fill_subdivision(point, intercept=True, window=window)

			# If ids are in the range of the live map
			idx, idy = self.coord_to_ids(point)
			if 0 < idx < self.list_size and 0 < idy < self.list_size:
				new_value = max(self.map[idy, idx], 100 + int(100 * (1 - ut.distance(self.robot.pos_calc, point)/max_lidar_distance)))
				self.occurance_map[idy, idx] = min(self.occurance_map[idy, idx] + 3, 200)
				if self.map[idy, idx] != new_value:
					self.map[idy, idx] = new_value
					self.updated_cells[(idx, idy)] = 1


		
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
		line = ut.compute_line(p1, p2)

		# Define perpendicular vector for width
		ortho_vec = ut.compute_ortho_vec(line)
		# Distance between points
		length = ut.distance(p1, p2)

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
			

	def fill_subdivision(self, p1, intercept, window):
		# Initialisation of subdivision list
		robot_pos = self.robot.pos_calc
		line = ut.compute_line(p1, robot_pos)

		new_cpt = 0

		p1_ids = self.coord_to_ids(p1)
		p2_ids = self.coord_to_ids(robot_pos)

		ids_line = ut.move_on_line(p2_ids, p1_ids, p2_ids, p1_ids)
		for ids in ids_line:
			j, i = ids
			rect = self.ids_to_rect(*ids)
			is_in_rect = ut.compute_segment_rect_inter(line, p1, robot_pos, rect)

			if self.coord_to_ids(p1) == ids:
				break

			if self.map[i, j] == 19:
				continue
			if self.occurance_map[i, j] > 10 and self.map[i, j] != 200:
				self.map[i, j] = 200
				self.updated_cells[j, i] = 1
				continue

			# If the second point is in the box
			if is_in_rect:
				# If the distance between the robot and the rect wall is lower than the distance between the robot and the obstacle
				if not intercept:
					if ut.distance(robot_pos, is_in_rect) < ut.distance(robot_pos, p1) - 2 * self.size:
						new_value = max(0, self.occurance_map[i, j]-1)
						if self.occurance_map[i, j] != new_value:
							self.occurance_map[i, j] = new_value
							self.updated_cells[j, i] = 1
				if ut.distance(robot_pos, is_in_rect) < ut.distance(robot_pos, p1) - 1.42 * self.size:
					if self.occurance_map[i, j] == 0 and self.map[i, j] != 20:
						self.map[i, j] = 20
						self.updated_cells[j, i] = 1
						continue
					if self.map[i, j] == 0:
						# Add known square cpt (allow to compute the discovery score)
						new_cpt += 1
						self.map[i, j] = 20
						self.updated_cells[j, i] = 1
						continue

		return new_cpt


	def find_frontiers(self):
		mask = self.map == 19
		map_image = np.array(self.map, dtype=np.uint8)
		map_image[mask] += 1
		
		red_contours = cv2.threshold(map_image, 99, 255, cv2.THRESH_BINARY)[1]  # ensure binary
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
				elif value == 20:
					color = (130, 240, 130)
				elif value == 19:
					color = (35, 200, 180)
				else:
					# gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (value, value//2, value)

				top_left_x, top_left_y, _, _ = self.ids_to_rect(idx, idy)
				# pygame.draw.circle(window, (255, 0, 0), (top_left_x, top_left_y), 2)
				# pygame.draw.circle(window, (255, 255, 0), self.pos, 2)
				# pygame.display.update()				


				rect = pygame.Rect(top_left_x, top_left_y, self.size, self.size)
				pygame.draw.rect(window, color, rect)

	def draw_occurance(self, window):
		"""Draw the live grid map, the color change according to the value of confidence, 1 means it is sure that a wall is there
		and drawn as a black square and 0 the opposite

		Args:
			window (surface): Surface on which to draw
		"""
		for idy in range(self.list_size):
			for idx in range(self.list_size):
				value = self.occurance_map[idy, idx]
				if value == 0:
					continue
				else:
					# gray_scale = min(255, max(0, 255 * (1 - value)))
					color = (value//2, 255, value)

					print(value)

				top_left_x, top_left_y, _, _ = self.ids_to_rect(idx, idy)
				# pygame.draw.circle(window, (255, 0, 0), (top_left_x, top_left_y), 2)
				# pygame.draw.circle(window, (255, 255, 0), self.pos, 2)
				# pygame.display.update()				


				rect = pygame.Rect(top_left_x, top_left_y, self.size, self.size)
				pygame.draw.rect(window, color, rect)
			
	def save_map_to_image(self):
		def set_color(value):
			if value == 0:
				color = (0, 0, 0)
			elif value == 20:
				color = (130, 240, 130)
			elif value == 19:
				color = (180, 200, 35)
			else:
				color = (value, value//2, value)
			return color
				
		map = [[set_color(value) for value in line] for line in self.map]
		map_image = np.array(map, dtype=np.uint8)
		cv2.imwrite("Images/map_explored.png", map_image)
