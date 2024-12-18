from math import pi, sin, cos, atan2
from util import *
from voronoi import build_voronoi_diagram, draw_voronoi_diagram, draw_polygon

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from robot import BeaconRobot  # Import uniquement pour l'annotation de type


class Controller:
	def __init__(self, robot, freq, mode):
		self.robot:BeaconRobot = robot

		self.mode = mode            # Mode 1 = Autonomous

		self.waypoints = []       # List of point to explore
		self.waypoint_radius = 10
		self.waypoint = None
		self.waypoint_reached = True

		self.local_waypoints = []
		self.local_waypoint_radius = 5
		self.local_waypoint = None
		self.local_waypoint_reached = True

		self.range_safe_path_computer = 100	# Range of the robot to find the new path with the voronoi


		self.no_safe_path_found_counter = 0


		self.voronoi_diagram = None
		self.thinned_voronoi_polygon = None

	def manage_waypoint_system(self, window):
		# Arrived at waypoint
		if self.waypoint:
			if point_in_circle(self.robot.pos_calc.to_tuple(), self.waypoint, self.waypoint_radius):
				self.waypoint_reached = True

		# Arrived at local waypoint
		if self.local_waypoint:
			if point_in_circle(self.robot.pos_calc.to_tuple(), self.local_waypoint, self.local_waypoint_radius):
				self.local_waypoint_reached = True


		### MANAGE WAYPOINT
		if not len(self.waypoints) and self.waypoint_reached:
			# Compute the new waypoint to go
			wp = find_new_waypoint(self.robot, window)

			# If the map is completely explored
			if wp == 1:
				self.mode = 100
				return

			# If the waypoint is accessible
			if wp:
				self.waypoints.append(wp)
				self.no_safe_path_found_counter = 0
			else:
				self.no_safe_path_found_counter += 1


		if len(self.waypoints) and self.waypoint_reached:
			self.waypoint = self.waypoints.pop(0)
			self.waypoint_reached = False
		
		### MANAGE LOCAL WAYPOINT
		
		# if self.waypoint and not len(self.local_waypoints) and self.local_waypoint_reached:
		# 	# Compute safe path
		# 	lwps = find_safe_path(self.waypoint, self.robot, self.range_safe_path_computer, window)
		# 	if lwps:
		# 		for lwp in lwps:
		# 			self.local_waypoints.append(lwp)

		# if len(self.local_waypoints) and self.local_waypoint_reached:
		# 	self.local_waypoint = self.local_waypoints.pop(0)
		# 	self.local_waypoint_reached = False

		if self.no_safe_path_found_counter > 100:
			print("No safe path found ! Exit the simulation, the domain must be explored !")
			exit()


	def move_to_waypoint(self, dt, window):
		"""Move to the next waypoint

		Args:
			dt (float): Time enlapsed
		"""
		# Point must be safe to go
		self.manage_waypoint_system(window)
		
		if self.waypoint:
			self.local_waypoint = find_safe_path(self.waypoint, self.robot, 1, window)[0]
		
		if self.local_waypoint is None:
			return
		
		# Correct angle to point to the local waypoint
		angle_diff = get_absolute_angle(self.local_waypoint,self.robot.pos_calc.to_tuple()) * 180 / pi - self.robot.rot_calc
		angle_diff %= 360
		if angle_diff > 180:
			angle_diff -= 360

		# Compute speed according to the distance from the local waypoint the robot is
		move_speed = max(0, min(distance(self.robot.pos_calc, self.local_waypoint)/100 + 0.2 - abs(angle_diff)/120, 1))
		rot_speed = max(min(angle_diff, 1), -1)
		rot_dir = sign(rot_speed)

		self.robot.move(dt, 1, move_speed)
		self.robot.rotate(dt, rot_dir, abs(rot_speed))
		
	

###   WAYPOINTS   ###

def is_safe_waypoint(pos, safe_radius, live_grid_map):
		ids_wp = live_grid_map.coord_to_ids(pos)

		# Check the box around the known map of the robot to see if an obstacle is there
		for i in range(ids_wp[1] - safe_radius, ids_wp[1] + safe_radius + 1):
			for j in range(ids_wp[0] - safe_radius, ids_wp[0] + safe_radius + 1):
				if live_grid_map.map[i, j] > 0.1:
					return False
		
		return True


def find_new_waypoint(robot, window):
	"""Find the next location to be explored

	Args:
		live_grid_map (Live_grid_map): Live grid map of the robot
		window (surface): Surface on which to draw
	"""
	open_boundaries = robot.live_grid_map.find_frontiers()

	if not len(open_boundaries):
		print("Goal achieved ! \nNothing more to explore :)")
		return 1

	# Connections tables of all frontiers pixels
	connection_table = make_connection_table(open_boundaries)
	# Connected lines of indices of frontiers pixels
	c_lines_ids = connect_lines_ids(connection_table, open_boundaries)
	# Connected lines of frontiers pixels
	c_lines = convert_lines_ids_to_pixels(c_lines_ids, open_boundaries)

	# Order pixel lines
	ordered_lines = order_lines(c_lines, c_lines)

	# If no line, then the map is entierly explored
	if len(ordered_lines) == 0:
		if len(c_lines):
			for c_line in c_lines:
				if len(c_line) > 4:
					pos = (robot.pos_calc.x, robot.pos_calc.y - 50)
					return pos
		print("Goal achieved ! \nNothing more to explore :)")
		return 1
	

	live_grid_map = robot.live_grid_map

	# Variable to say if a waypoint is accessible and safe
	safe_waypoint = False
	n = -1

	# Find center of edges
	center_frontiers = [line[len(line)//2] for line in ordered_lines]

	while not safe_waypoint:
		# Iterate in the first place
		n += 1
		
		# Safe counter
		if n > 100:
			print("No safe path found !!")
			break

		# Convert and extract position from live grid map
		point_id, (idy, idx) = find_best_ids_point_angle(robot, center_frontiers, window)
		pos = live_grid_map.ids_to_rect(idx, idy)[0:2]
		
		safe_waypoint = is_safe_waypoint(pos, 1, live_grid_map)
		if safe_waypoint:
			return pos
				
		center_frontiers.pop(point_id)
		if not len(center_frontiers):
			return None

	return None


def find_safe_path(waypoint, robot, c_range, window):
	"""The goal would be to develop an algorithm to avoid obstacle but guaranty that the robot go to the point

	First method would be an AI
	"""
	if point_in_circle(waypoint, robot.pos_calc.to_tuple(), robot.lidar.max_dist):
		return [waypoint]


	lidar_data = robot.lidar.data
	points = [(robot.pos_calc.x + dist/2*cos(ang), robot.pos_calc.y + dist/2*sin(ang)) for dist, ang in lidar_data if dist > 1.5 * robot.lidar.max_dist]

	# for point in points:
	# 	pygame.draw.line(window, (0, 255, 255), robot.pos_calc.to_tuple(), point, 2)
	# 	pygame.display.update()
	# 	pygame.time.delay(10)
	# Variable to say if a waypoint is accessible and safe
	safe_waypoint = False
	n = -1
	
	while not safe_waypoint:
		# Iterate in the first place
		n += 1
		
		# Safe counter
		if n > 100:
			print("No safe path found !!")
			break

		# Convert and extract position from live grid map
		point_id, pos = find_best_point_dist(robot, points, waypoint)
		
		safe_waypoint = is_safe_waypoint(pos, 1, robot.live_grid_map)
		if safe_waypoint:
			ids = robot.live_grid_map.coord_to_ids(pos)
			pos = robot.live_grid_map.ids_to_center(*ids)
			return [pos]
				
		points.pop(point_id)
		if not len(points):
			return None



	return [waypoint]


def dfs(start_id, ids, visited):
	"""Depth-First Search algorithme to go through all connected ids

	Args:
		start_id (int): Start index for going through the line
		ids (list): List of indices
		visited (list): List of visited nodes

	Returns:
		list: List of connected nodes
	"""
	stack = [start_id]
	nodes = []
	while len(stack):
		node = stack.pop()
		if not visited[node]:
			visited[node] = True
			nodes.append(node)
			
			for n_node in ids[node]:
				if not visited[n_node]:
					stack.append(n_node)
	return nodes


def make_connection_table(ids:list):
	"""Given a list of ids, find all the connection between ids

	Args:
		ids (list): List of index [[idx0, idy0], [idx1, idy1], ...]

	Returns:
		list: List of connection between nodes
	"""
	connection_table = [[] for _ in range(len(ids))]

	for i in range(len(ids)):
		for j in range(i+1, len(ids)):
			# Horizontal connections
			if abs(ids[i][0] - ids[j][0]) < 2 and ids[i][1] == ids[j][1]:
				connection_table[i].append(j)
				connection_table[j].append(i)
			# Vertical connections
			if abs(ids[i][1] - ids[j][1]) < 2 and ids[i][0] == ids[j][0]:
				connection_table[i].append(j)
				connection_table[j].append(i)
		
	return connection_table
	

def connect_lines_ids(connection_table:list, ids:list):
	"""Take a list of 2D indices (could represent pixels) and connect them into separate list representing connected lines

	Args:
		ids (list): List of list of indices index (pixel) for each line
	"""	
	
	# Keep track of visited nodes
	visited = [False] * len(ids)

	c_ids = []
	for i in range(len(ids)):
		if not visited[i]:
			c_ids.append(dfs(i, connection_table, visited))
	
	return c_ids


def convert_lines_ids_to_pixels(c_ids:list, ids:list):
	"""Take a list of indices (could represent index of pixels) and convert to 2D ids (pixel positions)

	Args:
		ids (list): List of list of 2D ids (pixel positions) 
	"""	
	c_lines = [[[] for _ in range(len(c))] for c in c_ids]
	for i in range(len(c_ids)):
		for j in range(len(c_ids[i])):
			c_lines[i][j] = ids[c_ids[i][j]]

	return c_lines


def order_lines(connected_lines:list, ids_lines:list):
	"""Take a list of connected lines and order indices (pixels) from line start to line end

	Args:
		connected_lines (list): List of connected lines of ordered indices
	"""

	ordonned_lines = []
	for line, ids_line in zip(connected_lines, ids_lines):
		c_table = make_connection_table(line)
		if len(c_table) < 3:
			continue

		start_ids = []
		for i, c_id in enumerate(c_table):
			if len(c_id) == 1:
				start_ids.append(i)

		visited = [False] * len(c_table)
		while i < len(start_ids):
			s_id = start_ids[i]

			next_id = c_table[s_id][0]
			if len(c_table[next_id]) > 2:
				visited[next_id] = True
				visited[s_id] = True
				start_ids.remove(s_id)
				i -= 1
			i += 1

		# Avoid squares
		if not len(start_ids):
			continue

		line_ids = dfs(start_ids[0], c_table, visited)
		ordonned_lines.append([ids_line[p_id] for p_id in line_ids])
	return ordonned_lines


def find_best_ids_point_angle(robot, center_frontier_points:list, window):
	robot_ids_pos = robot.live_grid_map.coord_to_ids(robot.pos_calc.to_tuple())
	robot_idx_pos, robot_idy_pos = robot_ids_pos
	best_point_id = 0
	ang_min = 1000


	for i, point in enumerate(center_frontier_points):
		idx, idy = point[1], point[0]
		
		# In the reference of the robot
		didx, didy = (idx - robot_idx_pos, idy - robot_idy_pos)
		# p1 must be of form (-y, x) like point is
		robot_rot_point_angle = atan2(didy, didx) * 180 / pi
		
		ang = abs(robot_rot_point_angle - robot.rot_calc)%360
		ang = ang * (ang < 180) + (360 - ang) * (ang >= 180)
		
		if ang < ang_min:
			ang_min = ang
			best_point_id = i
			
	
	return best_point_id, center_frontier_points[best_point_id]


def find_best_point_dist(robot, points:list, waypoint):
	best_point_id = 0
	dist_min = 1000

	robot_pos = robot.pos_calc.to_tuple()

	for i, point in enumerate(points):
		dist1 = distance_tuple(robot_pos, point)
		dist2 = distance_tuple(waypoint, point)
		dist = dist1 + dist2
		
		if dist < dist_min:
			dist_min = dist
			best_point_id = i
			
	return best_point_id, points[best_point_id]
