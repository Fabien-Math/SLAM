from math import pi, sin, cos, atan2, ceil
import util as ut
import numpy as np
import pygame

from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import BeaconRobot  # Import uniquement pour l'annotation de type


class Controller:
	def __init__(self, robot, mode):
		self.robot:BeaconRobot = robot
		# Controller mode
		self.mode = mode            # Mode 1 = Autonomous

		### Waypoint
		# List of waypoint
		self.waypoints = []
		# Radius from the wp to validate its visit
		self.waypoint_radius = 10
		self.waypoint = None
		self.waypoint_reached = True

		### Local waypoint
		# List of local waypoint to acheive waypoint
		self.local_waypoints = []
		# Radius from the wp to validate its visit
		self.local_waypoint_radius = 10
		self.local_waypoint = None
		self.local_waypoint_reached = True

		# Number of time no path is found to go to the wp
		self.no_safe_path_found_counter = 0

		# Last position, check robot deadlock
		self.last_static_pos = self.robot.pos.to_tuple()
		# Last time the robot is known to move outside a deadlock
		self.last_time_static_pos = 0
		# Boolean to asked for global path planning in case of a deadlock
		self.need_global_path = False


	def manage_waypoint_system(self, window):
		"""Manage the order of waypoints and local waypoints to set them properly
		"""

		if self.waypoint:
			# Arrived at waypoint
			if ut.point_in_circle(self.robot.pos_calc.to_tuple(), self.waypoint, self.waypoint_radius):
				self.waypoint_reached = True

			# If the waypoint is too close to a wall
			if not is_safe_waypoint(self.waypoint, 2 * self.robot.radius, self.robot.live_grid_map):
				self.waypoint_reached = True
				self.local_waypoint_reached = True
				self.local_waypoints = []
			
		if self.local_waypoint:
			# Arrived at local waypoint
			if ut.point_in_circle(self.robot.pos_calc.to_tuple(), self.local_waypoint, self.local_waypoint_radius):
				self.local_waypoint_reached = True

			# # If the local waypoint is too close to a wall, could be acheive when the wall isn't still discovered
			# if not is_safe_waypoint(self.local_waypoint, 2 * self.robot.radius, self.robot.live_grid_map):
			# 	self.local_waypoint_reached = True
			# 	self.local_waypoints = []

			# If the robot need to cross a wall to go to the local waypoint, could be acheive when the wall isn't still discovered
			if need_line_split(self.robot.live_grid_map, self.robot.pos.to_tuple(), self.local_waypoint, 2 * self.robot.radius):
				self.local_waypoint_reached = True
				self.local_waypoints = []
				

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
		if self.waypoint and not len(self.local_waypoints) and self.local_waypoint_reached:

			if self.need_global_path:
				print("Global path needed")
				lwps = find_global_path(self.robot, self.waypoint, self.robot.radius * 2, window)
				# Reset the demand
				self.need_global_path = False
			else:
				# Compute safe next local waypoint
				lwps = find_safe_path(self.waypoint, self.robot, self.robot.radius * 2, window)
			
			# If no path found or going to the point is impossible
			if lwps is None or lwps[0] == 1:
				self.waypoint_reached = True
				self.local_waypoint_reached = True
				return
			
			if lwps:
				for lwp in lwps:
					self.local_waypoints.append(lwp)

		if len(self.local_waypoints) and self.local_waypoint_reached:
			self.local_waypoint = self.local_waypoints.pop(0)
			self.local_waypoint_reached = False

		# If no path is found several times, exit
		if self.no_safe_path_found_counter > 30:
			print("No safe path found ! Exit the simulation, the domain must be explored !")
			self.mode = 100
			return
		
	def check_if_in_deadlock(self, dt):
		if not ut.point_in_circle(self.robot.pos.to_tuple(), self.last_static_pos, 50):
			self.last_static_pos = self.robot.pos.to_tuple()
			self.last_time_static_pos = 0
		else:
			self.last_time_static_pos += dt
		
		if self.last_time_static_pos > 5:
			print("\n#####   Robot is in a deadlock   #####\n")
			self.local_waypoint_reached = True
			self.local_waypoints = []
			self.need_global_path = True
			self.last_time_static_pos = 0
			return True
		return False



	def move_to_waypoint(self, dt, window):
		"""Move to the next waypoint

		Args:
			dt (float): Time enlapsed
		"""

		# Before planning path, check if in a deadlock
		self.check_if_in_deadlock(dt)

		# Point must be safe to go
		self.manage_waypoint_system(window)

		# If no local waypoint to go
		if self.local_waypoint is None:
			print("No local waypoint !")
			return
		
		# Correct angle to point to the local waypoint
		angle_diff = ut.get_absolute_angle(self.local_waypoint,self.robot.pos_calc.to_tuple()) * 180 / pi - self.robot.rot_calc
		angle_diff %= 360
		if angle_diff > 180:
			angle_diff -= 360

		# Compute speed and rotation speed according to the distance and the angle from the local waypoint the robot is
		move_speed = max(0, min(ut.distance(self.robot.pos_calc, self.local_waypoint)/100 + 0.2 - abs(angle_diff)/20, 1))
		# move_speed = max(0, min(ut.distance(self.robot.pos_calc, self.local_waypoint)/100 + 0.5 - abs(angle_diff)/60, 1))
		rot_speed = max(min(angle_diff, 1), -1)
		rot_dir = ut.sign(rot_speed)

		# Move and rotate the robot
		self.robot.move(dt, 1, move_speed)
		self.robot.rotate(dt, rot_dir, abs(rot_speed))
		
	

###   WAYPOINTS   ###
def is_safe_waypoint(wp_pos, safe_range, live_grid_map) -> bool:
	"""Check if the waypoint given is safe, ie not too close to a wall

	Args:
		wp_pos (tuple): Position of the waypoint
		safe_range (float): Distance from which the wp have to be to the wall
		live_grid_map (_type_): Feature map of the robot

	Returns:
		bool: Return if the wp is safe or not
	"""
	ids_wp = live_grid_map.coord_to_ids(wp_pos)
	ids_range = ceil(safe_range / live_grid_map.size)

	# Check the box around the known map of the robot to see if an obstacle is there
	for i in range(ids_wp[1] - ids_range, ids_wp[1] + ids_range + 1):
		for j in range(ids_wp[0] - ids_range, ids_wp[0] + ids_range + 1):
			if live_grid_map.map[i, j] > 99:
				return False
			
	return True


def find_frontiers(live_grid_map, window):
	"""Find the next location to be explored  
		It works by finding all the frontiers of the robot feature map (robot known map), 
		extract connected walls, order lines by ascending order of length, find the start
		and the end of a wall

	Args:
		live_grid_map (Live_grid_map): Live grid map of the robot
		window (surface): Surface on which to draw

	Returns:
		list: List of ordered frontiers lines
	"""
	open_boundaries = live_grid_map.find_frontiers()

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
	# ordered_lines = order_lines(c_lines)

	return c_lines

def find_new_waypoint(robot, window):
	"""Find a safe new waypoint from the open frontiers

	Args:
		robot (BeaconRobot): Robot

	Returns:
		tuple||None: A new safe waypoint if it exists
	"""
	live_grid_map = robot.live_grid_map

	c_lines = find_frontiers(live_grid_map, window)

	# If no line, then the map is entierly explored
	if not len(c_lines):
		print("Goal achieved ! \nNothing more to explore :)")
		return 1
		
	# Variable to say if a waypoint is accessible and safe
	safe_waypoint = False
	n = 0

	center_frontiers = list(np.concatenate([[point for point in line] for line in c_lines]))

	while not safe_waypoint:
		# Iterate in the first place
		n += 1
		
		# Safe counter
		if n > 100:
			print("No safe waypoint found !!")
			break

		# Convert and extract position from live grid map
		point_id, (idy, idx) = find_best_ids_point_angle(robot, center_frontiers, window)
		pos = live_grid_map.ids_to_center(idx, idy)
		
		safe_waypoint = is_safe_waypoint(pos, robot.radius*2, live_grid_map)
		if safe_waypoint:
			return pos
				
		center_frontiers.pop(point_id)
		if not len(center_frontiers):
			return None

	return None

def need_line_split(live_grid_map, p1, p2, safe_range):
	"""Determines if the straight line between two points (p1 and p2) is obstructed by obstacles 
	or too close to them within a specified safety range.

	This function checks the line segment between `p1` and `p2` for safety by evaluating 
	points along the line in the robot's live grid map. If any point along the line falls within 
	the obstacle range, the function indicates that the line should be split.

	Args:
		live_grid_map (Live_grid_map): The robot's live grid map containing obstacle information.
		p1 (tuple): The starting point of the line as (x, y).
		p2 (tuple): The ending point of the line as (x, y).
		safe_range (float): The minimum distance from obstacles required to consider the path safe.

	Returns:
		bool: 
			- `True` if the line between `p1` and `p2` is obstructed or too close to obstacles.
			- `False` if the line is clear and safe.

	Notes:
		- The function uses `points_in_safe_range_to_ids` to determine the grid cell IDs 
		  along the line that need to be evaluated for safety.
		- The grid map values (`live_grid_map.map`) are checked:
			- Values greater than 0 indicate the presence of an obstacle.
		- The safety check ensures that all points along the line maintain a clearance of at least `safe_range`.

	"""
	
	map_line_ids = live_grid_map.points_in_safe_range_to_ids(p1, p2, safe_range)
	for ids in map_line_ids:
		if live_grid_map.map[ids[1], ids[0]] > 99:
			return True	
	return False

def split_line(live_grid_map, map_size, p1, p2, line, safe_range, imposed_sign = 0,	window = None):
	"""Splits the line segment between two points (p1 and p2) and identifies a safe point normal 
	to the line, considering obstacles and a specified safe distance.

	The function calculates two potential midpoint candidates (up and down) along the line's 
	normal vector. It iteratively projects these points outward until a "safe" point is found, 
	defined as a point sufficiently far from obstacles in the grid map. The direction of projection 
	can be constrained using the `imposed_sign` parameter.

	Args:
		live_grid_map (Live_grid_map): Live grid map object containing the obstacle map.
		map_size (tuple): Size of the virtual map as (width, height).
		p1 (tuple): Starting point of the line segment (x1, y1).
		p2 (tuple): Ending point of the line segment (x2, y2).
		line (tuple): Line equation coefficients (a, b, c) for ax + by = c.
		safe_range (float): Minimum distance the safe point should maintain from obstacles.
		imposed_sign (int, optional): Specifies projection direction. 
									  1 for upward projection, -1 for downward projection, 
									  and 0 for both directions. Defaults to 0.
		window (optional): Placeholder for a visualization window, not used in the current implementation.

	Returns:
		tuple: A safe point (x, y) normal to the line, or None if no such point is found.

	Notes:
		- The function uses a grid-based approach to evaluate safety.
		- Safety is determined by ensuring the area around a candidate point 
		  (defined by `safe_range`) is free from obstacles.
		- The search stops after a maximum of 100 iterations or when a safe point is found.
		- If both candidates (upward and downward) are invalid, the function returns None.
	"""
	
	map = live_grid_map.map
	a, b, _ = line	# ax + by = c
	if a:
		ortho_vec = ut.normalize_vec((a, b))
	else:
		ortho_vec = (0, ut.sign(b)) 
	
	# Up middle point
	mdp_1 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	# Down middle point
	mdp_2 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	match imposed_sign:
		case 1:
			mdp_2 = None
		case -1:
			mdp_1 = None

	cpt = 0
	ids_range = ceil(safe_range / map.size)
	while True and cpt < 100:
		cpt += 1
		if mdp_1:
			# Normal projection from the line
			mdp_1 = (mdp_1[0] + 0.5 * safe_range * ortho_vec[0], mdp_1[1] + 0.5 * safe_range * ortho_vec[1])
			ids_mdp1 = live_grid_map.coord_to_ids(mdp_1)

			safe = True
			for idx in range(ids_mdp1[0] - ids_range, ids_mdp1[0] + ids_range + 1):
				for idy in range(ids_mdp1[1] - ids_range, ids_mdp1[1] + ids_range + 1):
					if map[idy][idx] > 100:
						safe = False
						break
			if safe:
				return mdp_1

			if not ut.point_in_box_tuple(mdp_1, (0, 0), map_size):
				mdp_1 = None
			

		if mdp_2:
			mdp_2 = (mdp_2[0] - 0.25 * safe_range * ortho_vec[0], mdp_2[1] - 0.25 * safe_range * ortho_vec[1])
			ids_mdp2 = live_grid_map.coord_to_ids(mdp_2)

			safe = True
			for idx in range(ids_mdp2[0] - ids_range, ids_mdp2[0] + ids_range + 1):
				for idy in range(ids_mdp2[1] - ids_range, ids_mdp2[1] + ids_range + 1):
					if map[idy][idx] > 100:
						safe = False
						break
			if safe:
				return mdp_2

			if not ut.point_in_box_tuple(mdp_2, (0, 0), map_size):
				mdp_2 = None
		

		if mdp_1 is None and mdp_2 is None:
			return None
	
	return None

def find_next_local_waypoint(map, robot_pos, waypoint_pos, safe_range, window = None):
	"""Determines the next safe local waypoint for a robot to navigate towards a target waypoint, 
	considering obstacles and a specified safety distance.

	The function iteratively checks if the direct line between the robot's current position 
	and the target waypoint is safe. If the path is obstructed or too close to obstacles, 
	it splits the line and searches for an intermediate safe point. The process continues until 
	either a safe waypoint is found or no viable path exists within the specified safe range.

	Args:
		map (Live_grid_map): Grid-based obstacle map representing the robot's environment.
		robot_pos (tuple): Current position of the robot as (x, y).
		waypoint_pos (tuple): Target waypoint position as (x, y).
		safe_range (float): Minimum safe distance to maintain from obstacles.
		window (optional): Placeholder for a visualization window, not used in the current implementation.

	Returns:
		tuple: Coordinates of the next safe local waypoint (x, y) if found.
		int: Returns 1 if no safe path is available within the given safe range.

	Notes:
		- The function operates within a fixed map size of (1100, 600).
		- Safety is determined by evaluating the need to split the line (robot to waypoint) 
		  and iteratively finding a safe midpoint using `split_line`.
		- The search halts after a maximum of 100 iterations to prevent infinite loops.
		- If the direct path is safe, the function immediately returns the target waypoint.
	"""

	map_size = (1100, 600)

	cpt = 0
	safe_local_waypoint = waypoint_pos
	while True and cpt < 100:
		cpt += 1
		if need_line_split(map, robot_pos, safe_local_waypoint, safe_range):
			line = ut.compute_line_tuple(robot_pos, safe_local_waypoint)
			mdp = split_line(map, map_size, robot_pos, safe_local_waypoint, line, safe_range, 0)
			if mdp:
				safe_local_waypoint = mdp
			else:
				print(f"No path available with this safe range : {safe_range}")
				return 1
					
		else:
			return safe_local_waypoint
			
	return



### NEED TO BE CHANGE
def find_global_path(robot, waypoint_pos, safe_range, window = None):
	robot_pos = robot.pos.to_tuple()
	map = robot.live_grid_map
	valid_sub_wp = [robot_pos, waypoint_pos]
	map_size = (1100, 600)


	cpt = 0
	while True and cpt < 100:
		cpt += 1

		new_valid_sub_wp = [robot_pos]
		line_splited = False

		for i in range(len(valid_sub_wp) - 1):
			p1 = valid_sub_wp[i]
			p2 = valid_sub_wp[i+1]
			line = ut.compute_line_tuple(p1, p2)
			if need_line_split(map, p1, p2, safe_range):
				mdp = split_line(map, map_size, p1, p2, line, safe_range)
				if mdp:
					if mdp != p2:
						line_splited = True
						new_valid_sub_wp.append(mdp)
				else:
					print(f"No path available with this safe range : {safe_range}")
					return
			new_valid_sub_wp.append(p2)
				
		valid_sub_wp = new_valid_sub_wp[:]

		if not line_splited:
			return valid_sub_wp[1::]
	
	return


def find_safe_path(waypoint, robot, safe_range, window):
	"""Determines a safe path for the robot to reach a specified waypoint while avoiding obstacles.

	This function serves as a high-level path-planning algorithm. It ensures the robot navigates 
	toward the target waypoint, avoiding obstacles within a defined safety range. The function 
	relies on `find_next_local_waypoint` to iteratively compute intermediate waypoints 
	that are safe for traversal.

	Args:
		waypoint (tuple): Target waypoint coordinates as (x, y) that the robot must reach.
		robot (Robot): The robot object containing its position (`robot.pos`) and live grid map (`robot.live_grid_map`).
		safe_range (float): Minimum distance the robot should maintain from obstacles.
		window (optional): Visualization window for debugging or monitoring purposes (not used in this implementation).

	Returns:
		list: A list containing the next safe waypoint as (x, y). 
			  Returns `None` if no viable path can be found to reach the target within the safety constraints.

	Notes:
		- This function guarantees that the robot progresses toward the waypoint while maintaining safety.
		- If no safe local waypoint can be found (e.g., due to obstacles blocking all possible paths), 
		  the function returns `None` to indicate failure.
		- The underlying safety and navigation logic is handled by `find_next_local_waypoint`.
	"""

	safe_local_waypoint = find_next_local_waypoint(robot.live_grid_map, robot.pos.to_tuple(), waypoint, safe_range, window)

	if safe_local_waypoint:
		return [safe_local_waypoint]

	return None



def draw_local_waypoint(window, local_waypoint_pos, delay):
	color = (100, 255, 0)
	pygame.draw.circle(window, color, local_waypoint_pos, 3)
	pygame.display.update()
	pygame.time.delay(delay)




def dfs(start_id, ids, visited):
	"""Implementation of the Depth-First Search (DFS) algorithm to traverse and find all nodes 
	connected to a starting node in a graph.

	This function explores a graph represented by adjacency lists and returns all 
	nodes connected to the starting node. It uses a stack-based iterative approach 
	to avoid recursion limits in large graphs.

	Args:
		start_id (int): The index of the starting node for the traversal.
		ids (list): Adjacency list representing the graph. 
					Each element `ids[i]` is a list of nodes connected to node `i`.
		visited (list): A boolean list where `visited[i]` indicates whether node `i` 
						has been visited.

	Returns:
		list: A list of all nodes connected to the starting node, in the order they are visited.

	Notes:
		- The function uses a stack for iterative DFS to avoid recursion.
		- It marks nodes as visited in the `visited` list to prevent revisiting them.
		- The adjacency list `ids` should be a complete representation of the graph.
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
	"""Constructs a connection table that represents the connectivity between a list of nodes 
	based on their proximity in a 2D grid.

	Args:
		ids (list): A list of indices representing nodes in a 2D grid. 
					Each element is a list or tuple of coordinates [x, y].

	Returns:
		list: An adjacency list where each element is a list of connected nodes for the corresponding node in `ids`.
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


def order_lines(connected_lines:list):
	"""Take a list of connected lines and order indices (pixels) from line start to line end

	Args:
		connected_lines (list): List of connected lines of ordered indices
	"""

	ordonned_lines = []
	for line in connected_lines:
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
		ordonned_lines.append([line[p_id] for p_id in line_ids])
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
		dist1 = ut.distance_tuple(robot_pos, point)
		dist2 = ut.distance_tuple(waypoint, point)
		dist = dist1 + dist2
		
		if dist < dist_min:
			dist_min = dist
			best_point_id = i
			
	return best_point_id, points[best_point_id]
