from math import pi, sin, cos, atan2, ceil
import util as ut
import numpy as np
import display as disp
from time import perf_counter

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
		self.local_waypoint_radius = 5
		self.local_waypoint = None
		self.local_waypoint_reached = True

		# Number of time no path is found to go to the wp
		self.no_safe_path_found_counter = 0

		self.safe_range = self.robot.radius * 2
		self.emergency_mode = False
		self.global_path_mode = False

		self.time = 0
		self.times = []
		# Last position, check robot deadlock
		self.last_static_pos = self.robot.pos
		# Last time the robot is known to move outside a deadlock
		self.last_time_static_pos = 0
		# Boolean to asked for global path planning in case of a deadlock
		self.need_global_path = False


	def manage_waypoint_system(self, window):
		"""Manage the order of waypoints and local waypoints to set them properly
		"""

		if self.waypoint:
			# Arrived at waypoint
			if ut.point_in_circle(self.robot.pos_calc, self.waypoint, self.waypoint_radius):
				if self.emergency_mode:
					self.emergency_mode = False
				if self.global_path_mode:
					self.global_path_mode = False
				self.waypoint_reached = True
				self.local_waypoint_reached = True
				self.local_waypoints = []

			# If the waypoint is too close to a wall
			if not self.emergency_mode:
				if not is_safe_waypoint(self.waypoint, self.safe_range, self.robot.live_grid_map):
					self.waypoint_reached = True
					self.local_waypoint_reached = True
					self.local_waypoints = []
			
		if self.local_waypoint:
			# Arrived at local waypoint
			if ut.point_in_circle(self.robot.pos_calc, self.local_waypoint, self.local_waypoint_radius):
				self.local_waypoint_reached = True

			# If the local waypoint is too close to a wall, could be acheive when the wall isn't still discovered
				if not is_safe_waypoint(self.local_waypoint, self.safe_range, self.robot.live_grid_map):
					# print("Robot id :",self.robot.id, ", Local waypoint too close to a wall")
					self.local_waypoint_reached = True
					self.local_waypoints = []
					self.local_waypoint = None

				# If the robot need to cross a wall to go to the local waypoint, could be acheive when the wall isn't still discovered
				elif need_line_split(self.robot.live_grid_map, self.robot.pos, self.local_waypoint, self.safe_range, window):
					self.local_waypoint_reached = True
					self.local_waypoints = []
					self.local_waypoint = None
					

		### MANAGE WAYPOINT
		if not len(self.waypoints) and self.waypoint_reached:
			# Compute the new waypoint to go
			wp = self.compute_next_wp(window)

			# If the map is completely explored
			if wp == 1:
				self.mode = 100
				return

			# If the waypoint is accessible
			if wp:
				self.waypoints.append(wp)


		if len(self.waypoints) and self.waypoint_reached:
			self.waypoint = self.waypoints.pop(0)
			self.waypoint_reached = False

		if self.waypoint is None:
			lwps = self.compute_emergency_path_to_wp(window)
		
		### MANAGE LOCAL WAYPOINT
		if self.waypoint and not len(self.local_waypoints) and self.local_waypoint_reached:
			if self.need_global_path:
				wps = self.compute_emergency_path_to_wp(window)
				# lwps = self.compute_global_path_to_wp(window)

				if wps:
					self.local_waypoints = []
					self.waypoints = wps[1::]
					self.waypoint = wps[0]

				# Reset the demand
				self.need_global_path = False
			
			# Compute safe next local waypoint
			a = perf_counter()
			# lwps = self.compute_global_path_to_wp(window)
			lwps = self.compute_next_lwp(window)
			b = perf_counter()
			self.times.append(b-a)

			# If no path found or going to the point is impossible
			if lwps is None or lwps[0] == 1:
				self.waypoint_reached = True
				self.local_waypoint_reached = True
				self.no_safe_path_found_counter += 1
				return
			
			if lwps:
				self.no_safe_path_found_counter = 0
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
		if not ut.point_in_circle(self.robot.pos, self.last_static_pos, 40):
			self.last_static_pos = self.robot.pos
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
		self.time += dt

		# Before planning path, check if in a deadlock
		self.check_if_in_deadlock(dt)

		# Point must be safe to go
		self.manage_waypoint_system(window)

		# If no local waypoint to go
		if self.local_waypoint is None:
			print("No local waypoint !")
			self.robot.move(dt, 0, 0)
			self.robot.rotate(dt, 0, 0)
			return
		
		# Correct angle to point to the local waypoint
		angle_diff = ut.abs_angle(self.local_waypoint,self.robot.pos_calc) * 180 / pi - self.robot.rot_calc
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


		
	def compute_next_wp(self, window):
		"""Find a safe new waypoint from the open frontiers

		Returns:
			tuple||None: A new safe waypoint if it exists
		"""
		live_grid_map = self.robot.live_grid_map


		op_bds = live_grid_map.find_frontiers()

		if check_map_explored(op_bds):
			# If no line, then the map is entierly explored
			print("Goal achieved ! \nNothing more to explore :)")
			return 1
		

		pos_op_bds = [live_grid_map.ids_to_center(op[1], op[0]) for op in op_bds]

		costed_wps = [compute_wp_cost(self.robot, wp, window) for wp in pos_op_bds]

		# print(self.time)
		# if self.time > 56.0:
		# 	for i, (c, p) in enumerate(sorted(zip(costed_wps, pos_op_bds), key=lambda cost: cost[0])):
		# 		disp.draw_point(window, p, 10, (255, 0, 100))
		# 	for i, (c, p) in enumerate(sorted(zip(costed_wps, pos_op_bds), key=lambda cost: cost[0])):
		# 		if not i%11:
		# 			disp.write_text(f"{c:.1f}", window, (p[0]+10, p[1]), text_size=24, text_flush='right', update=False)
		# 	disp.draw_point(window, (-100,0), 10000, (255, 0, 100))

		ordered_wps = [wp for _, wp in sorted(zip(costed_wps, pos_op_bds), key=lambda cost: cost[0])]

		for ord_wp in ordered_wps[self.no_safe_path_found_counter::]:
			safe_waypoint = is_safe_waypoint(ord_wp, self.safe_range, live_grid_map)
			if safe_waypoint:
				return ord_wp
			
		return None
	

	def compute_next_lwp(self, window):
		"""Determines the next safe local waypoint for a robot to navigate towards a target waypoint, 
		considering obstacles and a specified safety distance.

		Args:
			window (optional): Placeholder for a visualization window, not used in the current implementation.

		Returns:
			tuple: Coordinates of the next safe local waypoint (x, y) if found.
			int: Returns 1 if no safe path is available within the given safe range.
		"""
		if self.waypoint is None:
			return

		map_size = (1100, 600)
		map = self.robot.live_grid_map
		robot_pos = self.robot.pos_calc
		safe_lwp = self.waypoint

		cpt = 0
		while True and cpt < 100:
			cpt += 1
			if need_line_split(map, robot_pos, safe_lwp, self.safe_range, window):
				line = ut.compute_line(robot_pos, safe_lwp)
				mdp = split_line(map, map_size, robot_pos, safe_lwp, line, self.safe_range, 0)
				if mdp:
					safe_lwp = mdp
				else:
					print(f"No path available with this safe range : {self.safe_range}")
					return [1]
						
			else:
				return [safe_lwp]
				
		return
	
	def compute_global_path_to_wp(self, window):
		self.global_path_mode = True
		
		if self.waypoint is None:
			return

		robot_pos = self.robot.pos_calc
		map = self.robot.live_grid_map
		map_size = (1100, 600)
		valid_sub_wp = [robot_pos, self.waypoint]


		cpt = 0
		while True and cpt < 100:
			cpt += 1

			new_valid_sub_wp = [robot_pos]
			line_splited = False

			for i in range(len(valid_sub_wp) - 1):
				p1 = valid_sub_wp[i]
				p2 = valid_sub_wp[i+1]
				line = ut.compute_line(p1, p2)
				if need_line_split(map, p1, p2, self.safe_range, window):
					mdp = split_line(map, map_size, p1, p2, line, self.safe_range)
					if mdp:
						if mdp != p2:
							line_splited = True
							new_valid_sub_wp.append(mdp)
					else:
						print(f"No path available with this safe range : {self.safe_range}")
						return
				new_valid_sub_wp.append(p2)
			
			if len(valid_sub_wp) > 100:
				return
			valid_sub_wp = shorten_path(map, new_valid_sub_wp[:], self.safe_range, window)
					
			if not line_splited:
				return valid_sub_wp[1::]
			
		
		print(f"Global analysis, no path found with this safe range : {self.safe_range}")
		return
	
	def compute_emergency_path_to_wp(self, window):
		self.emergency_mode = True
		live_grid_map = self.robot.live_grid_map
		safe_tiles = np.where(live_grid_map.map == 19)
		safe_tiles = list(zip(safe_tiles[0], safe_tiles[1]))

		tiles_pos = [live_grid_map.ids_to_center(tid[1], tid[0]) for tid in safe_tiles]
		dists_to_wp = [ut.distance(self.waypoint, tile_pos) for tile_pos in tiles_pos]

		self.waypoint = tiles_pos[np.argmin(dists_to_wp)]


		begin = live_grid_map.coord_to_ids(self.robot.pos_calc)
		end = live_grid_map.coord_to_ids(self.waypoint)

		start_id = [i for i, st in enumerate(safe_tiles) if st[1] == begin[0] and st[0] == begin[1]][0]
		end_id = [i for i, st in enumerate(safe_tiles) if st[1] == end[0] and st[0] == end[1]][0]

		c_table = make_connection_table(safe_tiles)
		# print(c_table)

		# visited = [False]*len(c_table)
		dijkstra_distances = dijkstra(start_id, c_table, window, tiles_pos)
		# print(dijkstra_distances)
		connected_path_ids = compute_dijkstra_path(end_id, start_id, dijkstra_distances, c_table)
		# print(connected_path_ids)
		# connected_path_ids = dfs(start_id, c_table, visited)
		connected_path = [tiles_pos[i] for i in connected_path_ids]

		# for p in connected_path:
		# 	disp.draw_point(window, p, 1, (255, 0, 0))

		if not len(connected_path):
			return [self.last_static_pos]

		connected_path = connected_path[::-3]

		### PROBLEM IN THIS FUNCTION, P1 MUST BE EQUAL TO P2 AT THE END OF A FOR LOOP
		emergency_path = shorten_path(live_grid_map, connected_path, self.safe_range, window)
			
		return emergency_path
		# return connected_path[:connected_path_ids.index(id_end)]


def shorten_path(map, path, safe_range, window):
	i = 0
	p1 = path[i]
	sh_path = [p1]
	while i < len(path) - 1:
		for n in range(i+1, len(path)):
			p2 = path[n]
			if need_line_split(map, p1, p2, safe_range, window):
				sh_path.append(path[n-1])
				break
		i = n
		# print(i)
		
		p1 = path[i-1]
	sh_path.append(path[-1])

	return sh_path

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


def need_line_split(live_grid_map, p1, p2, safe_range, window):
	"""Determines if the straight line between two points (p1 and p2) is obstructed by obstacles 
	or too close to them within a specified safety range.

	Args:
		live_grid_map (Live_grid_map): The robot's live grid map containing obstacle information.
		p1 (tuple): The starting point of the line as (x, y).
		p2 (tuple): The ending point of the line as (x, y).
		safe_range (float): The minimum distance from obstacles required to consider the path safe.

	Returns:
		bool: 
			- `True` if the line between `p1` and `p2` is obstructed or too close to obstacles.
			- `False` if the line is clear and safe.
	"""
	
	map_line_ids = live_grid_map.points_in_safe_range_to_ids(p1, p2, safe_range)
	for ids in map_line_ids:
		if live_grid_map.map[ids[1], ids[0]] > 99:
			return True	
	return False


def split_line(live_grid_map, map_size, p1, p2, line, safe_range, imposed_sign = 0,	window = None):
	"""Splits the line segment between two points (p1 and p2) and identifies a safe point normal 
	to the line, considering obstacles and a specified safe distance.

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
			mdp_1 = (mdp_1[0] + 0.25 * safe_range * ortho_vec[0], mdp_1[1] + 0.25 * safe_range * ortho_vec[1])
			
			if is_safe_waypoint(mdp_1, safe_range, live_grid_map):
				return mdp_1

			if not ut.point_in_box(mdp_1, (0, 0), map_size):
				mdp_1 = None
			

		if mdp_2:
			mdp_2 = (mdp_2[0] - 0.25 * safe_range * ortho_vec[0], mdp_2[1] - 0.25 * safe_range * ortho_vec[1])

			if is_safe_waypoint(mdp_2, safe_range, live_grid_map):
				return mdp_2

			if not ut.point_in_box(mdp_2, (0, 0), map_size):
				mdp_2 = None
		

		if mdp_1 is None and mdp_2 is None:
			return None
	
	return None




### NEED TO BE CHANGE




def dfs(start_id, ids, visited):
	"""Implementation of the Depth-First Search (DFS) algorithm to traverse and find all nodes 
	connected to a starting node in a graph.

	Args:
		start_id (int): The index of the starting node for the traversal.
		ids (list): Adjacency list representing the graph. 
					Each element `ids[i]` is a list of nodes connected to node `i`.
		visited (list): A boolean list where `visited[i]` indicates whether node `i` 
						has been visited.

	Returns:
		list: A list of all nodes connected to the starting node, in the order they are visited.

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


def dijkstra(start_id, c_table, window, tiles_pos):
	distances = [1e9] * len(c_table)
	distances[start_id] = 0

	visited = [False] * len(c_table)
	n = 0
	while 1e9 in distances and n < 2 * len(c_table):
		n += 1

		dist_min = 1e9
		for i in range(len(distances)):
			if not visited[i] and distances[i] < dist_min:
				dist_min = distances[i]
				id_dist_min = i

		# disp.draw_point(window, tiles_pos[id_dist_min], 1)
		for neighbor in c_table[id_dist_min]:
			distances[neighbor] = min(distances[neighbor], distances[id_dist_min] + 1)
		
		visited[id_dist_min] = visited
	return distances


def compute_dijkstra_path(end_id, start_id, distances, c_table):
	path = []
	current_id = end_id

	n = 0
	while current_id != start_id and n < 1000:
		n += 1
		path.append(current_id)
		id_dist_min = np.argmin([distances[neigh] for neigh in c_table[current_id]])
		current_id = c_table[current_id][id_dist_min]
	
	return path


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
			if abs(ids[i][0] - ids[j][0]) < 2 and abs(ids[i][1] - ids[j][1]) < 2:
				connection_table[i].append(j)
				connection_table[j].append(i)
		
	return connection_table
	


def compute_wp_cost(robot, wp_pos, window):
	"""Compute the cost of a waypoint to go to taking into acount position, speed, rotation and angular speed"""

	dist = ut.distance(robot.pos_calc, wp_pos)
	ang = ut.abs_angle(wp_pos, robot.pos_calc) * 180 / pi - robot.rot_calc
	ang %= 360
	if ang > 180:
		ang -= 360
	ang_speed = abs(ut.sign(robot.rot_speed_calc) - ut.sign(ang))
	speed = abs(ut.sign(robot.speed_calc) - 1 * (abs(ang) <= 90) + 1 * (abs(ang) > 90))
	
	# Weight associated with the distance to waypoint
	p1 = 1/5
	# Weight associated with the angle to waypoint
	p2 = 1
	# Weight associated with the angular speed
	p3 = 0
	# Weight associated with the speed
	p4 = 0

	dist_cost = dist * p1
	ang_cost = abs(ang) * p2
	ang_speed_cost = ang_speed * p3
	speed_cost = speed * p4

	cost = dist_cost + ang_cost + ang_speed_cost + speed_cost

	return cost


def check_map_explored(frontiers):
	# Connections tables of all frontiers pixels
	connection_table = make_connection_table(frontiers)
	# Connected lines of indices of frontiers pixels
	c_lines_ids = connect_lines_ids(connection_table, frontiers)

	for line in c_lines_ids:
		if len(line) > 5:
			return False
	
	return True


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