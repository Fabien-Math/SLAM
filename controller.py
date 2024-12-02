from math import pi, sin, cos
from random import randint
from util import *
import pygame		
import cv2

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from robot import BeaconRobot  # Import uniquement pour l'annotation de type


class Controller:
	def __init__(self, robot, freq, mode):
		self.robot:BeaconRobot = robot

		self.mode = mode            # Mode 1 = Autonomous

		self.waypoints = [(randint(100, 1100), randint(100, 500)) for _ in range(20)]       # List of point to explore
		self.waypoint_radius = 10
		self.waypoint = None
		self.waypoint_reached = True

		self.local_waypoints = []
		self.local_waypoint_radius = 5
		self.local_waypoint = None
		self.local_waypoint_reached = True

		self.range_safe_path_computer = 100	# Range of the robot to find the new path with the voronoi

		self.freq_check_for_new_path = freq
		self.last_time_check_for_new_path = 0

		self.voronoi_diagram = None
		self.thinned_voronoi_polygon = None


	def move_to_waypoint(self, dt):
		"""Move to the next waypoint

		Args:
			dt (float): Time enlapsed
		"""
		# Point must be safe to go
		if self.waypoints is None:
			return

		if len(self.waypoints) > 0 and self.waypoint_reached:
			self.waypoint = self.waypoints.pop()
			self.waypoint_reached = False

		if len(self.local_waypoints):
			# self.check_safe_path()
			self.local_waypoint = self.local_waypoints.pop()
			self.local_waypoint_reached = False

		if self.local_waypoint is None:
			return

		# Correct angle to point to the local waypoint
		angle_diff = get_absolute_angle(self.local_waypoint,self.robot.pos_calc.to_tuple()) * 180 / pi - self.robot.rot_calc
		angle_diff %= 360
		if angle_diff > 180:
			angle_diff -= 360

		# Compute speed according to the distance from the local waypoint the robot is
		move_speed = min(distance(self.robot.pos_calc, self.local_waypoint)/200 + 0.1 - abs(angle_diff)/180, 1)
		rot_speed = max(min(angle_diff, 1), -1)
		rot_dir = sign(rot_speed)

		self.robot.move(dt, 1, move_speed)
		self.robot.rotate(dt, rot_dir, abs(rot_speed))

		# Arrived at local waypoint
		if point_in_circle(self.robot.pos_calc.to_tuple(), self.local_waypoint, self.local_waypoint_radius):
			self.local_waypoint_reached = True

		# Arrived at waypoint
		if point_in_circle(self.robot.pos_calc.to_tuple(), self.waypoint, self.waypoint_radius):
			self.waypoint_reached = True


	def check_safe_path(self):
		"""Construct a Voronoi diagram to check if the path is safe

		Args:
			t (float): Current time
		"""
		# # If not enough time pass
		# if t < self.last_time_check_for_new_path + 1 / self.freq_check_for_new_path:
		# 	return

		idx, idy = self.robot.live_grid_map.coord_to_ids(self.robot.pos_calc.to_tuple())
		range_ids = int(self.range_safe_path_computer/self.robot.live_grid_map.size)

		live_grid_map_list_size = self.robot.live_grid_map.list_size
		points = []
		for i in range(idy - range_ids, idy + range_ids + 1):
			for j in range(idx - range_ids, idx + range_ids + 1):
				if 0 < j < live_grid_map_list_size and 0 < i < live_grid_map_list_size:
					value = self.robot.live_grid_map.map[i, j]
					if value > 0:
						rect = self.robot.live_grid_map.ids_to_rect(j, i)
						points.append(middle_point(rect[0:2], rect[2:4]))
		
		self.build_voronoi_diagram(points)


		next_point = self.thinned_voronoi_polygon[-1]
		# for i in range(len(self.thinned_voronoi_polygon)):
		# 	p1 = self.thinned_voronoi_polygon[i-1]
		# 	p2 = self.thinned_voronoi_polygon[i]
		# 	p_inter = segment_intersection(p1, p2, self.robot.pos_calc.to_tuple(), self.waypoint)
		# 	if p_inter:
		# 		next_point = p_inter
		# 		break
		

		# self.local_waypoints.append(next_point)


		# self.last_time_check_for_new_path = t

	def find_new_direction(self, lidar_data, lidar_pos, max_lidar_distance, live_grid_map, window):

		if not self.local_waypoint_reached:
			return

		i = 0
		n = len(lidar_data)
		points = []
		
		dist = lidar_data[i][0]
		crit_dist = 1.5 * max_lidar_distance

		roll = False
		if dist > crit_dist:
			roll = True

		while i < n+1 or dist > crit_dist:
			i += 1
			dist = lidar_data[i % n][0]
			dist_next = lidar_data[(i + 1) % n][0]
			if dist > crit_dist and dist_next < crit_dist:
				ang = lidar_data[(i + 1) % n][1]
				points.append((lidar_pos.x + dist_next*cos(ang), lidar_pos.y + dist_next*sin(ang)))
			elif dist < crit_dist and dist_next > crit_dist:
				ang = lidar_data[i % n][1]
				points.append((lidar_pos.x + dist*cos(ang), lidar_pos.y + dist*sin(ang)))
			
		if roll:
			p = points.pop(0)
			points.append(p)
		
		n_mid = len(points)//2
		mid_points = [0]*n_mid
		for i in range(n_mid):
			mid_points[i] = middle_point(points[2*i], points[2*i+1])
		
		open_boundaries = self.robot.live_grid_map.find_frontiers()
		if len(open_boundaries):
			connection_table = make_connection_table(open_boundaries)
			c_lines_ids = connect_lines_ids(connection_table, open_boundaries)
			c_lines = connect_lines(c_lines_ids, open_boundaries)
			ordered_lines = order_lines(c_lines, c_lines)
			if len(ordered_lines) == 0:
				print("Goal achieved !")
				print("Nothing more to explore :)")
				self.mode = 100
				return

			n_r = np.random.randint(0, len(ordered_lines))
			idy, idx = ordered_lines[n_r][len(ordered_lines[n_r])//2]
			pos = live_grid_map.ids_to_rect(idx, idy)[0:2]
			self.local_waypoints.append(pos)



		
		# if len(self.local_waypoints) == 0:
		# 	if len(open_boundaries):
		


	def build_voronoi_diagram(self, points):
		"""
		Build Voronoi diagram with walls
		
		Args:
			obstacles (list[tuple]): List of lidar points
		
		Returns:
			list: List of centres and edges of the Voronoi diagram
		"""
		# Definition de la zone pour effectuer la subdivision
		rect = (int(self.robot.pos.x - self.range_safe_path_computer), int(self.robot.pos.y - self.range_safe_path_computer), 2*self.range_safe_path_computer + 1, 2*self.range_safe_path_computer + 1)
		subdiv = cv2.Subdiv2D(rect)

		# Insertion des points dans la subdivision pour en extraire le diagram de Voronoi
		subdiv.insert(self.robot.pos.to_tuple())
		for point in points:
			if point_in_circle(point, self.robot.pos.to_tuple(), self.range_safe_path_computer):
				subdiv.insert(point)

		# Calcul du diagramme de Voronoi
		self.voronoi_diagram  = subdiv.getVoronoiFacetList([])	
		self.thinned_voronoi_polygon = self.thin_polygon(self.voronoi_diagram[0][0], -self.robot.radius)


	def thin_polygon(self, polygon, offset):
		"""
		Offset the polygon and make sure no point are in the safe range
		Other way which don't care of overlap : https://stackoverflow.com/questions/68104969/offset-a-parallel-line-to-a-given-line-python/68109283#68109283

		Args:
			polygon (_type_): List of point defining the polygon
			offset (_type_): Signed distance to offset
		"""
		num_points = len(polygon)

		thinned_polygon = []
		lines = [None] * num_points
		parallel_lines = [None] * num_points

		# Build lines and parallel lines
		for i in range(num_points):
			lines[i] = compute_line_tuple(polygon[i-1], polygon[i])
			parallel_lines[i] = compute_parallel_line(lines[i], offset)

		# Find all parallel line intersections
		intersections = []
		for i in range(num_points):
			for j in range(num_points):
				if j == i:
					continue
				p = find_intersection(parallel_lines[j], parallel_lines[i])
				if p:
					if point_in_polygon(p, polygon):
						intersections.append(p)

		# Check points to be at the right distance from the polygon walls
		for p in intersections:
			to_be_added = True
			for line in lines:
				if orthogonal_projection(p, line) < abs(offset)*0.98:
					to_be_added = False
					break
				
			if to_be_added:
				thinned_polygon.append(p)
		
		return convex_hull(thinned_polygon)


	def draw_voronoi_diagram(self, window):
		"""Draw the Voronoi diagram on the screen

		Args:
			window (surface): Surface on which to draw
		"""
		if self.voronoi_diagram is None or len(self.thinned_voronoi_polygon) < 2:
			return None
		
		# Draw complete Voronoi diagram
		# for f in self.voronoi_diagram[0]:
		# 	polygon = [(float(f[i][0]), float(f[i][1])) for i in range(len(f))]
		# 	pygame.draw.polygon(window, (100, 255, 100), polygon, 2)
		
		# Draw robot Voronoi diagram
		robot_polygon_wrong_type = self.voronoi_diagram[0][0]
		robot_polygon = [(float(robot_polygon_wrong_type[i][0]), float(robot_polygon_wrong_type[i][1])) for i in range(len(robot_polygon_wrong_type))]
		pygame.draw.polygon(window, (100, 255, 100), robot_polygon, 1)

		# Draw thinned robot Voronoi diagram
		robot_polygon_thinned_wrong_type = self.thinned_voronoi_polygon
		robot_polygon_thinned = [(float(robot_polygon_thinned_wrong_type[i][0]), float(robot_polygon_thinned_wrong_type[i][1])) for i in range(len(robot_polygon_thinned_wrong_type))]
		pygame.draw.polygon(window, (255, 255, 100), robot_polygon_thinned, 1)



def dfs(start_id, idss, visited):
	stack = [start_id]
	nodes = []
	while len(stack):
		node = stack.pop()
		if not visited[node]:
			visited[node] = True
			nodes.append(node)
			
			for n_node in idss[node]:
				if not visited[n_node]:
					stack.append(n_node)
	return nodes


def make_connection_table(ids:list):
	connection_table = [[] for _ in range(len(ids))]
	for i in range(len(ids)):
		for j in range(i+1, len(ids)):
			if abs(ids[i][0] - ids[j][0]) < 2 and ids[i][1] == ids[j][1]:
				connection_table[i].append(j)
				connection_table[j].append(i)
			if abs(ids[i][1] - ids[j][1]) < 2 and ids[i][0] == ids[j][0]:
				connection_table[i].append(j)
				connection_table[j].append(i)
		
	return connection_table
	


def connect_lines_ids(connection_table:list, ids:list):
	"""Take a list of 2D indices and connect them into separate list

	Args:
		ids (list): List of 2D ids to sort
	"""	
	
	visited = [False] * len(ids)

	c_ids = []
	for i in range(len(ids)):
		if not visited[i]:
			c_ids.append(dfs(i, connection_table, visited))
	
	return c_ids

def connect_lines(c_ids:list, ids:list):
	"""Take a list of 2D indices and connect them into separate list

	Args:
		ids (list): List of 2D ids to sort
	"""	
	c_lines = [[[] for _ in range(len(c))] for c in c_ids]
	for i in range(len(c_ids)):
		for j in range(len(c_ids[i])):
			c_lines[i][j] = ids[c_ids[i][j]]

	return c_lines



def order_lines(connected_lines:list, ids_lines:list):
	"""Take a list of connected lines and get the max / min indice along x and y

	Args:
		connected_lines (list): List of connected lines of indices
	"""

	ordonned_lines = []
	for line, ids_line in zip(connected_lines, ids_lines):
		c_table = make_connection_table(line)
		if len(c_table) < 5:
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
	
		line_ids = dfs(start_ids[0], c_table, visited)
		ordonned_lines.append([ids_line[p_id] for p_id in line_ids])
	return ordonned_lines


def compute_ids_center(connected_line, extremum_connected_line):
	"""Compute the mid point of a line defined by ids

	Args:
		connected_line (list): Connected line
		extremum_connected_lines (_type_): Extremum of the connected line
	"""
