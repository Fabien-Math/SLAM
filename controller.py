from math import pi
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
		if self.waypoints is None and self.waypoint is None:
			return

		if len(self.waypoints) > 0 and self.waypoint_reached:
			self.waypoint = self.waypoints.pop()
			self.waypoint_reached = False

		# Correct angle to point to the waypoint
		angle_diff = get_absolute_angle(self.waypoint,self.robot.pos_calc.to_tuple()) * 180 / pi - self.robot.rot_calc
		angle_diff %= 360
		if angle_diff > 180:
			angle_diff -= 360

		# Compute speed according to the distance from the waypoint the robot is
		move_speed = min(distance(self.robot.pos_calc, self.waypoint)/200 + 0.3, 1)
		rot_speed = max(min(angle_diff, 1), -1)
		rot_dir = sign(rot_speed)

		self.robot.move(dt, 1, move_speed)
		self.robot.rotate(dt, rot_dir, abs(rot_speed))

		# Arrived at waypoint
		if point_in_circle(self.robot.pos_calc.to_tuple(), self.waypoint, self.waypoint_radius):
			self.waypoint_reached = True


	def check_safe_path(self, t):
		"""Construct a Voronoi diagram to check if the path is safe

		Args:
			t (float): Current time
		"""
		# If not enough time pass
		if t < self.last_time_check_for_new_path + 1 / self.freq_check_for_new_path:
			return

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
		self.last_time_check_for_new_path = t


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
		for f in self.voronoi_diagram[0]:
			polygon = [(float(f[i][0]), float(f[i][1])) for i in range(len(f))]
			pygame.draw.polygon(window, (100, 255, 100), polygon, 2)
		
		# Draw robot Voronoi diagram
		robot_polygon_wrong_type = self.voronoi_diagram[0][0]
		robot_polygon = [(float(robot_polygon_wrong_type[i][0]), float(robot_polygon_wrong_type[i][1])) for i in range(len(robot_polygon_wrong_type))]
		pygame.draw.polygon(window, (100, 255, 100), robot_polygon, 1)

		# Draw thinned robot Voronoi diagram
		robot_polygon_thinned_wrong_type = self.thinned_voronoi_polygon
		robot_polygon_thinned = [(float(robot_polygon_thinned_wrong_type[i][0]), float(robot_polygon_thinned_wrong_type[i][1])) for i in range(len(robot_polygon_thinned_wrong_type))]
		pygame.draw.polygon(window, (255, 255, 100), robot_polygon_thinned, 1)
