import numpy as np
# from robot import BeaconRobot
from util import *
import pygame		
import cv2

class Controller:
	def __init__(self, mode):
		self.mode = mode            # Mode 1 = Autonomous
		self.waypoints = None       # List of point to explore
		self.waypoint = None
		self.waypoint_reached = True
		self.safe_radius_multiplicator = 4
		self.range_safe_path_computer = 100	# Range of the robot to find the new path with the voronoi
		self.voronoi_diagram = None
		self.thinned_voronoi_polygon = None

	def move_to_waypoint(self):
		# Point must be safe to go
		if self.waypoints is None and self.waypoint is None:
			return

		if len(self.waypoints) > 0 and self.waypoint_reached:
			self.waypoint = self.waypoints.pop()
			self.waypoint_reached = False

		


	
	def follow_gost():
		...

	def find_new_direction(self, points, robot, window):

		in_void = False

		longuest_void = []
		safe_points = []
		n = len(points)

		i = 0
		# points connections are treated at the end
		angle_range = robot.lidar.resolution * (1 + 10 * robot.lidar.precision / 100)
		if get_angle_tuple_deg(points[-1], robot.pos.to_tuple(), points[0]) > angle_range:
			while get_angle_tuple_deg(points[i], robot.pos.to_tuple(), points[i+1]) > angle_range:
				i += 1

		test_angle_gap = get_angle_tuple_deg(points[i], robot.pos.to_tuple(), points[i+1]) > angle_range
		while test_angle_gap or i < n:
			if not test_angle_gap and not in_void:
				id_min = i
				in_void = True
			
			if test_angle_gap and in_void:
				id_max = i
				longuest_void.append(id_max - id_min)
				safe_points.append((points[id_min], points[id_max%n]))
				in_void = False
		
			test_angle_gap = get_angle_tuple_deg(points[i%n], robot.pos.to_tuple(), points[(i+1)%n]) > angle_range
			i += 1

		dtype = [("distance", float), ("point", tuple)]
		list_points = [(dist, safe_ang) for dist, safe_ang in zip(longuest_void, points)]

		struct_array = np.array(list_points, dtype)
		sorted_safe_points = np.sort(struct_array, order="distance")

		for pts in safe_points:
			for point in pts:
				pygame.draw.circle(window, (255, 0, 0), point, 3)
				pygame.display.update()
				pygame.time.delay(20)
		cell_size = robot.live_grid_map_size

	def find_new_direction_2(self, points, robot, window):
		##############################################################################
		
		
		
		### LES POINTS NE SONT PAS TRIE CORRECTEMENT DANS L'ORDRE DES ANGLES !!!!!!!!!
		### CA NE PEUT PAS FONCTIONNER COMME CA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



		##############################################################################
		longuest_void = []
		safe_points = []
		n = len(points)

		# i = 0
		# points connections are treated at the end
		# if distance_tuple(points[-1], points[0]) > 2*robot.radius:
			# while distance_tuple(points[i], points[i+1]) > 2*robot.radius:
				# i += 1
 
		for i in range(-1,n-1):
			if distance_tuple(points[i], points[i+1]) > self.safe_radius_multiplicator*robot.radius:
				longuest_void.append(distance_tuple(points[i], points[(i+1)]))
				safe_points.append((points[i], points[(i+1)%n]))

		dtype = [("distance", float), ("point", tuple)]
		list_points = [(dist, safe_ang) for dist, safe_ang in zip(longuest_void, points)]

		struct_array = np.array(list_points, dtype)
		sorted_safe_points = np.sort(struct_array, order="distance")

		# for i, pts in enumerate(safe_points):
		# 	pygame.draw.line(window, ((len(safe_points)-i)/len(safe_points)*255, 0, i/len(safe_points)*255), pts[0], pts[1], 3)
		# 	pygame.display.update()
		# 	pygame.time.delay(200)
		
	

	def check_safe_path(self, robot):
		points = None

		self.build_voronoi_diagram(robot, points)


	

	def thin_polygon(self, polygon, offset):
		"""
		https://stackoverflow.com/questions/68104969/offset-a-parallel-line-to-a-given-line-python/68109283#68109283

		Args:
			polygon (_type_): List of point defining the polygon
			offset (_type_): Signed distance to offset
		"""
		num_points = len(polygon)

		thinned_polygon = []

		for i in range(num_points):
			prev = (i + num_points - 1) % num_points
			next = (i + 1) % num_points

			vn = (polygon[next, 0] - polygon[i, 0], polygon[next, 1] - polygon[i, 1])
			vnnX, vnnY = normalizeVec(vn)
			nnnX = vnnY
			nnnY = -vnnX

			vp = (polygon[i, 0] - polygon[prev, 0], polygon[i, 1] - polygon[prev, 1])
			vpnX, vpnY = normalizeVec(vp)
			npnX = vpnY
			npnY = -vpnX

			bis = ((nnnX + npnX), (nnnY + npnY))

			bisnX, bisnY = normalizeVec(bis)
			bislen = offset /  np.sqrt((1 + nnnX*npnX + nnnY*npnY)/2)
			
			p = (polygon[i, 0] + bislen * bisnX, polygon[i, 1] + bislen * bisnY)

			to_be_added = True
			for j in range(len(polygon)):
				line = compute_line_tuple(polygon[j], polygon[j-1])
				if orthogonal_projection(p, line) < abs(offset) * 0.95:
					to_be_added = False
			if to_be_added:
				thinned_polygon.append(p)
		
		return thinned_polygon


	def build_voronoi_diagram(self, robot, points):
		"""
		## Description
			Construction du diagramme de Voronoi avec les obstacles et le bateau
		
		### Args:
			robot (BeaconRobot): Objet robot
			obstacles (list[tuple]): List of lidar points
		
		### Returns:
			list: List of centres and edges of the Voronoi diagram
		"""
		# Definition de la zone pour effectuer la subdivision
		rect = (int(robot.pos.x - self.range_safe_path_computer), int(robot.pos.y - self.range_safe_path_computer), 2*self.range_safe_path_computer + 1, 2*self.range_safe_path_computer + 1)
		subdiv = cv2.Subdiv2D(rect)

		# Insertion des points dans la subdivision pour en extraire le diagram de Voronoi
		subdiv.insert(robot.pos.to_tuple())
		for point in points:
			if point_in_circle(point, robot.pos.to_tuple(), self.range_safe_path_computer):
				subdiv.insert(point)

		# Calcul du diagramme de Voronoi
		self.voronoi_diagram  = subdiv.getVoronoiFacetList([])		
		self.thinned_voronoi_polygon = self.thin_polygon(self.voronoi_diagram[0][0], -robot.radius)

	def draw_voronoi_diagram(self, window):
		if self.voronoi_diagram is None:
			return None
		

		# for f in self.voronoi_diagram[0]:
		# 	polygon = [(float(f[i][0]), float(f[i][1])) for i in range(len(f))]
		# 	pygame.draw.polygon(window, (100, 255, 100), polygon, 2)
				# pygame.draw.line(window, (100, 255, 100), (f[i, 0], f[i, 1]), (f[(i+1)%len(f), 0], f[(i+1)%len(f), 1]), 2)

		robot_polygon_wrong_type = self.voronoi_diagram[0][0]
		robot_polygon_thinned_wrong_type = self.thinned_voronoi_polygon
		robot_polygon = [(float(robot_polygon_wrong_type[i][0]), float(robot_polygon_wrong_type[i][1])) for i in range(len(robot_polygon_wrong_type))]
		robot_polygon_thinned = [(float(robot_polygon_thinned_wrong_type[i][0]), float(robot_polygon_thinned_wrong_type[i][1])) for i in range(len(robot_polygon_thinned_wrong_type))]

		for p in robot_polygon:
			pygame.draw.circle(window, (255, 0, 0), p, 2)

		pygame.draw.polygon(window, (100, 255, 100), robot_polygon, 2)
		pygame.draw.polygon(window, (255, 255, 100), robot_polygon_thinned, 2)


	