import time
import numpy as np

from robot import BeaconRobot
from map import Map
import util as ut

class World:
	def __init__(self, window):
		self.window = window
		# TIME INITIALIZATION
		self.real_time: float = time.time()
		self.time: float = 0
		self.dt: float = 1/60

		# ROBOT INITIALIZATION
		self.robots: list[BeaconRobot] = []
		self.active_robot = 0

		self.crashed_robot: list = []

		for i in range(1):
			beacon = BeaconRobot(i, (500+50*i, 300+20*i), 50, 1000, 50, 50, 150)
			# beacon = BeaconRobot(i, (200+50*i, 350), 50, 1000, 50, 50, 150)
			# beacon = BeaconRobot((300, 400), 50, 1000, 100, 25, 150)
			# Equip sensors
			beacon.equip_lidar(fov=360, freq=5, res=7, prec=(0.05, 0.02), max_dist=100)
			beacon.equip_accmeter(precision=(0.0005, 0.00002), time=self.time)
			beacon.equip_controller(mode=1)
			beacon.equip_communicator(self)

			self.robots.append(beacon)
			self.crashed_robot.append(False)
		self.linked_robot = []
		
		# MAP INITIALIZATION
		self.map_element_size:float = 40
		# self.map_element_size = 40		# Demo case
		# self.map_element_size = 70		# (Thin Wall Problem)
		# self.map_element_size = 100		# (Empty explored loop SOLVED)
		self.map_size:tuple = (1100, 600)
		self.map_offset = (50, 50)
		# Number of subdivisions in the map, used to list the lines
		self.map_subdiv_number = (30, 30)
		self.map: Map = Map(self.map_size, self.map_offset, self.map_subdiv_number, 
					  		self.map_element_size, int(np.sin(np.pi/3) * self.map_element_size))
		
		self.map_explored = False
		self.end_simulation = False
		
		
		# COMMUNICATION INITIALIZATION
		self.message_buffer = {str(r.id): '' for r in self.robots}

		


	def update(self):		
		self.time += self.dt
		self.linked_comm_robots()
		self.dispatch_message()

		if sum(self.crashed_robot) == len(self.robots):
			self.end_simulation = True

		for i, robot in enumerate(self.robots):			
			if robot.crashed:
				# continue
				pass

			### SIMULATION
			robot.compute_pos_calc(self.time)
			robot.scan_environment(self.time, self.map, robot.id, self.robots, self.window)
			robot.compute_robot_collision(self.map, self.robots, self.window)
			robot.live_grid_map.update_robot_path()
			robot.communicator.update_communicator()

			robot.controller.move_to_waypoint(self.dt, self.window)

			if robot.controller.mode == 100:
				robot.live_grid_map.save_map_to_image()
				print("Mean time for global path planning", np.mean(robot.controller.times))
				self.map_explored = True
			
			if robot.crashed:
				print(f"\n\n\n###   ROBOT {robot.id} CRASHED !!!   ###\n")
				self.crashed_robot[i] = True

	def dispatch_message(self):
		for robot in self.robots:
			if len(self.message_buffer[str(robot.id)]):
				for link in self.linked_robot:
					if link[0] == robot.id:
						# print(robot.id, "send message to", link[1])
						self.robots[link[1]].communicator.receiver_buffer[str(robot.id)] += self.message_buffer[str(robot.id)]
					if link[1] == robot.id:
						# print(robot.id, "send message to", link[0])
						self.robots[link[0]].communicator.receiver_buffer[str(robot.id)] += self.message_buffer[str(robot.id)]
				self.message_buffer[str(robot.id)] = ''

	def linked_comm_robots(self):
		for i, r1 in enumerate(self.robots):
			if r1.crashed:
				for r2 in self.robots[i+1:]:
					if (r1.id, r2.id) in self.linked_robot:
						self.linked_robot.remove((r1.id, r2.id))
			else:
				for r2 in self.robots[i+1:]:
					if r2.crashed:
						if (r1.id, r2.id) in self.linked_robot:
							self.linked_robot.remove((r1.id, r2.id))
					else:
						if self.is_wall_between(r1.pos, r2.pos):
							if (r1.id, r2.id) in self.linked_robot:
								self.linked_robot.remove((r1.id, r2.id))
						else:
							if (r1.id, r2.id) not in self.linked_robot:
								self.linked_robot.append((r1.id, r2.id))

	def is_wall_between(self, r1_pos:tuple, r2_pos:tuple):
		ids_r1 = self.map.subdiv_coord_to_ids(r1_pos)
		ids_r2 = self.map.subdiv_coord_to_ids(r2_pos)
		idsf_r1 = self.map.subdiv_coord_to_ids_float(r1_pos)
		idsf_r2 = self.map.subdiv_coord_to_ids_float(r2_pos)
		ids_line = ut.move_on_line(ids_r1, ids_r2, idsf_r1, idsf_r2)

		for ids in ids_line:
			if self.check_wall_collision(ids, r1_pos, r2_pos):
				return True
		return False
			
	def check_wall_collision(self, ids:tuple, r1_pos:tuple, r2_pos:tuple):
		"""Check if the line between r1 and r2 collide with a wall

		Args:
			map (Map): Simulation map
			ids (tuple): idx, idy of the map subdivision

		Returns:
			tuple: Intersection point found or None if no point found
		"""

		# Check if the ids are in the subdivided map
		if 0 <= ids[0] < self.map.subdiv_number[0]:
			if 0 <= ids[1] < self.map.subdiv_number[1]:
				# Get the list of indices of walls in the subdivision box 
				subdiv = self.map.subdivision[ids[1]][ids[0]]

				# If any wall is in the cell
				if subdiv != []:

					# Go through all walls in the map subdivision
					for k in subdiv:
						wall = self.map.walls[k]
						intersection = ut.compute_segment_inter(wall.p1, wall.p2, r1_pos, r2_pos)
												
						if intersection:
							rect = self.map.subdiv_ids_to_rect(ids[0], ids[1])
							if ut.point_in_box(intersection, (rect[0], rect[1]), (rect[2], rect[3])):
								return intersection
		return None