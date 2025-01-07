import time
import numpy as np
import matplotlib.pyplot as plt

from robot import BeaconRobot
from map import Map

def robot_outside_map(robot_pos, map:Map):
	safe_offset = 5
	x_min = map.map_offset[0] + safe_offset
	x_max = map.map_offset[0] + map.map_size[0] - safe_offset
	y_min = map.map_offset[1] + safe_offset
	y_max = map.map_offset[1] + map.map_size[1] - safe_offset

	if not(x_min < robot_pos.x < x_max) or not(y_min < robot_pos.y < y_max):
		return True

	return False

def launch_simulation(map:Map):
	### TIME INIT
	begin_real_time = time.time()
	t = 0
	i = 0
	dt = 2e-3

	x_robot_positions = []
	y_robot_positions = []



	### ROBOT INITIALISATION
	beacon = BeaconRobot((500, 300), 50, 1000, 100, 25, 150)
	# beacon = BeaconRobot((300, 400), 50, 1000, 100, 25, 150)
	# Equip sensors
	beacon.equip_lidar(fov=360, freq=2, res=3.5, prec=(0.05, 0.02), max_dist=100)
	beacon.equip_accmeter(precision=(0.0005, 0.00002), time=t)
	beacon.equip_controller(check_safe_path_frequency=5, mode=1)

	# State of the simulation
	running = True
	# Toggle to draw the map

	beacon_crashed = False
	
	while running:
		i += 1 

		### SIMULATION
		beacon.compute_pos_calc(t)
		beacon.scan_environment(t, map)
		beacon.compute_robot_collision(map)
		beacon.live_grid_map.update_robot_path()

		beacon.controller.move_to_waypoint(dt)
		x_robot_positions.append(beacon.pos_calc.x)
		y_robot_positions.append(beacon.pos_calc.y)

		# TIME
		t += dt
					
		if beacon.controller.mode == 100:
			beacon.live_grid_map.save_map_to_image()
			running = False
		
		# ROBIOT COLLISION
		# if beacon.crashed_in_wall:
		# 	running = False
		if robot_outside_map(beacon.pos_calc, map):
			running = False
		if t > 100:
			running = False

		if not i%int(1/dt):
			print(f"Time : {t:.3f} s", end='\r')

	end_real_time = time.time()
	print(f"Time to explore all the map : {t:.3f} s")
	print(f"Time to explore all the map : {t//3600:g} h, {(t - t//3600)//60:g} m, {t - (t//3600)*3600 - ((t - t//3600)//60)*60:.3f} s")

	print(f"Conputational  time : {end_real_time-begin_real_time:.3f} s")

	return x_robot_positions, y_robot_positions, t


def draw_path(map:Map, x, y, t):
	for wall in map.walls:
		plt.plot((wall.p1.x, wall.p2.x), (wall.p1.y, wall.p2.y), color="black")
	plt.plot(x, y, linewidth=5)
	plt.title(f"Time to explore the map : {t:.3f} s")
	plt.show()


def main():

	seed = 0
	np.random.seed(seed)
	# MAP INITIALISATION
	map_size = (1100, 600)
	map_offset = (50, 50)
	# Number of subdivisions in the map, used to list the lines
	subdiv_number = (25, 25)
	# Initilize the map
	map = Map(map_size, map_offset, subdiv_number, 40, int(np.sin(np.pi/3) * 40))
	draw_path(map, [0], [0], 0)

	answer = input("Continue ?")

	if answer == '':
		seed = 3
		np.random.seed(seed)
		xrbtp, yrbtp, t = launch_simulation(map)

		draw_path(map, xrbtp, yrbtp, t)



	...


if __name__ == '__main__':
	main()