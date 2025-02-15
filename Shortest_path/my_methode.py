import util as ut
 
import time
import numpy as np

def need_line_split(map, p1, p2, line, safe_range):
	for c in map:
		match c[0]:
			case 'line':
				if not ut.compute_segment_inter(p1, p2, c[1], c[2]) is False:
					return True
			case 'circle':
				line = ut.compute_line(p1, p2)
				ortho_point = ut.orthogonal_point(c[1], p1, p2, line)
				if ut.distance(c[1], ortho_point) < c[2] + safe_range:
					if ut.point_in_box(ortho_point, p1, p2):
						return True
			case 'rect':
				lines = [(c[4], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[4])]
				for p3, p4 in lines:
					if not ut.compute_segment_inter(p1, p2, p3, p4) is False:
						return True
					
	return False
			
def is_safe_point(map, p, safe_range):
	if not ut.point_in_box(p, (0, 0), (1200, 700)):
		return False
	for c in map:
		match c[0]:
			case 'line':
				line = ut.compute_line(c[1], c[2])
				dist = ut.orthogonal_projection(p, line)
				if dist > safe_range:
					continue
				ortho_point = ut.orthogonal_point(p, c[1], c[2], line)
				if ut.point_in_box(ortho_point, c[1], c[2]):
					return False
				if ut.point_in_circle(ortho_point, c[1], safe_range):
					return False
				if ut.point_in_circle(ortho_point, c[2], safe_range):
					return False

			case 'circle':
				if ut.distance(c[1], p) < c[2] + safe_range:
					return False
			
			case 'rect':
				if ut.point_in_box(p, c[1], c[3]):
					return False
				
				lines = [(c[4], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[4])]
				for p1, p2 in lines:
					line = ut.compute_line(p1, p2)
					dist = ut.orthogonal_projection(p, line)
					if dist > safe_range:
						continue
					ortho_point = ut.orthogonal_point(p, p1, p2, line)
					if ut.point_in_box(ortho_point, p1, p2):
						return False
					if ut.point_in_circle(ortho_point, p1, safe_range):
						return False
					if ut.point_in_circle(ortho_point, p2, safe_range):
						return False

	return True

			
	

def split_line(map, map_size, p1, p2, line, safe_range, imposed_sign = 0):
	a, b, _ = line	# ax + by = c
	if a:
		ortho_vec = ut.normalize_vec((a, b))
	else:
		ortho_vec = (0, ut.sign(b)) 
	
	mdp_1 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	mdp_2 = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
	match imposed_sign:
		case 1:
			mdp_2 = None
		case -1:
			mdp_1 = None

	cpt = 0
	while True and cpt < 100:
		cpt += 1
		if mdp_1:
			mdp_1 = (mdp_1[0] + max(5, 0.25 * safe_range) * ortho_vec[0], mdp_1[1] + max(5, 0.25 * safe_range) * ortho_vec[1])
			
			if is_safe_point(map, mdp_1, safe_range):
				return mdp_1

			if not ut.point_in_box(mdp_1, (0, 0), map_size):
				mdp_1 = None
			

		if mdp_2:
			mdp_2 = (mdp_2[0] - max(5, 0.25 * safe_range) * ortho_vec[0], mdp_2[1] - max(5, 0.25 * safe_range) * ortho_vec[1])

			if is_safe_point(map, mdp_2, safe_range):
				return mdp_2

			if not ut.point_in_box(mdp_2, (0, 0), map_size):
				mdp_2 = None
		

		if mdp_1 is None and mdp_2 is None:
			return None
	
	print("No split line possible !")
	return None

def find_next_local_waypoint(map, map_size, robot_pos, waypoint_pos, safe_range):
	cpt = 0
	safe_local_waypoint = waypoint_pos
	mdp = waypoint_pos
	while True and cpt < 100:
		cpt += 1
		line = ut.compute_line(robot_pos, safe_local_waypoint)
		if need_line_split(map, robot_pos, safe_local_waypoint, line, safe_range):
			mdp = split_line(map, map_size, robot_pos, safe_local_waypoint, line, safe_range)
			if mdp:
				safe_local_waypoint = mdp
			else:
				print(f"No path available with this safe range : {safe_range}")
				safe_range -= 1
				if safe_range < 10:
					print(f"No path available !!")
					return
					
		else:
			return mdp
	print(f"No path available with this safe range : {safe_range}")
	return

def find_global_path(map, map_size, robot_pos, waypoint_pos, safe_range):
	valid_sub_wp = [robot_pos, waypoint_pos]

	cpt = 0
	while True and cpt < 50:
		cpt += 1

		new_valid_sub_wp = [robot_pos]
		line_splited = False

		for i in range(len(valid_sub_wp) - 1):
			p1 = valid_sub_wp[i]
			p2 = valid_sub_wp[i+1]

			line = ut.compute_line(p1, p2)
			if need_line_split(map, p1, p2, line, safe_range):
				line_splited = True
				mdp = split_line(map, map_size, p1, p2, line, safe_range)
				if mdp:
					new_valid_sub_wp.append(mdp)
				else:
					print(f"No path available with this safe range : {safe_range}")
					return
			new_valid_sub_wp.append(p2)
				
		valid_sub_wp = shorten_path(map, new_valid_sub_wp[:], safe_range)
		# valid_sub_wp = new_valid_sub_wp[:]
		print(valid_sub_wp)

		if not line_splited:
			return valid_sub_wp
	
	return

def shorten_path(map, path, safe_range):
	i = 0
	p1 = path[i]
	sh_path = [p1]
	while i < len(path) - 1:
		for n in range(i+1, len(path)):
			p2 = path[n]
			if need_line_split(map, p1, p2, ut.compute_line(p1, p2), safe_range):
				sh_path.append(path[n-1])
				break
		i = n
		
		p1 = path[i-1]
	sh_path.append(path[-1])

	return sh_path

def compute_path(source_pos, wp_pos, map_size, obstacles, safe_range):
	start_time_cpt = time.perf_counter()

	safe_local_waypoints = find_global_path(obstacles, map_size, source_pos, wp_pos, safe_range)

	end_time_cpt = time.perf_counter()

	print(f"Global path enlapse time : {end_time_cpt-start_time_cpt:.3e}s")
	print(f"Global path FPS : {1/(end_time_cpt-start_time_cpt):.1f} FPS")

	if safe_local_waypoints is None:
		return
	
	total_path_length = np.sum([ut.distance(safe_local_waypoints[i], safe_local_waypoints[i+1]) for i in range(len(safe_local_waypoints[1::]))])
	return total_path_length
