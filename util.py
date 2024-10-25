from math import sqrt, acos
import numpy as np




class Vector2:
	"""
	### Vector with two elements (x, y)
	"""
	def __init__(self, x:float, y:float) -> None:
		self.x = x
		self.y = y

	def add(self, x:float, y:float):
		"""Add (x, y) to the point

		Args:
			x (float): x coordinate to be added
			y (float): y coordinate to be added
		"""
		self.x += x
		self.y += y
	
	def __str__(self):
		"""Redefine the print operator

		Returns:
			str: text to describe the point
		"""
		return f"({self.x}, {self.y})"
	
	def copy(self):
		"""
		Copy the point given to a new object
		"""
		return Vector2(self.x, self.y)

	def to_tuple(self):
		"""
		Convert the point to a tuple (x, y)
		"""
		return (self.x, self.y)

	def middle_point(self, p):
		"""Compute the middle point between p and itself

		Args:
			p (Vector2): Other point

		Returns:
			Vector2: Middle point
		"""
		return Vector2((self.x + p.x)/2, (self.y + p.y)/2)
	
	def distance(self, p):
		"""Compute the distance between a point and itself

		Args:
			p (Vector2): Other point

		Returns:
			float: Distance between points
		"""
		return sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
	

def get_angle(p1:Vector2, p2:Vector2, p3:Vector2):
	"""Compute the angle with p2 as center
	"""
	a1, b1, _ = compute_line(p1, p2)
	a2, b2, _ = compute_line(p2, p3)
	x = np.array([a1, b1])
	y = np.array([a2, b2])
	xy_norm = norm(x)*norm(y)
	if xy_norm:
		return acos(np.dot(x,y)/(xy_norm))

	return 0
	

def norm(u:tuple):
	"""Get the norm of a tuple
	"""
	return sqrt(u[0]**2 + u[1]**2)


def evaluate_line(l, x):
	"""Evaluate the line in 'x'

	Args:
		l (tuple): (a, b, c) for ax + by = c
		x (float): x to be evaluated

	Returns:
		float: line value in x
	"""
	if l[1] != 0:
		return (l[2] - l[0] * x) / l[1]
	else:
		return x


def distance(p1:Vector2, p2:tuple):
	"""Compute the distance between a Vector2 point and a tuple
	"""
	return sqrt((p2[0] - p1.x)**2 + (p2[1] - p1.y)**2)


def distance_points(p1:tuple, p2:tuple):
	"""Get the distance between two points as tuple
	"""
	return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def compute_line(p1:Vector2, p2:Vector2):
	"""Compute line equation : ax + by = c
	"""
	a = p2.y - p1.y
	b = p1.x - p2.x
	c = p1.x*(p2.y - p1.y) - p1.y*(p2.x - p1.x)
	return a,b,c


def point_in_box(p:tuple, p1:Vector2, p2:Vector2, error_offset:float = 1e-6):
	"""Check if the point 'p' is in the box defined by p1 and p2

	Args:
		p (tuple): Point to be checked
		p1 (Vector2): First point
		p2 (Vector2): Second point
		error_offset (float, optional): Error offset to overcome python binary error on float. Defaults to 1e-6.

	Returns:
		bool: If the point is in the box
	"""
	if p[0] > min(p1.x, p2.x) - error_offset and p[0] < max(p2.x, p1.x) + error_offset:
		if p[1] > min(p1.y, p2.y) - error_offset and p[1] < max(p2.y, p1.y) + error_offset:
			return True
	return False

def find_intersection(l1, l2):
	"""Find the intersection point between two lines if it exists (ax + by = c)
	"""
	a1, b1, c1 = l1
	a2, b2, c2 = l2
	d = a1*b2 - a2*b1
	if d:
		x = (c1*b2 - c2*b1)/d
		y = (a1*c2 - a2*c1)/d
		return x, y
	return False


def compute_colision(robot, walls):
	"""Compute collision between the robot and walls
	"""
	offset = robot.radius*sqrt(2)
	for wall in walls:
		if robot.pos.x > wall[0] - offset and robot.pos.x < wall[2] + offset:
			if robot.pos.y > wall[1] - offset and robot.pos.y < wall[3] + offset:
				robot.color = (255, 0, 0)
				return True
	robot.color = (0, 200, 255)
	return False


def find_circle_intersection(p1, r1, p2, r2):
	"""Find the intersection point between two circle if it exists
	"""
	d = distance_points(p1, p2)
	if d >= (r1 + r2):
		print("No circle intersection found ! Returned None")
		return None
	if d < abs(r1 - r2):
		print("No circle intersection found ! Returned None")
		return None
	if abs(r1 - r2) < 1e-6:
		print("No circle intersection found ! Returned None")
		return None

	# https://lucidar.me/fr/mathematics/how-to-calculate-the-intersection-points-of-two-circles/
	a = (r1**2 - r2**2 + d**2)/(2*d)
	h = sqrt(r1**2 - a**2)

	x5 = p1[0] + (a / d) * (p2[0] - p1[0]) 
	y5 = p1[1] + (a / d) * (p2[1] - p1[1]) 

	xi1 = x5 - h * (p2[1] - p1[1])/d
	yi1 = y5 + h * (p2[0] - p1[0])/d

	xi2 = x5 + h * (p2[1] - p1[1])/d
	yi2 = y5 - h * (p2[0] - p1[0])/d

	pi1 = (xi1, yi1)
	pi2 = (xi2, yi2)
	return pi1, pi2


def compute_segment_rect_intersection(line, line_p1, line_p2, rect):
	"""Compute the intersection between two segment

	Args:
		line (tuple): Line
		line_p1 (Vector2): First point of the segment defined by the line
		line_p2 (Vector2): Second point of the segment defined by the line
		rect (list): List of point of a rect (x1, y1, x2, y2)

	Returns:
		bool: If the two segment collide
	"""
	# rect = (x1, y1, x2, y2) avec p1 top-left et p2 bottom-right 
	points = [(rect[0], rect[1], rect[0], rect[3]),
				(rect[0], rect[3], rect[2], rect[3]),
				(rect[2], rect[3], rect[2], rect[1]),
				(rect[2], rect[1], rect[0], rect[1])]
	for p1x, p1y, p2x, p2y in points:
		p1 = Vector2(p1x, p1y)
		p2 = Vector2(p2x, p2y)

		line_rect = compute_line(p1, p2)
		intersection = find_intersection(line, line_rect)

		if intersection is not False:
			if point_in_box(intersection, p1, p2):
				if point_in_box(intersection, line_p1, line_p2):
					return True
	
	return False

def find_n_nearest(ps: list, n: int):
	"""Find the 'n' nearest point from each other

	ERROR IN THIS FUNCTION, PARAMETER d < '1' <- ARBITRARY
	"""
	idpts = []
	pts = ps[:]
	cpt_pop = 0
	n = len(pts)
	for i in range(n):
		p1 = pts.pop(0)
		cpt_pop += 1
		for j, p2 in enumerate(pts):
			k = j + cpt_pop
			d = distance_points(p1, p2)
			if d < 1:
				if i not in idpts:
					idpts.append(i)
				if k not in idpts:
					idpts.append(k)
	
	return idpts


def find_n_nearest_from_point(ps: list, n: int, pos:Vector2):
	"""Find the 'n' nearest point from a position
	"""
	ds = [0]*len(ps)
	for i, p in enumerate(ps):
		ds[i] = distance(pos, p)

	ds_min = [0]*n
	for i in range(n):
		id_min = find_id_min(ds)
		ds_min[i] = id_min + i
		ds.pop(id_min)

	return ds_min


def find_id_min(ls:list):
	"""Find the id of the min of a list"""
	id_min = 0
	min_value = ls[0]
	for i, l in enumerate(ls):
		if l < min_value:
			id_min = i
			min_value = l
	
	return id_min