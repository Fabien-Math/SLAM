from math import sqrt, acos
from numpy import array, dot, arctan2




class Vector2:
	"""
	### Vector with two elements (x, y)
	"""
	def __init__(self, x, y) -> None:
		self.x = x
		self.y = y

	def add(self, x, y):
		self.x += x
		self.y += y
	
	def __str__(self):
		return f"{self.x}, {self.y}"
	
	def copy(self):
		return Vector2(self.x, self.y)

	def to_tuple(self):
		return (self.x, self.y)

	def middle_point(self, p):
		return Vector2((self.x + p.x)/2, (self.y + p.y)/2)
	
	def distance(self, p):
		return sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
	
def get_angle(p1:Vector2, p2:Vector2, p3:Vector2):
	"""
	# Compute the angle [p1, p2, p3]
	"""
	a1, b1, _ = compute_line(p1, p2)
	a2, b2, _ = compute_line(p2, p3)
	x = array([a1, b1])
	y = array([a2, b2])
	xy_norm = norm(x)*norm(y)
	if xy_norm:
		return acos(dot(x,y)/(xy_norm))

	return 0
	
def norm(u:tuple):
	"""
	### Get the norm of a tuple
	"""
	return sqrt(u[0]**2 + u[1]**2)

def evaluate_line(l, x):
	if l[1] != 0:
		return (l[2] - l[0] * x) / l[1]
	else:
		return x


def distance(p1:Vector2, p2):
	"""
	### Get the distance between a Vector2 point and a tuple
	"""
	return sqrt((p2[0] - p1.x)**2 + (p2[1] - p1.y)**2)

def distance_points(p1, p2):
	"""
	### Get the distance between two points as tuple
	"""
	return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def compute_line(p1:Vector2, p2:Vector2):
	"""
	### Compute line equation : ax + by = c
	"""
	a = p2.y - p1.y
	b = p1.x - p2.x
	c = p1.x*(p2.y - p1.y) - p1.y*(p2.x - p1.x)
	return a,b,c

def point_in_box(p:tuple, p1:Vector2, p2:Vector2, error_offset:float = 1e-6):
	if p[0] > min(p1.x, p2.x) - error_offset and p[0] < max(p2.x, p1.x) + error_offset:
		if p[1] > min(p1.y, p2.y) - error_offset and p[1] < max(p2.y, p1.y) + error_offset:
			return True
	return False

# def find_intersection(a1:float, b1:float, c1:float, a2:float, b2:float, c2:float):
def find_intersection(l1, l2):
	"""
	### Find the intersection point between two lines if it exists (ax + by = c)
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
	"""
	Compute collision between the robot and walls
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
	"""
	### Find the intersection point between two circle if it exists
	"""
	r = distance_points(p1, p2)
	if r > (r1 + r2):
		print("No circle intersection found ! Returned None")
		return None

	xi1 = 0.5*(p2[0] + p1[0] + r1**2/(p2[0] - p1[0]))
	yi1 = 0.5*(p2[1] + p1[1] - r2**2/(p2[1] - p1[1]))
	xi2 = 0.5*(p2[0] + p1[0] - r2**2/(p2[0] - p1[0]))
	yi2 = 0.5*(p2[1] + p1[1] + r1**2/(p2[1] - p1[1]))

	pi1 = (xi1, yi1)
	pi2 = (xi2, yi2)
	return pi1, pi2


def compute_segment_rect_intersection(line, line_p1, line_p2, rect):
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
	"""
	### Find the 'n' nearest point from each other
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