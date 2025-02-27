from math import sqrt, acos, atan2, atan, pi, sqrt

import numpy as np

### CLASSES

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
		return f"({self.x:.2f}, {self.y:.2f})"
	
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
	

### ANGLES

def get_angle(p1:Vector2, p2:Vector2, p3:Vector2):
	"""Compute the angle with p2 as center
	"""
	a1, b1, _ = compute_line(p1, p2)
	a2, b2, _ = compute_line(p2, p3)

	xy_norm = norm((a1, b1))*norm((a2, b2))
	if xy_norm:
		return acos((a1 * a2 + b1 * b2)/(xy_norm))

	return 0

def get_angle_tuple_deg(p : tuple):
	ang = atan2(p[1], p[0]) * 180 / pi
	return ang + (360)*(ang < 0)

def get_angle_3tuple_deg(p1:tuple, p2:tuple, p3:tuple):
	"""Compute the angle with p2 as center
	"""
	dx1 = p2[0] - p1[0]
	dx2 = p2[0] - p3[0]
	dy1 = p2[1] - p1[1]
	dy2 = p2[1] - p3[1]

	ang1 = atan2(dy1, dx1)
	ang2 = atan2(dy2, dx2)

	if ang1 < 0:
		ang1 += 2 * pi

	if ang2 < 0:
		ang2 += 2 * pi

	return abs(ang1 - ang2) * 180 / pi



def get_absolute_angle(p1:tuple, p2:tuple):
	"""Compute and return the absolute angle between two points from (0, 0)
	"""
	ang = atan2(p1[1] - p2[1], p1[0] - p2[0])
	return ang + (2 * pi)*(ang < 0)


def get_signed_angle(p1:tuple, p2:tuple):
	"""Compute and return the absolute angle between two points from (0, 0)
	"""
	ang = atan2(p1[1] - p2[1], p1[0] - p2[0])
	return ang

### LINES

def compute_line(p1:Vector2, p2:Vector2):
	"""Compute line equation : ax + by = c
	"""
	a = p2.y - p1.y
	b = p1.x - p2.x
	c = p1.x*(p2.y - p1.y) - p1.y*(p2.x - p1.x)
	return a,b,c


def compute_line_tuple(p1:tuple, p2:tuple):
	"""Compute line equation : ax + by = c
	"""
	a = p2[1] - p1[1]
	b = p1[0] - p2[0]
	c = p1[0]*(p2[1] - p1[1]) - p1[1]*(p2[0] - p1[0])
	return a,b,c


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


def compute_parallel_line(line, dist) -> list:
	"""Compute coefficient of a parallel translated line from dist
	"""
	a, b, c = line
	return a, b, c + dist * (a ** 2 + b ** 2) ** 0.5


def move_on_line(ids1, ids2, ids1_float, ids2_float):
    idx, idy = ids1
    idx_float, idy_float = ids1_float
    
    a, b, _ = compute_line_tuple(ids1_float, ids2_float)
    if b:
        m = -a / b
    
    ids = []

    if ids1[0] == ids2[0]:  # Vertical line
        direction = 1 * (ids1[1] < ids2[1]) - 1 * (ids1[1] >= ids2[1])
        for idy in range(ids1[1], ids2[1] + direction, direction):
            ids.append((idx, idy))
        return ids

    if ids1[1] == ids2[1]:  # Horizontal line
        direction = 1 * (ids1[0] < ids2[0]) - 1 * (ids1[0] >= ids2[0])
        for idx in range(ids1[0], ids2[0] + direction, direction):
            ids.append((idx, idy))
        return ids

    direction = np.sign(ids1[0] - ids2[0])

    n = 0
    if abs(m) < 1:
        while n <= abs(ids1[0] - ids2[0]):
            ids.append((idx, idy - 1))
            ids.append((idx, idy + 1))
            ids.append((idx, idy))
            
            idx_float -= 1 * direction
            idy_float -= m * direction

            idx = int(idx_float)
            idy = int(idy_float)
            n += 1
    else:
        while n <= abs(ids1[1] - ids2[1]):
            ids.append((idx + 1, idy))
            ids.append((idx - 1, idy))
            ids.append((idx, idy))

            idx_float -= abs(1 / m) * direction
            idy_float -= np.sign(m) * direction

            idx = int(idx_float)
            idy = int(idy_float)

            n += 1

    return ids

### MATHS

def norm(u:tuple):
	"""Get the norm of a tuple
	"""
	return sqrt(u[0]**2 + u[1]**2)


def distance(p1:Vector2, p2:tuple):
	"""Compute the distance between a Vector2 point and a tuple
	"""
	return sqrt((p2[0] - p1.x)**2 + (p2[1] - p1.y)**2)


def distance_tuple(p1:tuple, p2:tuple):
	"""Get the distance between two points as tuple
	"""
	return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def dot_product(v1:tuple, v2:tuple):
	"""Dot product of two tuple
	"""
	return v1[0] * v2[0] + v1[1] * v2[1]


def normalize_vec(v:tuple):
	x, y = v
	dist = sqrt(x*x+y*y)
	return x/dist, y/dist


def orthogonal_projection(p:tuple, line:tuple) -> float:
	"""Compute the tinyest distance between a point and a line
	#### "https://fr.wikipedia.org/wiki/Distance_d'un_point_%C3%A0_une_droite"
	"""
	a, b, c = line
	return abs(a*p[0] + b*p[1] - c)/sqrt(a**2 + b**2)

def wall_orthogonal_point(p:tuple, wall):
	p1, p2 = left_point(wall.p1, wall.p2)
	vec = (p2.x - p1.x , p2.y - p1.y)
	line = compute_line(p1, p2)
	a, b, c = line

	dir_vec = normalize_vec(vec)

	yp1 = evaluate_line(line, p[0])
	y_point = (p[0], yp1)
	d = abs(a*p[0] + b*p[1] - c)/sqrt(a**2 + b**2)


	h2 = (p[1] - yp1)**2 - d**2
	if h2 > 0:
		h = sqrt(h2)
		if y_point[1] > p[1]:
			if p1.y > p2.y:
				H = (y_point[0] + h*dir_vec[0], y_point[1] + h*dir_vec[1])
			else:
				H = (y_point[0] - h*dir_vec[0], y_point[1] - h*dir_vec[1])
		else:
			if p1.y > p2.y:
				H = (y_point[0] - h*dir_vec[0], y_point[1] - h*dir_vec[1])
			else:
				H = (y_point[0] + h*dir_vec[0], y_point[1] + h*dir_vec[1])

		return H
	else:
		return y_point




def sign(a:float):
	return 1 * (a>=0) - 1 *(a<0)

### INTERSECTIONS

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


def find_segment_intersection(p11:Vector2, p12:Vector2, p21:Vector2, p22:Vector2):
	l1 = compute_line(p11, p12)
	l2 = compute_line(p21, p22)
	p_inter = find_intersection(l1, l2)
	if p_inter:
		if point_in_box(p_inter, p11, p12):
			if point_in_box(p_inter, p21, p22):
				return p_inter
	return False

def find_segment_intersection_tuple(p11, p12, p21, p22):
	l1 = compute_line_tuple(p11, p12)
	l2 = compute_line_tuple(p21, p22)
	p_inter = find_intersection(l1, l2)
	if p_inter:
		if point_in_box_tuple(p_inter, p11, p12):
			if point_in_box_tuple(p_inter, p21, p22):
				return p_inter
	return False


def find_circle_intersection(p1, r1, p2, r2):
	"""Find the intersection point between two circle if it exists
	"""
	d = distance_tuple(p1, p2)
	if d >= (r1 + r2):
		print("No circle intersection found (d >= (r1 + r2)) ! Returned None")
		return None
	if d < abs(r1 - r2):
		print("No circle intersection found (d < abs(r1 - r2)) ! Returned None")
		return None
	if d < 1e-6 and abs(r1 - r2) < 1e-6:
		print("No circle intersection found (abs(r1 - r2) < 1e-6) ! Returned None")
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
					return intersection
	
	return False


def compute_segment_rect_intersection_tuple(line, line_p1, line_p2, rect):
	"""Compute the intersection between two segment

	Args:
		line (tuple): Line
		line_p1 (tuple): First point of the segment defined by the line
		line_p2 (tuple): Second point of the segment defined by the line
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
		p1 = (p1x, p1y)
		p2 = (p2x, p2y)

		line_rect = compute_line_tuple(p1, p2)
		intersection = find_intersection(line, line_rect)

		if intersection is not False:
			if point_in_box_tuple(intersection, p1, p2):
				if point_in_box_tuple(intersection, line_p1, line_p2):
					return intersection
	
	return False


# CHECK IF IN SHAPE

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

def point_in_box_tuple(p:tuple, p1:tuple, p2:tuple, error_offset:float = 1e-6):
	"""Check if the point 'p' is in the box defined by p1 and p2

	Args:
		p (tuple): Point to be checked
		p1 (tuple): First point
		p2 (tuple): Second point
		error_offset (float, optional): Error offset to overcome python binary error on float. Defaults to 1e-6.

	Returns:
		bool: If the point is in the box
	"""
	if p[0] > min(p1[0], p2[0]) - error_offset and p[0] < max(p2[0], p1[0]) + error_offset:
		if p[1] > min(p1[1], p2[1]) - error_offset and p[1] < max(p2[1], p1[1]) + error_offset:
			return True
	return False

def point_in_circle(p:tuple, center:tuple, radius:float) -> bool:
	"""Check if the point is in the circle
	"""
	if distance_tuple(p, center) <= radius:
		return True
	return False


def point_in_polygon(point, polygon) -> bool:
	"""Check if the point is in the polygon

	Args:
		point (tuple): Point to be checked
		polygon (list): List of point defining the polygon
	"""
	x, y = point
	n = len(polygon)
	inside = False
	p1x, p1y = polygon[0]
	for i in range(1, n + 1):
		p2x, p2y = polygon[i % n]
		if y > min(p1y, p2y):
			if y <= max(p1y, p2y):
				if x <= max(p1x, p2x):
					if p1y != p2y:
						xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
					if p1x == p2x or x <= xinters:
						inside = not inside
		p1x, p1y = p2x, p2y
	return inside

### NEAREST POINTS

def middle_point(p1:tuple, p2:tuple):
	return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

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
			d = distance_tuple(p1, p2)
			if d < 1:
				if i not in idpts:
					idpts.append(i)
				if k not in idpts:
					idpts.append(k)
	
	return idpts


### MIN INDICES

def find_id_min(ls:list):
	"""Find the id of the min of a list"""
	id_min = 0
	min_value = ls[0]
	for i, l in enumerate(ls):
		if l < min_value:
			id_min = i
			min_value = l
	
	return id_min

def left_point(p1:Vector2, p2:Vector2):
	if p1.x > p2.x:
		return p2, p1
	return p1, p2

### CONVEX HULL

def find_leftmost_point(points:list):
    leftmost_point = points[0]
    for point in points[1:]:
        if point[0] < leftmost_point[0]:
            leftmost_point = point
        elif point[0] == leftmost_point[0] and point[1] > leftmost_point[1]:
            leftmost_point = point
    return leftmost_point

def orientation(p:tuple, q:tuple, r:tuple):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2

def convex_hull(points:list):
    n = len(points)
    if n < 3:
        return []
    hull = []
    l = find_leftmost_point(points)
    p = l
    q = None
    while True:
        hull.append(p)
        q = points[0]
        for r in points[1:]:
            o = orientation(p, q, r)
            if o == 2 or (o == 0 and ((q[0] - p[0])**2 + (q[1] - p[1])**2) < ((r[0] - p[0])**2 + (r[1] - p[1])**2)):
                q = r
        p = q
        if p == l:
            break
    return hull

### DISTRIBUTION

def normal_distribution(mean, covariance):
	return np.random.multivariate_normal(mean, covariance)
