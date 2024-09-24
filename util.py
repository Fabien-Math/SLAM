from math import sqrt

class Vector2:
	def __init__(self, x, y) -> None:
		self.x = x
		self.y = y

	def add(self, x, y):
		self.x += x
		self.y += y
	
	def __str__(self):
		return f"{self.x}, {self.y}"
	

def distance(p1:Vector2, p2):
	return sqrt((p2[0] - p1.x)**2 + (p2[1] - p1.y)**2)

def compute_line(p1:Vector2, p2:Vector2):	# ax + by = c
	a = p2.y - p1.y
	b = p1.x - p2.x
	c = p1.x*(p2.y - p1.y) - p1.y*(p2.x - p1.x)
	return a,b,c

# def find_intersection(a1:float, b1:float, c1:float, a2:float, b2:float, c2:float):
def find_intersection(l1, l2):
	a1, b1, c1 = l1
	a2, b2, c2 = l2
	d = a1*b2 - a2*b1
	if d:
		x = (c1*b2 - c2*b1)/d
		y = (a1*c2 - a2*c1)/d
		return x, y
	return False


def compute_colision(robot, walls):
	offset = robot.radius*sqrt(2)
	for wall in walls:
		if robot.pos.x > wall[0] - offset and robot.pos.x < wall[2] + offset:
			if robot.pos.y > wall[1] - offset and robot.pos.y < wall[3] + offset:
				robot.color = (255, 0, 0)
				return True
	robot.color = (0, 200, 255)
	return False