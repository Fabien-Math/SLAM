import pygame
from world import World
import time

class Window:
	def __init__(self, window_size, window_title):
		# Pygame initialization
		pygame.init()

		### WINDOW INITIALISATION
		self.window_size = window_size
		self.window = pygame.display.set_mode(window_size)
		pygame.display.set_caption(window_title)

		self.time = 0
		self.desired_fps = 20

	def update(self, world):
		# DRAW THE SCENE
		if time.time() - self.time > 1/self.desired_fps:
			self.update_display(world)
			self.time = time.time()


	def update_display(self, world:World):
		"""Update the display

		Args:
			world (World): World class
			dt (float): Display time step
		"""
		window = self.window
		map = world.map
		robots = world.robots
		# Erase the display
		window.fill((150, 150, 150))

		map.draw_map(window)
		# map.draw_subdiv_points(window)

		# Draw communication links
		self.draw_links(world)

		
		for robot in robots:
			if robot.controller.mode:
				if robot.controller.waypoint is not None:
					pygame.draw.circle(window, (255, 0, 255), robot.controller.waypoint, robot.controller.waypoint_radius)
				if robot.controller.local_waypoint is not None:
					pygame.draw.circle(window, (255, 0, 0), robot.controller.local_waypoint, robot.controller.local_waypoint_radius)

			# # Draw the robot
			if robot.id == 1:
				robot.live_grid_map.draw(window)
				# robot.live_grid_map.draw_occurance(window)
		
		for robot in robots:
			robot.draw(window)

		# Draw simulation time
		self.write_time(world.time)

		# Update scene display
		pygame.display.update()


	def write_time(self, time):
		"""Write the FPS on the top-right of the window

		Args:
			Time (float): Simulation time
		"""
		font = pygame.font.Font('freesansbold.ttf', 20)
		formated_time = f"{time//3600:g} h, {(time - (time//3600)*3600)//60:g} m, {time - (time//3600)*3600 - ((time - (time//3600)*3600)//60)*60:.3f} s"
		text = font.render(formated_time, True, (255, 255, 255))

		textRect = text.get_rect()

		textRect.center = (self.window_size[0]*0.5, self.window_size[1]*0.05)
		self.window.blit(text, textRect)


	def draw_links(self, world:World):
		for rs in world.linked_robot:
			r1, r2 = world.robots[rs[0]], world.robots[rs[1]]
			pygame.draw.line(self.window, (100, 50, 255), r1.pos, r2.pos)


	def destroy(self):
		pygame.quit()




def write_text(text, window, pos):
	"""Write the FPS on the top-right of the window

	Args:
		window (surface): Window on which writting
		pos (tuple): Position of the text on the window
	"""
	font = pygame.font.Font('freesansbold.ttf', 16)
	text = font.render(text, True, (255, 255, 255))

	textRect = text.get_rect()

	textRect.center = pos
	window.blit(text, textRect)
	pygame.display.update()



def draw_point(window, position, delay, color=(255, 0, 255), size=5):
	pygame.draw.circle(window, color, position, size)
	pygame.display.update()
	pygame.time.delay(delay)

def delay(t:int):
	pygame.time.delay(t)