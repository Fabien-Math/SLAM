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

	def update(self, world:World, draw_robots=True, draw_lgm=True, draw_wps=True, draw_links=True, draw_robot_nb = True, draw_map=True):
		# DRAW THE SCENE
		if time.time() - self.time < 1/self.desired_fps:
			return
		
		self.time = time.time()

		window = self.window
		map = world.map
		robots = world.robots
		# Erase the display
		window.fill((150, 150, 150))

		# Draw the map
		if draw_map:
			map.draw_map(window)
		# map.draw_subdiv_points(window)


		if draw_lgm:
			robots[world.active_robot].live_grid_map.draw(window)
		if draw_wps:
			for robot in robots:
				if robot.controller.mode:
					if robot.controller.waypoint is not None:
						pygame.draw.circle(window, (255, 0, 255), robot.controller.waypoint, robot.controller.waypoint_radius)
					if robot.controller.local_waypoint is not None:
						pygame.draw.circle(window, (255, 0, 0), robot.controller.local_waypoint, robot.controller.local_waypoint_radius)

		if draw_robots:
			for robot in robots:
				robot.draw(window)
		else:
			robots[world.active_robot].draw(window)
			
		if draw_robot_nb:
			for robot in robots:
				if robot.communicator.master:
					write_text(f"{robot.id}", window, (robot.pos[0] + 10, robot.pos[1] + 10), text_flush='centre', color=(255, 100, 100), update=False)
				else:
					write_text(f"{robot.id}", window, (robot.pos[0] + 10, robot.pos[1] + 10), text_flush='centre', update=False)
			
		write_text(f"Active robot : {world.active_robot}", window, (20, self.window_size[1]-20), text_flush='left', update=False)

		# Draw communication links
		if draw_links:
			self.draw_links(world)

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




def write_text(text, window, pos, text_flush='center', color=(255,255,255), update=True):
	"""Write the FPS on the top-right of the window

	Args:
		window (surface): Window on which writting
		pos (tuple): Position of the text on the window
	"""
	font = pygame.font.Font('freesansbold.ttf', 16)
	text = font.render(text, True, color)

	textRect = text.get_rect()

	if text_flush == 'centre':
		textRect.center = pos
	if text_flush == 'left':
		textRect.midleft = pos
	if text_flush == 'right':
		textRect.midright = pos
		
	window.blit(text, textRect)
	if update:
		pygame.display.update()



def draw_point(window, position, delay, color=(255, 0, 255), size=5):
	pygame.draw.circle(window, color, position, size)
	pygame.display.update()
	pygame.time.delay(delay)

def delay(t:int):
	pygame.time.delay(t)