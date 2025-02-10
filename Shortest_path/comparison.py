from my_methode import compute_path
from shortest_path_ca import compute_shortest_path, compute_and_save_shortest_path, display_grid, plt

### WORLD INITIALISATION
robot_pos = (100, 350)
last_static_pos = robot_pos
waypoint_pos = (1100, 350)

map = [
	[('circle', (600, 350), 200)],

	[('rect', (400, 150), (800, 150), (800, 550), (400, 550))],

	[('circle', (650, 350), 100),
	 ('circle', (550, 200), 100),
	 ('circle', (550, 500), 100)],
	
	[('rect', (550, 0), (650, 0), (650, 550), (550, 550)),
	 ('rect', (550, 600), (650, 600), (650, 1000), (550, 1000))],
	
	[('rect', (700, 0), (800, 0), (800, 600), (700, 600)),
	 ('rect', (400, 100), (500, 100), (500, 1000), (400, 1000))],
	 
	[('rect', (300, 150), (900, 150), (900, 200), (300, 200)),
	 ('rect', (300, 500), (900, 500), (900, 550), (300, 550)),
	 ('rect', (850, 150), (900, 150), (900, 550), (850, 550))],
	 
	[('rect', (300, 150), (900, 150), (900, 200), (300, 200)),
	 ('rect', (850, 150), (900, 150), (900, 1000), (850, 1000))]
]

imap = 5
obstacles = map[imap]

map_size = (1200, 700)
source_pos = (100, 350)
wp_pos = (1100, 350)
safe_range = 0
granularity = 5
scale = 1/5

save_anim_name = "map_" + str(imap) + "_scale_" + scale.__format__(':.2f')

path_dist = compute_path(source_pos, wp_pos, map_size, obstacles, safe_range)
print(f"Computed path length : {path_dist:.3f} um")

shortest_path_dist, lattice = compute_and_save_shortest_path(source_pos, wp_pos, map_size, scale, obstacles, granularity, save_anim_name)
print(f"Shortest computed path length : {shortest_path_dist:.3f} um")

# display_grid(lattice, scale, wp_pos, shortest_path_dist, obstacles)
# plt.show()
