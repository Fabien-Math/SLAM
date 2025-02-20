from my_methode import compute_path
from shortest_path_ca import compute_shortest_path, compute_and_save_shortest_path, display_grid, plt

### WORLD INITIALISATION
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
	 ('rect', (850, 150), (900, 150), (900, 1000), (850, 1000))],

	[('rect', (850, 0), (900, 0), (900, 200), (850, 200)),
	 ('rect', (0, 300), (900, 300), (900, 350), (0, 350)),
	 ('rect', (350, 350), (400, 350), (400, 600), (350, 600)),
	 ('rect', (600, 500), (1200, 500), (1200, 550), (600, 550))]
]

imap = 1
obstacles = map[imap-1]

map_size = (1200, 700)
if imap == 8:
	source_pos = (100, 100)
	wp_pos = (100, 450)
else:
	source_pos = (100, 350)
	wp_pos = (1100, 350)

safe_range = 0
granularity = 20
scale = 1/2

save_anim_name = "map_" + str(imap) + "_scale_" + f"{int(1 / scale):g}_g20"
print(save_anim_name)

path_dist = compute_path(source_pos, wp_pos, map_size, obstacles, safe_range)
if path_dist:
	print(f"Computed path length : {path_dist:.3f} um")

shortest_path_dist, lattice = compute_and_save_shortest_path(source_pos, wp_pos, map_size, scale, obstacles, granularity, save_anim_name)
# shortest_path_dist, lattice = compute_shortest_path(source_pos, wp_pos, map_size, scale, obstacles, granularity)
print(f"Shortest computed path length : {shortest_path_dist:.3f} um")
# display_grid(lattice, scale, wp_pos, shortest_path_dist, obstacles)

# plt.show()
