import numpy as np
import matplotlib.pyplot as plt

# Mock util methods
def distance_tuple(a, b):
    """Calculate the Euclidean distance between two points a and b."""
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

# Mock Live_grid_map class
class Live_grid_map:
    def __init__(self, grid):
        self.map = grid

    def ids_to_center(self, idx, idy):
        """Mock method to return the grid center of a cell."""
        return (idx + 0.5, idy + 0.5)

def node_with_lowest_score(open_set, open_cost):
    min_cost = float('inf')
    p = None
    for i in range(len(open_cost)):
        if open_cost[i] < min_cost:
            min_cost = open_cost[i]
            p = i
    return p

def cost(p, pos_start, pos_end, live_grid):
    pos_p = live_grid.ids_to_center(p[0], p[1])
    cost_from_start = distance_tuple(pos_p, pos_start)
    cost_from_end = distance_tuple(pos_p, pos_end)
    return cost_from_start + cost_from_end

def traversable(grid, p, safe_radius):
    idx, idy = p
    rows, cols = grid.shape
    for i in range(max(0, idx - safe_radius), min(rows, idx + safe_radius + 1)):
        for j in range(max(0, idy - safe_radius), min(cols, idy + safe_radius + 1)):
            if grid[i, j] > 0:
                return False
    return True

def a_star(live_grid: Live_grid_map, start, end, safe_radius):
    grid = live_grid.map

    pos_start = live_grid.ids_to_center(start[0], start[1])
    pos_end = live_grid.ids_to_center(end[0], end[1])

    open_set = [start]
    open_cost = [cost(start, pos_start, pos_end, live_grid)]
    came_from = {}
    close_set = set()

    dirs = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    while open_set:
        id_current = node_with_lowest_score(open_set, open_cost)
        current = open_set.pop(id_current)
        open_cost.pop(id_current)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        close_set.add(current)

        for dir in dirs:
            neigh = (current[0] + dir[0], current[1] + dir[1])

            if not (0 <= neigh[0] < grid.shape[0] and 0 <= neigh[1] < grid.shape[1]):
                continue
            if not traversable(grid, neigh, safe_radius) or neigh in close_set:
                continue

            tentative_cost = cost(neigh, pos_start, pos_end, live_grid)

            if neigh not in open_set:
                open_set.append(neigh)
                open_cost.append(tentative_cost)
                came_from[neigh] = current

    return None

# Visualization function
def visualize_path(grid, path, start, end):
    plt.imshow(grid, cmap='gray_r', origin='upper')
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, color='blue', label='Path')
    plt.scatter(start[1], start[0], color='green', label='Start')
    plt.scatter(end[1], end[0], color='red', label='End')
    plt.legend()
    plt.title("A* Pathfinding")
    plt.show()


import time
# Demonstration script
if __name__ == "__main__":
    # Create a larger test grid
    grid = np.zeros((100, 100))
    grid[30:70, 50:55] = 1  # Vertical wall
    grid[35:60, 30:70] = 1  # Horizontal wall

    # Initialize the Live_grid_map
    live_grid = Live_grid_map(grid)

    # Define start and end points
    start = (0, 0)
    end = (99, 99)
    safe_radius = 2

    a = time.perf_counter()
    # Run A* algorithm
    path = a_star(live_grid, start, end, safe_radius)
    b = time.perf_counter()
    print("Enlapsed time : ", b-a)
    print("FPS : ", 1/(b-a))

    # if path:
    #     print("Path found:", path)
    # else:
    #     print("No path found.")

    # Visualize the result
    visualize_path(grid, path, start, end)