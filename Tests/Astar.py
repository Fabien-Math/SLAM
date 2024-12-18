import util as ut
from live_grid_map import Live_grid_map


def node_with_lowest_score(open_set, open_cost):
    if not len(open_set):
        return None

    min_cost = 1e9
    p = None
    for i in range(len(open_cost)):
        if open_cost[i] < min_cost:
            min_cost = open_cost[i]
            p = open_set[i]
    
    return p

def cost(p, pos_start, pos_end, live_grid):
    pos_p = live_grid.ids_to_center(p[0], p[1])

    cost_from_start = ut.distance_tuple(pos_p, pos_start)
    cost_from_end = ut.distance_tuple(pos_p, pos_end)
    return cost_from_start + cost_from_end
    
def traversable(grid, p, safe_radius):
    idx, idy = p
    for i in range(idy - safe_radius, idy + safe_radius):
        for j in range(idx - safe_radius, idx + safe_radius):
            if grid[i, j] > 0:
                return False
    
    return True

def a_star(live_grid: Live_grid_map, start, end, safe_radius):
    grid = live_grid.map

    pos_start = live_grid.ids_to_center(start[0], start[1])
    pos_end = live_grid.ids_to_center(end[0], end[1])
    
    open_set = []
    open_cost = []
    close_set = []
    close_cost = []
    open_set.append(start)
    open_cost.append(cost(start, pos_start, pos_end, live_grid))

    dirs = [(0, 1), (0,-1), (-1, -1), (-1, 0), (-1, 1), (1, 0), (1, 1), (1, -1)]

    n = 0
    while n < 1000:
        if len(open_set):
            id_current = node_with_lowest_score(open_set, open_cost)
            current = open_set.pop(id_current)
            current_cost = open_cost.pop(id_current)
        close_set.append(current)
        close_cost.append(current_cost)

        if current == end:
            return close_set
        
        for dir in dirs:
            neigh_idx = current[0][0] + dir[0]
            neigh_idy = current[0][1] + dir[1]
            neigh = (neigh_idx, neigh_idy)
            if not traversable(grid, neigh, safe_radius) or neigh in close_set:
                continue
        
            neigh_cost = cost(neigh, pos_start, pos_end, live_grid)
            if neigh_cost < current_cost:
                current = neigh
                current_cost = neigh_cost
            elif neigh not in open_set:
                open_set.append(neigh)
                open_cost.append(neigh_cost)

        n += 1
