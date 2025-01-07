import cv2
from util import *

def build_voronoi_diagram(robot, c_range):
    """
    Build Voronoi diagram with walls
    
    Args:
        obstacles (list[tuple]): List of lidar points
    
    Returns:
        list: List of centres and edges of the Voronoi diagram
    """
    live_grid_map = robot.live_grid_map
    idx, idy = robot.live_grid_map.coord_to_ids(robot.pos_calc.to_tuple())
    range_ids = int(c_range/live_grid_map.size)

    live_grid_map_list_size = live_grid_map.list_size

    # Definition de la zone pour effectuer la subdivision
    rect = (int(robot.pos.x - c_range), int(robot.pos.y - c_range), 2*c_range + 1, 2*c_range + 1)
    subdiv = cv2.Subdiv2D(rect)
    subdiv.insert(robot.pos_calc.to_tuple())

    for i in range(idy - range_ids, idy + range_ids + 1):
        for j in range(idx - range_ids, idx + range_ids + 1):
            if 0 < j < live_grid_map_list_size and 0 < i < live_grid_map_list_size:
                value = live_grid_map.map[i, j]
                if value > 0:
                    point = live_grid_map.ids_to_center(j, i)
                    if point_in_circle(point, robot.pos.to_tuple(), c_range):
                        # Insertion des points dans la subdivision pour en extraire le diagram de Voronoi
                        subdiv.insert(point)
                    

    # Calcul du diagramme de Voronoi
    voronoi_diagram  = subdiv.getVoronoiFacetList([])	
    thinned_voronoi_polygon = thin_polygon(voronoi_diagram[0][0], -robot.radius)
    return voronoi_diagram, thinned_voronoi_polygon


def thin_polygon(polygon, offset):
    """
    Offset the polygon and make sure no point are in the safe range
    Other way which don't care of overlap : https://stackoverflow.com/questions/68104969/offset-a-parallel-line-to-a-given-line-python/68109283#68109283

    Args:
        polygon (_type_): List of point defining the polygon
        offset (_type_): Signed distance to offset
    """
    num_points = len(polygon)

    thinned_polygon = []
    lines = [None] * num_points
    parallel_lines = [None] * num_points

    # Build lines and parallel lines
    for i in range(num_points):
        lines[i] = compute_line_tuple(polygon[i-1], polygon[i])
        parallel_lines[i] = compute_parallel_line(lines[i], offset)

    # Find all parallel line intersections
    intersections = []
    for i in range(num_points):
        for j in range(num_points):
            if j == i:
                continue
            p = find_intersection(parallel_lines[j], parallel_lines[i])
            if p:
                if point_in_polygon(p, polygon):
                    intersections.append(p)

    # Check points to be at the right distance from the polygon walls
    for p in intersections:
        to_be_added = True
        for line in lines:
            if orthogonal_projection(p, line) < abs(offset)*0.98:
                to_be_added = False
                break
            
        if to_be_added:
            thinned_polygon.append(p)
    
    return convex_hull(thinned_polygon)


def draw_voronoi_diagram(voronoi_diagram, window):
    """Draw the Voronoi diagram on the screen

    Args:
        window (surface): Surface on which to draw
    """
    if voronoi_diagram is None:
        return None
    
    # Draw complete Voronoi diagram
    for f in voronoi_diagram[0]:
        polygon = [(float(f[i][0]), float(f[i][1])) for i in range(len(f))]
        pygame.draw.polygon(window, (100, 255, 100), polygon, 2)
    


def draw_polygon(thinned_voronoi_polygon, voronoi_polygon, window):
    """Draw the Voronoi diagram on the screen

    Args:
        window (surface): Surface on which to draw
    """
    
    if len(thinned_voronoi_polygon) < 2:
        return
    # Draw robot Voronoi diagram
    robot_polygon_wrong_type = voronoi_polygon
    robot_polygon = [(float(robot_polygon_wrong_type[i][0]), float(robot_polygon_wrong_type[i][1])) for i in range(len(robot_polygon_wrong_type))]
    pygame.draw.polygon(window, (100, 255, 100), robot_polygon, 1)

    # Draw thinned robot Voronoi diagram
    robot_polygon_thinned_wrong_type = thinned_voronoi_polygon
    robot_polygon_thinned = [(float(robot_polygon_thinned_wrong_type[i][0]), float(robot_polygon_thinned_wrong_type[i][1])) for i in range(len(robot_polygon_thinned_wrong_type))]
    pygame.draw.polygon(window, (255, 255, 100), robot_polygon_thinned, 1)
