import numpy as np
from math import sin, cos
import cv2
from util import *
import time



# Calcul et renvoie le point appartenant au polygone le plus proche d'un autre point 
def closest_point_on_polygon(polygon, point):
    closest_point = None
    closest_distance = float('inf')
    for i in range(len(polygon)):
        segment_start = polygon[i]
        segment_end = polygon[(i+1) % len(polygon)]
        current_point = point_on_segment(segment_start, segment_end, point)
        current_distance = distance(current_point, point)
        if current_distance < closest_distance:
            closest_point = current_point
            closest_distance = current_distance
    return closest_point

# Test si le point trouvé est sur le segment et renvoie le point le plus proche sur le segment
def point_on_segment(segment_start, segment_end, point):
    segment_vec = [segment_end[i]-segment_start[i] for i in range(2)]
    point_vec = [point[i]-segment_start[i] for i in range(2)]
    segment_length = distance(segment_start, segment_end)
    projection = dot_product(segment_vec, point_vec) / segment_length
    if projection < 0:
        return segment_start
    elif projection > segment_length:
        return segment_end
    else:
        return [segment_start[i] + projection * segment_vec[i]/segment_length for i in range(2)]






# Amicissement du polygone
def thinPolygon(points, distance) -> list[list[float, float]]:  
    """
    #   Calcul toutes les lignes parallele aux segments du polygone et les translate de 'distance' vers l'intérieur
    #   Calcul les intersection de chaque ligne et ne garde que celle qui sont à l'intérieur du polygone
    #   Supprime tous les points qui sont à une distance inférieur à 'distance' d'un segement du polygone
    #   Renvoie l'enveloppe convexe des points restants grace à l'algorithme de Jarvis
    """

    lineCoefs = []
    for i in range(len(points)):
        l1 = compute_line_tuple(points[i], points[i-1])
        l2 = compute_parallel_line(l1 ,distance)
        l3 = compute_line_tuple(points[i-1], points[i-2])
        l4 = compute_parallel_line(l3 ,distance)
        if l2 not in lineCoefs:
            lineCoefs.append(l2)
        if l4 not in lineCoefs:
            lineCoefs.append(l4)

    temporaryPolygon = []
    for i in range(len(lineCoefs)):
        for j in range(i+1, len(lineCoefs)):
            l1 = lineCoefs[i]
            l2 = lineCoefs[j]
            p = find_intersection(l1, l2)
            if point_in_polygon(p, points):
                temporaryPolygon.append(p)
    
    thinnedPolygon = []
    for i in range(len(temporaryPolygon)):
        for j in range(len(points)):
            line = compute_line_tuple(points[j], points[j-1])
            if orthogonal_projection(temporaryPolygon[i], line) >= distance:
                thinnedPolygon.append(temporaryPolygon[i])

    return convex_hull(thinnedPolygon)



def buildVoronoiDiagram(robot, points):
    """
    ## Description
        Construction du diagramme de Voronoi avec les obstacles et le bateau
    
    ### Args:
        robot (BeaconRobot): Objet robot
        obstacles (list[tuple]): List of lidar points
    
    ### Returns:
        list: List of centres and edges of the Voronoi diagram
    """
    # Definition de la zone pour effectuer la subdivision
    rect = (int(robot.x - 0), int(robot.y - 10), 20, 20)
    subdiv = cv2.Subdiv2D(rect)

    # Insertion des points dans la subdivision pour en extraire le diagram de Voronoi
    subdiv.insert(robot.position)
    for point in points:
        if point_in_circle(point.position, robot.position, 4):
            subdiv.insert(point.position)

    # Calcul du diagramme de Voronoi
    (facets, centers) = subdiv.getVoronoiFacetList([])

    return facets, centers