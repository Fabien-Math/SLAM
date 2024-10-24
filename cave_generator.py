import numpy as np
import matplotlib.pyplot as plt
from util import Vector2

def compute_contours(triangles, points, values, thresh):
# List of list two points defining lines
    contours = []
    for tri in triangles:
        binary = (values[tri[2]] > thresh)*4 +  (values[tri[1]] > thresh)*2 +  (values[tri[0]] > thresh)*1
                                    
        # Empty triangle
        if binary == 0:
            continue

        # bottom left point is activated
        elif binary == 1:
            p1 = points[tri[0]].middle_point(points[tri[1]])
            p2 = points[tri[0]].middle_point(points[tri[2]])
        
        # bottom right point is activated
        elif binary == 2:
            p1 = points[tri[1]].middle_point(points[tri[0]])
            p2 = points[tri[1]].middle_point(points[tri[2]])
        
        # both bottom point are activated  
        elif binary == 3:
            p1 = points[tri[2]].middle_point(points[tri[0]])
            p2 = points[tri[2]].middle_point(points[tri[1]])

        # top point is activated
        elif binary == 4:
            p1 = points[tri[2]].middle_point(points[tri[0]])
            p2 = points[tri[2]].middle_point(points[tri[1]])

        # top and bottom left are activated
        elif binary == 5:
            p1 = points[tri[1]].middle_point(points[tri[0]])
            p2 = points[tri[1]].middle_point(points[tri[2]])
        
        # # top and bottom right are activated
        elif binary == 6:
            p1 = points[tri[0]].middle_point(points[tri[1]])
            p2 = points[tri[0]].middle_point(points[tri[2]])
        
        # Filled triangle
        else:
            continue
        
        contours.append((p1, p2))
    return contours

def generate_map(n, m):    
    map = np.random.randint(0, 2, (n+2, m+2))
    map[8:13, 8:13] = 1

    s = 0
    while np.sum(map) != s:
        s = np.sum(map)
        new_map = map.copy()
        for i in range(1, n+1):
            for j in range(1, m+1):
                sum_neighbors = np.sum(map[i-1: i+2,j-1: j+2])
                if sum_neighbors > 4:
                    new_map[i, j] = 1
                else:
                    new_map[i, j] = 0
        map = new_map.copy()
    
    return map[1:n+1, 1:m+1]


def create_trigle_grid(map_size, dx = 1, dy = 1) -> list:

    m = int(map_size[0] / dx)
    n = int(map_size[1] / (dy * np.sin(np.pi/3)))

    values = np.zeros(n*m)
    points = np.empty(n*m, dtype=Vector2)
    triangles = []

    value_map = generate_map(n, m)
    for i in range(n):
        for j in range(m):
            x = ((i%2 == 0)*j + (i%2 == 1)*(j+1/2))*dx
            y = (i*np.sin(np.pi/3))*dy
            points[j + i*m] = Vector2(x, y)
            values[j + i*m] = value_map[i, j]
            
    # Create bottom top triangle
    for i in range(n-1):
        for j in range(m-1):
            triangles.append((j + i*m, j+1 + i*m, j + i%2 + m*(i+1)))
    # Create top bottom triangle
    for i in range(1, n):
        for j in range(m-1):
            triangles.append((j + i*m, j+1 + i*m, j+i%2 + m*(i-1)))

    return triangles, points, values