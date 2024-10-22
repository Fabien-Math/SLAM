import numpy as np
import numpy.random as nprd
import matplotlib.pyplot as plt

class Point:
    def __init__(self, x:float, y:float, value:float) -> None:
        self.x = x
        self.y = y
        self.value = value
    
    # Compute the point at the mid distance between p and the self point
    def mid_distance(self, p):
        return Point((p.x + self.x) / 2, (p.y + self.y) / 2, 1)
    
    # Compute the point at the interpolation distance between p and the self point
    def inter_distance(self, p):
        return Point(p.x + (self.x - p.x) * (p.value + self.value)/2, p.y + (self.y - p.y) * (p.value + self.value)/2, 1)
    
    def show_point(self):
        plt.scatter(self.x, self.y, s=nprd.randint(5,15))

    def to_tuple(self):
        return (self.x, self.y)
        

class Triangle:
    def __init__(self, p1:Point, p2:Point, p3:Point) -> None:
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

class Grid:    
    def __init__(self, triangles:Triangle = [], points:Point = [], thresh:float = 0.5) -> None:
        self.tri = triangles
        self.points = points
        self.contours = None
        self.thresh = thresh
        
    def compute_contours(self):
    # List of list two points defining lines
        self.contours = []
        for tri in self.tri:
            binary = (self.points[tri[2]].value > self.thresh)*4 +  (self.points[tri[1]].value > self.thresh)*2 +  (self.points[tri[0]].value > self.thresh)*1
                                        
            # Empty triangle
            if binary == 0:
                continue

            # bottom left point is activated
            elif binary == 1:
                p1 = self.points[tri[0]].mid_distance(self.points[tri[1]])
                p2 = self.points[tri[0]].mid_distance(self.points[tri[2]])
            
            # bottom right point is activated
            elif binary == 2:
                p1 = self.points[tri[1]].mid_distance(self.points[tri[0]])
                p2 = self.points[tri[1]].mid_distance(self.points[tri[2]])
            
            # both bottom point are activated  
            elif binary == 3:
                p1 = self.points[tri[2]].mid_distance(self.points[tri[0]])
                p2 = self.points[tri[2]].mid_distance(self.points[tri[1]])

            # top point is activated
            elif binary == 4:
                p1 = self.points[tri[2]].mid_distance(self.points[tri[0]])
                p2 = self.points[tri[2]].mid_distance(self.points[tri[1]])

            # top and bottom left are activated
            elif binary == 5:
                p1 = self.points[tri[1]].mid_distance(self.points[tri[0]])
                p2 = self.points[tri[1]].mid_distance(self.points[tri[2]])
            
            # # top and bottom right are activated
            elif binary == 6:
                p1 = self.points[tri[0]].mid_distance(self.points[tri[1]])
                p2 = self.points[tri[0]].mid_distance(self.points[tri[2]])
            
            # Filled triangle
            else:
                continue
            
            self.contours.append((p1, p2))
        
    def print_points(self):
        for p in self.points:
            print(f"x: {p.x}, y: {p.y}")
    
    def draw_triangles(self):
        for tri in self.tri:
            x = []
            y = []
            for i in range(3):
                x.append(self.points[tri[i]].x)
                y.append(self.points[tri[i]].y)
            plt.scatter(x, y)
        plt.axis("equal")
    
    def draw_contours(self):
        self.compute_contours()
        plt.figure("Thresholded values scatter plot")
        #plt.figure("Contours")
        print(len(self.contours))
        
        for i, line in enumerate(self.contours):
            plt.plot([line[0].x, line[1].x], [line[0].y, line[1].y])
            mid_point = line[0].mid_distance(line[1])
            plt.annotate(str(i), (mid_point.to_tuple()))
        plt.axis("equal")
        
    def draw_values(self):
        x = []
        y = []
        v = []
        for p in self.points:
            x.append(p.x)
            y.append(p.y)
            v.append(p.value)
        plt.figure("Values scatter plot")
        plt.scatter(x, y, c = 1 - np.array(v), cmap='gray')
        plt.axis("equal")
        
    def draw_values_thresh(self):
        thresh = grid.thresh
        plt.figure("Thresholded values scatter plot")
        x = []
        y = []
        v_thresh = []
        for p in grid.points:
            x.append(p.x)
            y.append(p.y)
            v_thresh.append((p.value > thresh) * p.value)
        plt.scatter(x, y, c = 1 - np.array(v_thresh), cmap='gray')
        plt.axis("equal")
    
    
def threshold(grid: Grid):
    values = [p.value for p in grid.points]
    return np.where(np.array(values) > grid.thresh, np.array(values), 0)

def generate_map(n, m):    
    map = np.random.randint(0, 2, (n, m))
    map[8:13, 8:13] = 1

    s = 0
    while np.sum(map) != s:
        s = np.sum(map)
        new_map = map.copy()
        for i in range(n):
            for j in range(m):
                sum_neighbors = np.sum(map[i-1: i+2,j-1: j+2])
                if sum_neighbors > 4:
                    new_map[i, j] = 1
                else:
                    new_map[i, j] = 0
        map = new_map.copy()
    
    return map


def create_trigle_grid(n, m = None, dx = 1, dy = 1, thresh = 0.5) -> Grid:
    if m is None: m = n
    
    grid = Grid()
    grid.thresh = thresh
    
    # perlin = generate_perlin_noise(n, 8)
    # perlin += np.min(perlin) * ((np.min(perlin) < 0) * -1 + (np.min(perlin) > 0) * 1)
    # perlin /= np.max(perlin)
    value_map = generate_map(n, m)
    for i in range(n):
        for j in range(m):
            x = ((i%2 == 0)*j + (i%2 == 1)*(j+1/2))*dx
            y = (i*np.sin(np.pi/3))*dy
            value = value_map[i, j]

            grid.points.append(Point(x, y, value))
            
    # Create bottom top triangle
    for i in range(n-1):
        for j in range(m-1):
            grid.tri.append((j + i*m, j+1 + i*m, j + i%2 + m*(i+1)))
    # Create top bottom triangle
    for i in range(1, n):
        for j in range(m-1):
            grid.tri.append((j + i*m, j+1 + i*m, j+i%2 + m*(i-1)))
    return grid



n = 30

grid = create_trigle_grid(n, dx = 0.1, dy = 0.1, thresh=0.4)
# grid.print_points()
# grid.draw_triangles()
grid.draw_values()
grid.draw_values_thresh()
grid.draw_contours()


plt.show()
# if input("Save image ? ") == '1':
if True:
    # img_name = input("Image name ?")
    img_name = '1'
    v = []
    thresh = []
    for p in grid.points:
        v.append((p.value, p.value, p.value, 1.0))
        if p.value < 0.5:
            thresh.append((0.0, 0.0, 0.0, 0.0))
        else:
            thresh.append(v[-1])
    v = np.reshape(v, (n,n,4))
    thresh = np.reshape(thresh, (n,n,4))
    # plt.imshow(thresh, cmap="gray")
    plt.imsave(img_name + ".png", v, cmap="gray")
    plt.imsave(img_name + "_tresh.png", thresh, cmap="gray")