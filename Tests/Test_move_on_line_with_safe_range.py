import numpy as np
import matplotlib.pyplot as plt

# Mock dependencies
def compute_line_tuple(p1, p2):
    return (p2[0] - p1[0], p2[1] - p1[1])

def compute_ortho_vec(line):
    dx, dy = line
    norm = np.sqrt(dx**2 + dy**2)
    return (-dy / norm, dx / norm)

def distance_tuple(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# Mock grid class
class Grid:
    def __init__(self, grid_size, block_size):
        self.grid_size = grid_size
        self.block_size = block_size

    def coord_to_ids(self, coord):
        """Convert world coordinates to grid indices."""
        return (
            int(coord[0] // self.block_size[0]),
            int(coord[1] // self.block_size[1]),
        )

    def ids_to_center(self, idx, idy):
        """Convert grid indices to the center coordinates of the cell."""
        return (
            idx * self.block_size[0] + self.block_size[0] / 2,
            idy * self.block_size[1] + self.block_size[1] / 2,
        )

    def points_in_safe_range_to_ids(self, p1, p2, safe_range, ax):
        """
        Get all list indices inside a rotated rectangle on a grid.
        """
        rotation = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        print(rotation)
        line = compute_line_tuple(p1, p2)

        # Define perpendicular vector for width
        ortho_vec = compute_ortho_vec(line)
        length = distance_tuple(p1, p2)

        p1, p2 = np.array(p1), np.array(p2)
        half_width = np.array([safe_range * ortho_vec[0], safe_range * ortho_vec[1]])
        corners = np.array([
            p1 + half_width,
            p2 + half_width,
            p1 - half_width,
            p2 - half_width,
        ])
        for corner in corners:
            ax.scatter(corner[0], corner[1], 100)
        rotation_matrix = np.array([
            [np.cos(rotation), -np.sin(rotation)],
            [np.sin(rotation), np.cos(rotation)],
        ])
        transposed_rot_matrix = rotation_matrix.T

        # rotated_corners = [np.dot(rotation_matrix, corner - p1) + p1 for corner in corners]
        ids = np.array([self.coord_to_ids(corner) for corner in corners])
        x_max = np.max(ids[:, 0])
        x_min = np.min(ids[:, 0])
        y_max = np.max(ids[:, 1])
        y_min = np.min(ids[:, 1])
        # for corner in rotated_corners:
        #     ax.scatter(corner[0], corner[1], 100)

        inside_points = []
        for idx in range(x_min, x_max + 1):
            for idy in range(y_min, y_max + 1):
                point = np.array(self.ids_to_center(idx, idy))
                local_point = np.dot(transposed_rot_matrix, point - p1)
                if (
                    -safe_range <= local_point[0] <= length + safe_range and
                    -safe_range <= local_point[1] <= safe_range
                ):
                    inside_points.append((idx, idy))

        return inside_points

# Test script
def test():
    grid_size = (20, 20)
    block_size = (1, 1)
    grid = Grid(grid_size, block_size)

    p1 = (1, 5)
    p2 = (15, 10)
    safe_range = 2.0

    fig, ax = plt.subplots(figsize=(8, 8), layout="constrained")
    inside_points = grid.points_in_safe_range_to_ids(p1, p2, safe_range, ax)

    # Visualization
    # ax.set_xlim(0, grid_size[0])
    # ax.set_ylim(0, grid_size[1])
    ax.set_aspect('equal')
    ax.set_xticks(np.arange(0, grid_size[0], 1))
    ax.set_yticks(np.arange(0, grid_size[1], 1))
    ax.grid(True, which='both', color='lightgray', linestyle='--', linewidth=0.5)

    # Plot start and end points
    ax.plot(p1[0], p1[1], 'go', label='Start Point')
    ax.plot(p2[0], p2[1], 'ro', label='End Point')

    # Plot points inside rectangle
    for idx, idy in inside_points:
        center = grid.ids_to_center(idx, idy)
        ax.plot(center[0], center[1], 'bo')

    plt.title("Safe Range Points to IDs")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    test()
