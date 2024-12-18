import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Grid cell dimensions
lx, ly = 2.1, 1.3  # Modify as needed

# Updated functions for grid scaling
def coord_to_ids(coord):
    """Convert coordinate to integer grid indices based on cell size."""
    return int(coord[0] // lx), int(coord[1] // ly)

def coord_to_ids_float(coord):
    """Convert coordinate to float grid indices based on cell size."""
    return coord[0] / lx, coord[1] / ly

def compute_line_tuple(p1, p2):
    """Compute line coefficients (a, b, c) for the equation ax + by + c = 0."""
    x1, y1 = p1
    x2, y2 = p2
    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2
    return a, b, c

# Function under test
def move_on_line(p_r):
    ids1 = coord_to_ids(center)
    ids2 = coord_to_ids(p_r)
    ids1_float = coord_to_ids_float(center)
    ids2_float = coord_to_ids_float(p_r)

    idx, idy = ids1
    idx_float, idy_float = ids1_float
    
    a, b, _ = compute_line_tuple(ids1_float, ids2_float)
    if b:
        m = -a / b
    
    ids = []

    if ids1[0] == ids2[0]:  # Vertical line
        direction = 1 * (ids1[1] < ids2[1]) - 1 * (ids1[1] >= ids2[1])
        for idy in range(ids1[1], ids2[1] + direction, direction):
            ids.append((idx, idy))
        return ids

    if ids1[1] == ids2[1]:  # Horizontal line
        direction = 1 * (ids1[0] < ids2[0]) - 1 * (ids1[0] >= ids2[0])
        for idx in range(ids1[0], ids2[0] + direction, direction):
            ids.append((idx, idy))
        return ids

    direction = np.sign(ids1[0] - ids2[0])

    n = 0
    if abs(m) < 1:
        while n <= abs(ids1[0] - ids2[0]):
            ids.append((idx, idy + 1))
            ids.append((idx, idy - 1))
            ids.append((idx, idy))
            
            idx_float -= 1 * direction
            idy_float -= m * direction

            idx = int(idx_float)
            idy = int(idy_float)
            n += 1
    else:
        while n <= abs(ids1[1] - ids2[1]):
            ids.append((idx + 1, idy))
            ids.append((idx - 1, idy))
            ids.append((idx, idy))

            idx_float -= abs(1 / m) * direction
            idy_float -= np.sign(m) * direction

            idx = int(idx_float)
            idy = int(idy_float)

            n += 1

    return ids

# Animation function
def animate_circle():
    global center
    grid_size = 40
    center = (20.5, 20.5)  # Fixed center of the circle
    radius = 16         # Radius of the circle
    num_frames = 360   # Number of animation frames

    # Initialize the figure
    fig, ax = plt.subplots()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_aspect('equal')

    # Draw the grid
    for i in range(0, int((grid_size + 1 )/ lx)):
        ax.axvline(i * lx, color='gray', linestyle='--', linewidth=0.5)
    for i in range(0, int((grid_size + 1 )/ ly)):
        ax.axhline(i * ly, color='gray', linestyle='--', linewidth=0.5)

    # Line and rectangles
    line, = ax.plot([], [], color='orange', lw=2)
    rectangles = []

    # Update function for the animation
    def update(frame):
        nonlocal rectangles
        angle = np.radians(frame)  # Convert frame to angle in radians
        p_r = (center[0] + radius * np.cos(angle), center[1] + radius * np.sin(angle))  # Point on circle

        # Clear previous rectangles
        for rect in rectangles:
            rect.remove()
        rectangles = []

        # Get subdivisions hit by the line
        traversed_ids = move_on_line(p_r)

        # Draw the line
        line.set_data([center[0], p_r[0]], [center[1], p_r[1]])

        # Draw the subdivisions
        for idx, idy in traversed_ids:
            rect_x, rect_y = idx * lx, idy * ly
            rect = plt.Rectangle((rect_x, rect_y), lx, ly, color='cyan', alpha=0.4)
            ax.add_patch(rect)
            rectangles.append(rect)

        return line, *rectangles

    # Create the animation
    ani = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

    # Show the animation
    plt.title("Rotating Line Traversing Subdivisions (Grid with Cell Size lx x ly)")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()

# Run the animation
animate_circle()
