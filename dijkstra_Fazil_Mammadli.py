import numpy as np
import cv2
from queue import PriorityQueue as PQ
import math as m
import time as tm
import matplotlib.pyplot as plt

class Point:
    def __init__(self, loc, g_cost, ancestor):
        self.loc = loc
        self.g_cost = g_cost
        self.ancestor = ancestor

def draw_rectangle(arena, top_left, bottom_right, color, border_color, border_thickness=cv2.FILLED):
    cv2.rectangle(arena, top_left, bottom_right, color, cv2.FILLED)
    cv2.rectangle(arena, top_left, bottom_right, border_color, border_thickness)

def draw_hexagon(arena, centre, length, color, border_color, thickness=5, angle_degrees=0):
    angle_radians = np.radians(angle_degrees)
    hex_vertices = []
    for i in range(6):
        theta_deg = 60 * i + 30
        theta_rad = np.radians(theta_deg)
        x = int(centre[0] + length * m.cos(theta_rad + angle_radians))
        y = int(centre[1] + length * m.sin(theta_rad + angle_radians))
        hex_vertices.append((x, y))
    cv2.polylines(arena, [np.array(hex_vertices)], isClosed=True, color=border_color, thickness=thickness)
    cv2.fillPoly(arena, [np.array(hex_vertices)], color)

def create_arena():
    arena = np.ones((500, 1200, 3), dtype=np.uint8) * 255  # Multiplied by 255 for white
    color_blue = (255, 0, 0) 
    color_black = (0, 0, 0)

    # Drawing rectangles with  colors
    draw_rectangle(arena, (100, 0), (175, 400), color_black, color_blue, 5)
    draw_rectangle(arena, (275, 100), (350, 500), color_black, color_blue, 5)
    draw_rectangle(arena, (900, 50), (1020, 125), color_black, color_blue, 5)
    draw_rectangle(arena, (900, 375), (1020, 450), color_black, color_blue, 5)
    draw_rectangle(arena, (1020, 50), (1100, 450), color_black, color_blue, 5)

    # Drawing hexagon with  colors
    draw_hexagon(arena, (650, 250), 150, color_black, color_blue)

    return arena



def is_within_bounds(position):
    return 0 <= position[0] < 1200 and 0 <= position[1] < 500

def is_on_obstacle(arena, position):
    # Check if the position is on a black-colored obstacle
    return np.array_equal(arena[position[1], position[0]], np.array([0, 0, 0]))

def is_on_border(arena, position):
    # Check if the position is on a blue-colored border
    return np.array_equal(arena[position[1], position[0]], np.array([255, 0, 0]))


def validate_positions(arena, start_pos, end_pos):
    valid = True  # Assume positions are valid initially
    
    if not is_within_bounds(start_pos):
        print("Start position is out of bounds")
        valid = False

    if not is_within_bounds(end_pos):
        print("End position is out of bounds")
        valid = False

    if is_on_obstacle(arena, start_pos) or is_on_obstacle(arena, end_pos):
        print("Position is on an obstacle")
        valid = False

    if is_on_border(arena, start_pos):
        print("Start position is on the border")
        valid = False

    if is_on_border(arena, end_pos):
        print("End position is on the border")
        valid = False

    return valid
  
def shift_position(pos, direction, cost):
    """
    Shifts a position in the specified direction.

    Parameters:
    - pos: The current position as a list [x, y].
    - direction: A tuple (dx, dy) indicating the direction to shift.
    - cost: The cost of moving in the specified direction.

    Returns:
    - A new position after the shift and the associated cost.
    """
    new_pos = [pos[0] + direction[0], pos[1] + direction[1]]
    return new_pos, cost

def get_neighbor_points(point, arena):
    """
    Identifies all viable neighboring points from the current point.

    Parameters:
    - point: The current point object.
    - arena: The arena array for checking obstacles and bounds.

    Returns:
    - A list of viable neighboring points and their associated costs.
    """
    point_pos = point.loc
    directions = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                  (-1, 1, 1.4), (1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4)]

    potential_neighbors = [shift_position(point_pos, (dx, dy), cost) for dx, dy, cost in directions]
    viable_neighbors = [neighbor for neighbor in potential_neighbors if check_(neighbor[0], arena)]

    return viable_neighbors

def check_(pos, arena):
    """
    Checks if a position is within bounds, not on an obstacle, and not on a border.

    Parameters:
    - pos: The position to check.
    - arena: The arena array for checking obstacles and bounds.

    Returns:
    - True if the position is viable, False otherwise.
    """
    x, y = pos
    if not (0 <= x < 1200 and 0 <= y < 500):
        return False
    if is_on_obstacle(arena, pos) or is_on_border(arena, pos):
        return False
    return True
  
def initialize_search(arena):
    start_pos, goal_pos = get_valid_positions(arena)
    start_time = tm.time()

    cost_to_come = {str([x, y]): m.inf for x in range(1200) for y in range(500)}
    cost_to_come[str(start_pos)] = 0

    visited = set([str(start_pos)])
    queue = PQ()
    queue.put((0, start_pos))  # PriorityQueue uses first item of tuple for comparison

    point_objects = {str(start_pos): Point(start_pos, 0, None)}
    frames = []

    process_points(arena, queue, cost_to_come, visited, point_objects, frames, goal_pos)
    backtrack_and_visualize(arena, point_objects, goal_pos, frames)
    duration = tm.time() - start_time
    print(f"Algorithm duration: {duration} seconds")
    return frames, start_time, duration

def get_valid_positions(arena):
    valid = False
    while not valid:
        start_x, start_y = [int(value) for value in input("Enter start position coordinates (x y): ").split()]
        goal_x, goal_y = [int(value) for value in input("Enter goal position coordinates (x y): ").split()]
        
        # Flip Y-coordinates
        start_pos = [start_x, 499 - start_y]
        goal_pos = [goal_x, 499 - goal_y]

        valid = validate_positions(arena, start_pos, goal_pos)
    return start_pos, goal_pos


def process_points(arena, queue, cost_to_come, visited, point_objects, frames, goal_pos):
    count = 0
    while not queue.empty():
        _, current_pos = queue.get()
        current_point = point_objects[str(current_pos)]

        if current_pos == goal_pos:
            print("Goal reached.")
            break

        for neighbor, cost in get_neighbor_points(current_point, arena):
            new_cost = cost + current_point.g_cost
            neighbor_str = str(neighbor)

            if neighbor_str not in visited or new_cost < cost_to_come[neighbor_str]:
                visited.add(neighbor_str)
                arena[neighbor[1], neighbor[0]] = np.array([0, 255, 0])  # Visualize this point

                if count % 1000 == 0:
                    frames.append(arena.copy())

                cost_to_come[neighbor_str] = new_cost
                point_objects[neighbor_str] = Point(neighbor, new_cost, current_point)
                queue.put((new_cost, neighbor))

                count += 1
                
def backtrack_and_visualize(arena, point_objects, goal_pos, frames):
    if str(goal_pos) not in point_objects:
        print("Goal position was not reached.")
        return

    backtracked_point = point_objects[str(goal_pos)]
    ancestor_point = backtracked_point.ancestor
    backtrack_path = []

    # Backtracking logic to directly use stored positions
    while ancestor_point:
        backtrack_path.append(ancestor_point.loc)
        ancestor_point = ancestor_point.ancestor

    backtrack_path.reverse()  # Ensure the path is from start to goal

    # Visualization logic to use numpy indexing directly
    for position in backtrack_path:
        arena[position[1], position[0]] = np.array([0, 0, 255])  # Mark the path in red
        frames.append(arena.copy())

    visualize_path(frames)

def visualize_path(frames):
    video_clip = cv2.VideoWriter("Pathfinding_Visualization.mp4", cv2.VideoWriter_fourcc(*'MP4V'), 60, (1200, 500))
    for frame in frames:
        video_clip.write(frame)
    video_clip.release()
    print("Visualization video saved as 'Pathfinding_Visualization.mp4'.")




if __name__ == "__main__":
    arena = create_arena()  # Create the arena with obstacles
    frames,start_time,duration= initialize_search(arena)  # Start the search process and get frames for visualization

    # Plotting
    final_frame_rgb = cv2.cvtColor(frames[-1], cv2.COLOR_BGR2RGB)
    plt.figure(figsize=(12, 5))
    plt.imshow(final_frame_rgb)
    plt.axis('off')  # Turn off axis numbers and ticks
    plt.title("Final Path Visualization")
    plt.show()
