# ENPM661-Project-2
# Dijkstra Algorithm Implementation for Point Robot

## Overview
This project implements Dijkstra's Algorithm to find the shortest path for a point robot within a specified arena with obstacles. The algorithm explores nodes in the search space, visualizing both the exploration process and the final optimal path.

## How to Run the Code
1. Ensure you have Python installed on your machine. This code was tested with Python 3.8.
2. Install the required libraries if you haven't done so:
   - `numpy`
   - `opencv-python`
   - `matplotlib`
3. Run the script using the command: `python dijkstra_Fazil_Mammadli.py`
4. The program will prompt you to enter start and goal coordinates in the format `x y`. Input these coordinates considering the origin is at the bottom-left corner of the arena.
   - Example input for start position: `50 450`
   - Example input for goal position: `1100 50`
5. The program will visualize the node exploration and optimal path finding. Once the computation is complete, a video file named `Pathfinding_Visualization.mp4` will be saved in the current directory.

## Libraries/Dependencies Used
- `numpy`: Used for array manipulation and mathematical operations.
- `opencv-python` (cv2): Utilized for drawing the arena, obstacles, and for creating the pathfinding visualization.
- `matplotlib`: Employed for displaying the final path in a coordinate frame.

## Input Coordinates System
The coordinate system used in this project considers the origin `(0, 0)` to be at the **bottom-left corner** of the arena. The X-axis extends towards the right, and the Y-axis extends upwards. The maximum valid coordinates are `(1199, 499)`.

## Important Notes
- The arena size is fixed at 1200x500 pixels.
- Obstacles and borders are predefined and visualized in the generated video.
  
## Output 
- Start and goal positions must not be inside obstacles or outside the arena bounds.
- The program automatically accounts for a robot clearance of 5 mm when determining the feasibility of the path.

## Visualization and Output
The exploration of nodes and the optimal path are visualized in a video file named `Pathfinding_Visualization.mp4`. The exploration process is shown in green, while the optimal path is marked in red.

## Animation Video
In provided video (`Pathfinding_Visualization.mp4`) you can see that the user entered origin (0,0) as start point and (1111,333) as goal point.
