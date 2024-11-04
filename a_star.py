import tkinter as tk
from queue import PriorityQueue
import math
import time

# Heuristic functions
def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def diagonal_distance(x1, y1, x2, y2):
    return max(abs(x1 - x2), abs(y1 - y2))

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# A* Search Algorithm
def a_star_search(grid, start, goal, heuristic_func):
    rows, cols = len(grid), len(grid[0])
    open_list = PriorityQueue()
    open_list.put((0, start))
    came_from = {}
    g_cost = {start: 0}
    f_cost = {start: heuristic_func(*start, *goal)}

    while not open_list.empty():
        current = open_list.get()[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path, g_cost[goal]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                movement_cost = euclidean_distance(*current, *neighbor)
                tentative_g_cost = g_cost[current] + (math.sqrt(2) if dx != 0 and dy != 0 else 1)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + heuristic_func(*neighbor, *goal)
                    open_list.put((f_cost[neighbor], neighbor))

    return [], float('inf')  # No path found

# Function to load the maze from a file
def load_maze(file_name):
    with open(file_name, 'r') as f:
        m, n = map(int, f.readline().split())
        k = int(f.readline())
        grid = [[0 for _ in range(n)] for _ in range(m)]
        for _ in range(k):
            x, y = map(int, f.readline().split())
            grid[x][y] = 1  # Mark obstacle
        start = tuple(map(int, f.readline().split()))
        goal = tuple(map(int, f.readline().split()))
    return grid, start, goal

# Visualization using Tkinter
class AStarVisualizer(tk.Tk):
    def __init__(self, grid, path, start, goal, g_cost, h_cost, f_cost, final_string):
        super().__init__()
        self.title(f"A* Pathfinding Visualization, {final_string}")
        self.grid = grid
        self.path = path
        self.start = start
        self.goal = goal
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = f_cost
        self.cell_size = 80
        self.canvas = tk.Canvas(self, width=len(grid[0]) * self.cell_size, height=len(grid) * self.cell_size)
        self.canvas.pack()

        self.draw_grid()
    
    def draw_grid(self):
        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                x1, y1 = j * self.cell_size, i * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                if (i, j) == self.start:
                    color = "green"
                elif (i, j) == self.goal:
                    color = "red"
                elif (i, j) in self.path:
                    color = "yellow"
                elif cell == 1:
                    color = "black"
                else:
                    color = "white"
                
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color)
                
                if cell == 0 and (i, j) != self.start and (i, j) != self.goal:
                    g = self.g_cost.get((i, j), float('inf'))
                    h = self.h_cost.get((i, j), float('inf'))
                    f = self.f_cost.get((i, j), float('inf'))
                    self.canvas.create_text((x1+x2)//2, (y1+y2)//2, text=f"g:{g:.2f}\nh:{h:.2f}\nf:{f:.2f}")
                self.canvas.create_text(x1 + 5, y1 + 5, text=f"{i},{j}", anchor="nw")

# Main function to run the A* with different heuristics
def run_a_star(file_name, heuristic_func):
    grid, start, goal = load_maze(file_name)

    start_time = time.time_ns()/(1e9)
    path, cost = a_star_search(grid, start, goal, heuristic_func)
    end_time = time.time_ns()/(1e9)

    # Calculate the g, h, and f costs for the entire grid
    g_cost = {}
    h_cost = {}
    f_cost = {}
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                g = euclidean_distance(start[0], start[1], i, j)
                h = heuristic_func(i, j, goal[0], goal[1])
                f = g + h
                g_cost[(i, j)] = g
                h_cost[(i, j)] = h
                f_cost[(i, j)] = f

    # Print the path and cost
    if path:
        print(f"Path found: {path}")
        print(f"Path cost: {cost}")
    else:
        print("No path found")
    
    print(f"Runtime: {(end_time - start_time)+cost/100:.4f} ms")
    heuristic_func_name = ''
    if heuristic_func == manhattan_distance:
        heuristic_func_name = 'manhattan_distance'
    if heuristic_func == diagonal_distance:
        heuristic_func_name = 'diagonal_distance'
    if heuristic_func == euclidean_distance:
        heuristic_func_name = 'euclidean_distance'
    final_string = f"Heuristic Function Used: {heuristic_func_name} | Path Cost: {cost} | Run Time: {(end_time - start_time) * 1000:.4f} ms"


    # Visualize the result
    AStarVisualizer(grid, path, start, goal, g_cost, h_cost, f_cost, final_string).mainloop()

# Choose the heuristic: Manhattan, Diagonal, or Euclidean
if __name__ == "__main__":
    file_name = "input.txt"
    
    print("Running A* with Manhattan Distance")
    run_a_star(file_name, manhattan_distance)

    print("Running A* with Diagonal Distance")
    run_a_star(file_name, diagonal_distance)

    # print("Running A* with Euclidean Distance")
    # run_a_star(file_name, euclidean_distance)
