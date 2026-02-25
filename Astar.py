import heapq
import math

# Node class for A* algorithm
class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h
        
    def __lt__(self, other):
        return self.f < other.f

# Heuristic: Manhattan Distance 
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* algorithm
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    # We store the best g-cost for each position to avoid redundant processing
    best_g = {start: 0}
    
    heapq.heappush(open_list, Node(start, None, 0, heuristic(start, goal)))
    
    while open_list:
        current = heapq.heappop(open_list)
        
        if current.position == goal:
            return reconstruct_path(current)

        # 4-way movement: Up, Down, Left, Right (all cost 1)
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dr, dc in moves:
            r = current.position[0] + dr
            c = current.position[1] + dc
            next_pos = (r, c)

            # Boundary checks
            if 0 <= r < rows and 0 <= c < cols:
                # Obstacle/Fire check 
                if grid[r][c] in ['#', 'F']:
                    continue

                new_g = current.g + 1
                
                # If we found a shorter path to this neighbor, or haven't visited it:
                if next_pos not in best_g or new_g < best_g[next_pos]:
                    best_g[next_pos] = new_g
                    h = heuristic(next_pos, goal)
                    heapq.heappush(open_list, Node(next_pos, current, new_g, h))

    return None

# Path Reconstruction
def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]

# Dynamic Replanning Function
def dynamic_astar(grid, start, goal, new_fire_cells):
    current_position = start
    # Copy fire cells to avoid modifying the input list outside
    fires = list(new_fire_cells)
    
    print(f"Initial Planning from {current_position}...")
    current_path = astar(grid, current_position, goal)
    
    if not current_path:
        print("No initial path found...")
        return None

    final_executed_path = [current_position]
    
    i = 1
    while i < len(current_path):
        next_step = current_path[i]
        
        if next_step in fires:
            print(f"!!! Fire detected at {next_step}. STOPPING at {current_position} to Replan...")
            
            r, c = next_step
            grid[r][c] = 'F'
            fires.remove(next_step)
            
            new_plan = astar(grid, current_position, goal)
            
            if not new_plan:
                print("BLOCK: No safe path remains!")
                return final_executed_path 
            
            current_path = new_plan
            i = 1 
            print(f"New path calculated: {current_path}")
            continue             
            
        current_position = next_step
        final_executed_path.append(current_position)
        i += 1
        
    return final_executed_path