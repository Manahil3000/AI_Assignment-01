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

# Updated Heuristic: Octile Distance (Best for 8-way movement)
def heuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    # Straight moves cost 1, diagonal moves cost sqrt(2)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

# A* algorithm
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    closed_set = set()
    
    heapq.heappush(open_list, Node(start, None, 0, heuristic(start, goal)))
    
    while open_list:
        current = heapq.heappop(open_list)
        
        if current.position == goal:
            return reconstruct_path(current)
            
        closed_set.add(current.position)

        # 8-way movement: (row_change, col_change, cost)
        moves = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),       # Cardinal
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414) # Diagonal
        ]

        for dr, dc, cost in moves:
            r = current.position[0] + dr
            c = current.position[1] + dc
            next_pos = (r, c)

            # Boundary checks
            if r < 0 or r >= rows or c < 0 or c >= cols:
                continue
            
            # Obstacle/Fire check
            if grid[r][c] in ['#', 'F']:
                continue
                
            if next_pos in closed_set:
                continue

            # Blocks diagonal movement if passing between two touching obstacles
            if dr != 0 and dc != 0:
                if grid[current.position[0] + dr][current.position[1]] in ['#', 'F'] and \
                   grid[current.position[0]][current.position[1] + dc] in ['#', 'F']:
                    continue

            g = current.g + cost
            h = heuristic(next_pos, goal)
            heapq.heappush(open_list, Node(next_pos, current, g, h))

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
    print(f"Initial Planning from {current_position}...")
    
    current_path = astar(grid, current_position, goal)
    
    if not current_path:
        print("No initial path found...")
        return None

    final_executed_path = [current_position]
    
    i = 1
    while i < len(current_path):
        next_step = current_path[i]
        
        if next_step in new_fire_cells:
            print(f"!!! Fire detected at {next_step}. STOPPING at {current_position} to Replan...")
            
            r, c = next_step
            grid[r][c] = 'F'
            new_fire_cells.remove(next_step)
            
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