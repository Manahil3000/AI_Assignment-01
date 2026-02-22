import heapq
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

# Heuristic Distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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

        for move in [(-1,0), (1,0), (0,-1), (0,1)]:
            r = current.position[0] + move[0]
            c = current.position[1] + move[1]
            next_pos = (r, c)

            if r < 0 or r >= rows or c < 0 or c >= cols:
                continue
            if grid[r][c] in ['#', 'F']:
                continue
            if next_pos in closed_set:
                continue

            g = current.g + 1
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
    print("Initial Planning...")
    path = astar(grid, current_position, goal)
    if not path:
        print("No initial path found...")
        return None
    i = 1
    while i < len(path):
        step = path[i]
        if step in new_fire_cells:
            print(f"Fire appeared at {step}... Replanning...")
            r, c = step
            grid[r][c] = 'F'
            new_fire_cells.remove(step)  
            path = astar(grid, current_position, goal)
            if not path:
                print("No safe path after replanning...")
                return None
            i = 1 
            continue
        current_position = step
        i += 1
    return path