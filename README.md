# Smart Emergency Evacuation Planner

> Dynamic A\* Search · Manhattan Heuristic · Fire Discovery Simulation ·
> Interactive Grid Visualization

A grid-based pathfinding system that simulates emergency evacuation
using the **A\*** search algorithm.\
The project supports both **static pathfinding** and **dynamic
replanning** when hidden fire hazards are discovered during traversal.

This project demonstrates informed search, heuristic design, and
real-time replanning in uncertain environments.

------------------------------------------------------------------------

## Features

-    Optimal pathfinding using **A**\* search
-    Dynamic fire discovery with real-time replanning
-    Manhattan heuristic (admissible & optimal for grid movement)
-    Step counter & replan counter
-    Interactive GUI built with Streamlit
-    Built-in test cases for validation
-    No-path detection handling
-    Modular architecture (algorithm separated from UI)

------------------------------------------------------------------------

## Project Structure

    ├── Astar.py          # Core A* and Dynamic A* implementations
    ├── GUI.py            # Streamlit-based interactive interface
    ├── testCase.py       # Standalone algorithm testing (CLI mode)
    └── requirements.txt  # Python dependencies

------------------------------------------------------------------------

## Requirements

  Package     Purpose
  ----------- ---------------------------------
  streamlit   Web-based interactive interface
  heapq       Priority queue for A\* (stdlib)
  copy        Grid state cloning (stdlib)

Install dependencies:

``` bash
pip install -r requirements.txt
```

------------------------------------------------------------------------

## How to Run

### ▶ Run GUI Version

``` bash
streamlit run GUI.py
```

The app opens automatically at:

http://localhost:8501

------------------------------------------------------------------------

### ▶ Run Test Cases (No GUI)

``` bash
python testCase.py
```

------------------------------------------------------------------------

## GUI Usage Guide

### 1️. Design the Grid

Use the sidebar panel to paint cells:

  Symbol   Meaning
  -------- ---------
  S        Start
  E        Goal
  \#       Wall
  F        Fire

------------------------------------------------------------------------

### 2️. Select Mode

-   **A\* (Static)**\
    Fire cells act as obstacles from the beginning.

-   **Dynamic A**\*\
    Fire cells are hidden initially and discovered only when the agent
    reaches them.

------------------------------------------------------------------------

### 3. Run

Click **RUN PATHFINDER** to compute the evacuation route.

------------------------------------------------------------------------

## Algorithm Overview

### A\* Search

A\* selects nodes using:

f(n) = g(n) + h(n)

  Term   Description
  ------ -----------------------------------
  g(n)   Cost from start to node n
  h(n)   Heuristic estimate from n to goal
  f(n)   Total estimated cost

------------------------------------------------------------------------

### Heuristic: Manhattan Distance

h(a, b) = \|a.row - b.row\| + \|a.col - b.col\|

✔ Admissible\
✔ Consistent\
✔ Guarantees optimal path in 4-direction grids

------------------------------------------------------------------------

### Dynamic A\* (Replanning Strategy)

Dynamic mode simulates incomplete information.

Execution logic:

1.  Compute initial path ignoring fire.
2.  Move step-by-step.
3.  If next step is fire:
    -   Mark it permanently as blocked.
    -   Increment replan counter.
    -   Re-run A\* from current position.
4.  Continue until goal reached or no path exists.
5.  Accumulate all executed steps into a full journey.

The final output always preserves the complete path from the original
start.

------------------------------------------------------------------------

## Included Test Scenarios

  Scenario                Mode      Description
  ----------------------- --------- --------------------------
  Basic Walls             Static    Standard shortest path
  Static Fire             Static    Fire treated as obstacle
  Single Hidden Fire      Dynamic   One replan triggered
  Multiple Hidden Fires   Dynamic   Multiple replans
  Fully Blocked Grid      Static    No path possible

------------------------------------------------------------------------

## Complexity Analysis

  Metric         Complexity
  -------------- ------------------------------
  Time (A\*)     O(E log V)
  Space (A\*)    O(V)
  Dynamic Mode   O(k · A\*) where k = replans

Where: - V = number of grid cells\
- E = edges between cells

------------------------------------------------------------------------

## Grid Representation

  Symbol   Meaning   Behavior
  -------- --------- ----------------------------------------------------
  .        Empty     Traversable
  S        Start     Agent origin
  E        Goal      Destination
  \#       Wall      Always blocked
  F        Fire      Static: blocked · Dynamic: hidden until discovered

------------------------------------------------------------------------

## Learning Outcomes

This project demonstrates:

-   Informed search strategies
-   Heuristic admissibility & optimality
-   Real-time replanning in dynamic environments
-   Simulation modeling
-   Interactive algorithm visualization
-   Clean modular software design

------------------------------------------------------------------------

## Future Improvements

-   Implement D\* Lite for incremental search
-   Add diagonal movement
-   Animate traversal step-by-step
-   Simulate spreading fire over time
-   Add weighted terrain costs

------------------------------------------------------------------------

## Notes

-   Coordinates are (row, col) with (0,0) at the top-left.
-   Grid sizes supported: 2×2 up to 12×12.
-   Dynamic mode temporarily hides fire cells before execution.
-   All replanning events are counted and reported.

------------------------------------------------------------------------

## License

This project is for educational purposes.
