# Robot Navigation: Uniform Cost Search vs. A* (C++)

A C++ program that finds the shortest path between two vertices of a navigation graph using **Uniform Cost Search (UCS)** and **A\* search**, then compares how many nodes each one reaches.

The program is built around a visibility-graph problem where a robot must travel from a start point to a goal point around a convex polygonal obstacle. The shortest path must follow straight-line segments between the start, the obstacle's vertices, and the goal, so essentially a graph search.

### Data structures

**`nodeType`** is a node in the search tree:

| Field | Meaning |
|-------|---------|
| `state` | index of the vertex |
| `parent` | state of the parent node (`-1` for the start) |
| `costG` | g(n): path cost from the start |
| `costH` | h(n): heuristic estimate to the goal (0 for UCS) |

The `<` operator is overloaded so that `priority_queue` acts as a **min-heap on f = g + h**.

**`navigationGraph`** holds the vertex count, an adjacency matrix `adjM`, and the `xloc`/`yloc` coordinates.

### Edge costs

The input file only says *whether* two vertices are connected. `Expand` computes each edge's cost as the **Euclidean distance** between the two vertices' coordinates, so the weights come from the geometry and are not read from a file.

### Uniform Cost Search

Expands nodes in order of g(n) alone (h = 0), so it always finds the cheapest path but has no sense of direction toward the goal.

### A\* Search

Expands nodes in order of f(n) = g(n) + h(n), where h(n) is the **straight-line distance (SLD)** from the vertex to the goal. 

## Input File Format



## Build and Run

Put the graph files (`P2x2m.txt`, `P2x4s.txt`, ..., `P15x10.txt`) in the same folder, then:

```bash
g++ -std=c++17 -o nav_search [your_file].cpp
./nav_search
```

`main` runs both algorithms on each of 10 graph files and prints one row per file:

| Column | Meaning |
|--------|---------|
| Filename | the graph file |
| Cost | cost of the optimal path |
| UC cost size | number of nodes reached by UCS |
| A\* cost size | number of nodes reached by A\* |
| Solution Path | the UCS path on the first line, the A\* path on the second |

Both algorithms should report the **same cost**. A lower A\* count shows that the heuristic is pruning the search.

## Complexity

With V vertices and E edges:

- **Time:** O(V · V) for `Expand`, which scans a full adjacency-matrix row each time, plus O(E log V) for the priority queue operations
- **Space:** O(V²) for the adjacency matrix, plus O(V) for the queue and the reached map

