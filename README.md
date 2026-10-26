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

### Uniform Cost Search

Expands nodes in order of g(n) alone (h = 0), so it always finds the cheapest path but has no sense of direction toward the goal.

### A\* Search

Expands nodes in order of f(n) = g(n) + h(n), where h(n) is the **straight-line distance (SLD)** from the vertex to the goal. 

## Input File Format

Each graph file is plain text, separated by tabs or spaces, in three blocks:

```
m  n                 # number of vertices, number of edges
id  x  y             # m lines: one per vertex
...
a  b                 # n lines: one undirected edge per line
...
```

This is an example of how the file `P2x2m.txt` is transformed into the format:

| Block | Lines | Content |
|-------|-------|---------|
| Header | 1 | `18 73`: 18 vertices and 73 edges |
| Vertices | 18 | an id followed by x and y coordinates, such as `0  20.0  20.0` |
| Edges | 73 | a pair of vertex ids, such as `0  1` |

**Vertices.** Vertices `0` to `17` are the corners of the polygon obstacles plus the start and goal points. Examples from the file:

| id | x | y | Role |
|----|-----|-----|------|
| 0 | 20.0 | 20.0 | **start** |
| 1 | 5.17 | 49.79 | obstacle vertex |
| 2 | 36.17 | 37.79 | obstacle vertex |
| ... | ... | ... | ... |
| 17 | 100.0 | 200.0 | **goal** |

**Edges.** Each line lists two vertices that can see each other, meaning the straight line between them does not pass through an obstacle. For example, `0 1`, `0 2`, and `0 10` mean the start can reach vertices 1, 2, and 10 directly. Edges are undirected, so `readGraphFromFile` sets both `adjM[a][b]` and `adjM[b][a]`.

**Edge costs are not in the file.** The file only says which vertices are connected. `Expand` computes each cost as the Euclidean distance between the two vertices' coordinates.


### Output
This is an example of the output for the File P2x2m.txt.

```
  Filename      Cost        UC cost size        A* cost size     Solution Path (UC/A*)
 P2x2m.txt     203.2                  18                  17        0   2  11  17
                                                                    0   2  11  17
```

| Column | Meaning |
|--------|---------|
| Filename | the graph file that was searched |
| Cost | total length of the shortest path, in the file's coordinate units |
| UC cost size | number of nodes UCS reached (generated) |
| A\* cost size | number of nodes A\* reached (generated) |
| Solution Path | vertex ids from start to goal (UCS on the first line, A\* on the second) |

- **Both algorithms return the same path and cost.** This is expected, because UCS is optimal and the straight-line-distance heuristic keeps A\* optimal too.
- **A\* reaches fewer nodes** (17 versus 18). The savings are small on this graph because the goal is far from the start and most of the 18 vertices lie on the way. The gap grows on the larger files (`P10x10.txt`, `P15x10.txt`), where the heuristic prunes much more.

## Build and Run

```
g++ -std=c++17 -o nav_search your_file.cpp
./nav_search (for mac)
.\nav_search.exe (for windows)
```

`main` runs both algorithms on each of 10 graph files and prints one row per file:

## Complexity

With V vertices and E edges:

- **Time:** O(V · V) for `Expand`, which scans a full adjacency-matrix row each time, plus O(E log V) for the priority queue operations
- **Space:** O(V²) for the adjacency matrix, plus O(V) for the queue and the reached map

