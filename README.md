# Robot Navigation: Uniform Cost Search vs. A* (C++)

A C++ program that finds the shortest path between two vertices of a navigation graph using **Uniform Cost Search (UCS)** and **A\* search**, then compares how many nodes each one reaches.

The program is built around the assignment's visibility-graph problem: a robot must travel from a start point to a goal point around a convex polygonal obstacle. The shortest path must follow straight-line segments between the start, the obstacle's vertices, and the goal, so the scene reduces to a graph search.

## The Problem

In the example scene, the start is vertex `0`, the obstacle's corners are vertices `1-5`, and the goal is vertex `6`. Two vertices are connected when a straight line between them does not cross the obstacle (a *visibility graph*).

| Vertex | X | Y |
|--------|-----|-----|
| 0 (start) | -1 | 100 |
| 1 | 98 | 18 |
| 2 | 75 | 226 |
| 3 | 56 | 205 |
| 4 | 35 | 32 |
| 5 | 49 | -65 |
| 6 (goal) | 100 | 101 |

Edges (from the adjacency matrix): `0-3, 0-4, 0-5, 1-2, 1-5, 1-6, 2-3, 2-6, 3-4, 4-5`.

## How It Works

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

Expands nodes in order of f(n) = g(n) + h(n), where h(n) is the **straight-line distance (SLD)** from the vertex to the goal. SLD is **admissible** because no path can be shorter than a straight line, and it is also **consistent** because the edge costs are themselves straight-line distances. A\* therefore returns an optimal path while typically reaching fewer nodes than UCS.

### Shared search loop

1. Push the start node onto the priority queue and into the `reachedNodes` map.
2. Pop the lowest-cost node. If it is the goal, build the path with `Solution` and stop. (The goal test happens on **pop**, which is what guarantees optimality.)
3. Call `Expand` to generate the neighbors.
4. For each child, add it if its state is new, or if it was reached again at a lower cost. Update `reachedNodes` and push it onto the queue.
5. If the queue empties, no path exists.

`Solution` follows `parent` links from the goal back to the start. The path comes out goal-to-start, and `main` reverses it for printing.

## Input File Format

```
m n                    # number of vertices, number of edges
id x y                 # m lines: vertex id and coordinates
...
a b                    # n lines: an undirected edge between a and b
...
```

Example file for the scene above (`P7x10.txt`):

```
7 10
0 -1 100
1 98 18
2 75 226
3 56 205
4 35 32
5 49 -65
6 100 101
0 3
0 4
0 5
1 2
1 5
1 6
2 3
2 6
3 4
4 5
```

The start is always vertex `0` and the goal is always the last vertex (`size - 1`).

## Functions

| Function | Purpose |
|----------|---------|
| `readGraphFromFile` | Reads a graph file and fills in the adjacency matrix and coordinates |
| `Expand` | Returns the neighbors of a node with updated g-costs |
| `uniform_cost_search_Nav` | UCS from start to goal, returns the path |
| `astar_search_Nav` | A\* from start to goal, using SLD as h(n) |
| `Solution` | Rebuilds the path by following parent links |

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

## Known Limitations

- `system("pause")` in `main` works on Windows only. On macOS/Linux, replace it with `cin.get()`.
- The code uses `sqrt` and `pow` but does not include `<cmath>`. Add it if your compiler complains.
- The `start` parameter is currently ignored, since the search always begins at vertex `0`.
- In `readGraphFromFile`, the adjacency matrix is allocated with `n` (edges) instead of `m` (vertices). This works when edges >= vertices, but it can overflow for sparse graphs such as trees.
- The "size" columns count nodes **reached**, not nodes expanded.
- The adjacency matrix in the assignment lists rounded distances, but the code recomputes exact distances from the coordinates, so small rounding differences are expected.

## Project Structure

```
├── [your_file].cpp
├── P2x2m.txt ... P15x10.txt   # test graphs
└── README.md
```