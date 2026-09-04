# HeuristIQ — Heuristic-Driven Pathfinding & Graph Search Engine (C++)

![build-and-test](https://github.com/adiityaaa24/HeuristIQ/actions/workflows/ci.yml/badge.svg)

A from-scratch C++17 implementation of the **A\* search algorithm**, applied to
both a 2D grid (classic pathfinding-with-obstacles problem) and a generic
weighted graph (adjacency-list based, like a road network). Built to
demonstrate core data structures & algorithms concepts: priority queues
(binary heaps), hash maps, graph traversal, and heuristic search.

## Live visualizer

`web/index.html` is a self-contained browser demo of the same algorithm —
draw walls and weighted terrain, drop the start/goal anywhere, pick a
heuristic, and watch A* expand the grid node by node. No install or build
step: open the file directly in any browser, or host it for free on GitHub
Pages and link it from your resume.

```
web/index.html   # double-click to open, or serve it as a static page
```

It reimplements the same core logic as the C++ engine (binary-heap priority
queue, admissible heuristics, corner-clip prevention, weighted cells) in
plain JavaScript, plus a one-click "Compare with Dijkstra" button that shows
how many fewer nodes A* expands for the same optimal path.

## Features

- **Grid-based A\*** (`AStarGrid`)
  - 4-directional or 8-directional (diagonal) movement
  - Weighted terrain support (cells can cost more than 1 to enter, e.g. "swamp")
  - Corner-clipping prevention for diagonal moves
  - Multiple interchangeable heuristics: Manhattan, Euclidean, Diagonal (octile), Zero
  - Loads maps from plain-text files (`#` = wall, `.` = open, `2-9` = weighted cell)
  - ASCII visualization of the explored set and final path
- **Generic Graph A\*** (`GraphAStar<NodeId>`)
  - Header-only template, works with any hashable node type (`int`, `std::string`, custom structs)
  - Takes a user-supplied heuristic function — plug in any admissible heuristic
  - Useful as a template for real graph-search problems beyond grids
- **A\* vs Dijkstra comparison** — running A\* with a zero heuristic degenerates
  to Dijkstra's algorithm; the demo runs both on the same grid and prints
  nodes-expanded to show how the heuristic prunes the search space while the
  optimal path cost stays identical.
- Performance stats: nodes expanded, path cost, wall-clock time.

## Why this project is a good DSA showcase

| Concept | Where it shows up |
|---|---|
| Priority Queue / Binary Heap | `std::priority_queue` drives the A\* open set |
| Hash Maps / Hash Sets | `gScore`, `cameFrom`, and the closed set (O(1) lookups) |
| Graph representation | Implicit grid graph + explicit adjacency-list graph |
| Greedy + Dynamic elements | A\* combines Dijkstra's guarantees with a greedy heuristic |
| Algorithm complexity | O(E log V) with a binary heap, same as Dijkstra |
| Admissible heuristics | Manhattan / Euclidean / Diagonal implementations & their trade-offs |
| Templates / Generic programming | `GraphAStar<NodeId, Hash>` works over any node type |

## Project structure

```
HeuristIQ/
├── include/
│   ├── Grid.h          # Grid representation (obstacles, costs, neighbours)
│   ├── AStarGrid.h      # A* search over a Grid
│   └── GraphAStar.h     # Generic header-only A* for adjacency-list graphs
├── src/
│   ├── Grid.cpp
│   ├── AStarGrid.cpp
│   └── main.cpp          # CLI demo: grid search, A* vs Dijkstra, graph search
├── maps/
│   └── sample_map.txt    # Example map file
├── web/
│   └── index.html         # Browser-based A* visualizer (no build step)
├── tests/
│   ├── mini_test.h         # Tiny dependency-free test harness
│   ├── test_astar_grid.cpp
│   └── test_graph_astar.cpp
├── CMakeLists.txt
├── Makefile
└── README.md
```

## How A* works (quick summary)

A\* maintains a priority queue ("open set") of nodes ordered by
`f(n) = g(n) + h(n)`, where:

- `g(n)` = exact cost of the best known path from **start** to `n`
- `h(n)` = heuristic estimate of the cost from `n` to **goal**

At each step it expands the node with the lowest `f(n)`. As long as `h(n)`
never overestimates the true remaining cost (an **admissible** heuristic),
A\* is guaranteed to find the optimal path — while typically expanding far
fewer nodes than an uninformed search like Dijkstra's algorithm, because the
heuristic biases exploration toward the goal.

**Complexity:** O(E log V) time with a binary-heap priority queue (V = cells/nodes,
E = edges/possible moves), O(V) space for the score maps and closed set.

## Building & running

### Option 1: Makefile (g++)
```bash
make          # builds ./heuristiq
make run      # builds and runs with the sample map
```

### Option 2: CMake
```bash
mkdir build && cd build
cmake ..
cmake --build .
./heuristiq ../maps/sample_map.txt
```

### Usage
```bash
./heuristiq                     # uses a built-in demo grid
./heuristiq maps/sample_map.txt # loads a map from file
```

## Running the tests

```bash
make test          # builds and runs both test suites, prints PASS/FAIL per case
```

Tests use a tiny, dependency-free header-only harness (`tests/mini_test.h`) —
no GTest/Catch2 download required, just plain C++17, so `make test` works
anywhere the main build does. 11 test cases cover:

- straight-line and detour pathfinding, with exact cost checks
- unreachable goals (walled off, or start/goal on an obstacle)
- corner-clipping prevention on diagonal moves
- weighted terrain cost accumulation
- A* vs Dijkstra optimality agreement
- the generic `GraphAStar<T>` template with both `std::string` and `int` node types

With CMake, the same tests are registered with CTest:
```bash
cmake --build build && cd build && ctest --output-on-failure
```

### Map file format
```
. = walkable cell (cost 1)
# = obstacle (blocked)
2-9 = walkable cell with that movement cost (weighted terrain)
```

## Sample output

```
=== Grid Pathfinding (A*) ===
S * : : : : : : : : # . . . . . . . . .
: : * : : : : : : : # . . . . . . . . .
...
Legend: S=start  G=goal  *=path  :=explored  #=obstacle
Path length (steps): 22
Path cost:           24.485
Nodes expanded:      69
Time taken:          0.077 ms

=== A* vs Dijkstra (heuristic = 0) — nodes expanded comparison ===
A*       -> nodes expanded: 69, cost: 24.485
Dijkstra -> nodes expanded: 170, cost: 24.485
```

## Possible extensions (good talking points in interviews)

- Add **Jump Point Search (JPS)** for faster uniform-cost grid pathfinding
- Add **bidirectional A\*** (search from both start and goal simultaneously)
- Swap the binary heap for a **Fibonacci heap** and compare performance
- Add a real-time SFML/SDL visualizer instead of ASCII output
- Extend `GraphAStar` to load real road-network data (e.g. OpenStreetMap) and
  use haversine distance as the heuristic

## License

MIT — free to use for learning, coursework, or portfolio projects.
