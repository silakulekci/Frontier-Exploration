# Frontier-Exploration
# 🤖 Robot Navigation Suite

A browser-based robotics simulation suite built during an internship, covering two core navigation problems: **cost-aware path planning** and **autonomous area exploration**. No dependencies, no build step — open the HTML files and run.

---

## Overview

This project consists of two simulators that progressively model real-world autonomous robot behavior:

| Simulator | File | Core Concept |
|---|---|---|
| Costmap & A\* Navigation | `costmap_navigation.html` | Inflation Layer + cost-weighted pathfinding |
| Frontier Exploration | `frontier_exploration.html` | Comparative exploration with live metrics |

---

## Simulator 1 — Costmap & A* Navigation

### Motivation

A binary occupancy grid treats every obstacle as a hard boundary and every free cell as equally safe. In practice, a robot moving too close to a wall risks collision due to sensor noise, wheel slip, or physical width. The solution used in professional robotics frameworks (like ROS) is a **Costmap**: a continuous risk map layered on top of the occupancy grid.

### How It Works

**Inflation Layer**

A BFS (Breadth-First Search) wave propagates outward from every obstacle cell, computing the Euclidean distance from each free cell to its nearest obstacle. A cost is then assigned using exponential decay:

```
cost(d) = 253 × e^(−scale × d)
```

where `d` is the distance to the nearest obstacle and `scale` is the `cost_scaling_factor` parameter. Cells within the inflation radius receive a cost value; cells beyond it remain at zero (safe).

**Cost-Weighted A\***

The A\* heuristic remains standard Euclidean distance, but the movement cost is augmented:

```
g(n) = step_distance + cost(n) × 0.04
```

This makes cells near obstacles "expensive" to traverse. The planner naturally avoids corridor-hugging paths in favor of routes centered in open space — without any explicit safety rule.

**Dead-End Avoidance**

Because the costmap assigns near-lethal values to obstacle-adjacent cells, narrow corridors become costly by definition. The robot is implicitly steered toward wider passages.

### Parameters

| Parameter | Range | Effect |
|---|---|---|
| Inflation radius | 0 – 6 cells | How far the risk zone extends from obstacles |
| Cost scaling factor | 0.1 – 2.0 | How steeply cost drops with distance (higher = sharper boundary) |
| View mode | Costmap / Occupancy | Toggle between heatmap and binary grid visualization |

### Key Engineering Insight

> Increasing the scaling factor produces a narrow, sharp risk boundary — the robot passes closer to walls but with a steep penalty. Decreasing it spreads cost widely, forcing the robot to favor the center of corridors. This trade-off mirrors the `cost_scaling_factor` parameter in ROS Navigation Stack's `inflation_layer`.

---

## Simulator 2 — Frontier Exploration

### Motivation

Coverage efficiency depends on two factors: **time to explore** and **battery consumed**. Random Walk and Wall Following are simple strategies but suffer from systematic inefficiencies — redundant revisits and incomplete coverage. Frontier Exploration addresses this by directing the robot toward the boundary between known and unknown space.

### Algorithms Compared

#### Frontier Exploration *(developed)*

A **frontier** is any unknown cell adjacent to a known-free cell — the edge of explored territory.

At each decision step:
1. All frontier cells are extracted via a full grid scan.
2. Each frontier is scored using a cost-benefit function:

```
score(f) = unknown_neighbors / (manhattan_distance + 1 + dead_end_penalty)
```

3. The highest-scoring frontier is selected as the navigation target.
4. An **A\*** path is computed from the robot's current position to the target.
5. The robot follows the path one step per simulation tick.

**Dead-End Detection:** If a frontier cell is surrounded by obstacles on 3 or more sides, it receives a penalty of +20 in the denominator, making it low-priority without discarding it entirely.

**Frontier fallback:** If the selected frontier is unreachable (disconnected region), the algorithm iterates through the next-best candidates automatically.

#### Random Walk *(reference baseline)*

At each step, the robot continues in its current direction with an 80% probability, or picks a random new direction with 20% probability. On collision, it rotates to the next available direction. No memory of visited cells is maintained.

**Known weakness:** The robot revisits already-explored regions frequently. Achieving 90% coverage requires disproportionately many steps due to the coupon-collector problem.

#### Wall Following — Left-Hand Rule *(reference baseline)*

The robot always attempts to turn left first. If the left cell is blocked, it moves straight. If both are blocked, it turns right. This guarantees complete coverage of simply-connected environments but fails in rooms with islands or disconnected obstacles, where it can loop indefinitely.

### Metrics Collected

| Metric | Description |
|---|---|
| **Steps** | Total simulation ticks elapsed |
| **Distance** | Cumulative cells traveled (path length) |
| **Battery** | Remaining capacity (depletes 1 unit/step) |
| **Coverage** | Fraction of reachable free cells revealed |
| **Efficiency** | Coverage percentage per 1000 steps |

### Results (typical run, 30×22 map, 18% obstacle density)

| Algorithm | Coverage | Steps to 95% | Distance | Battery Left |
|---|---|---|---|---|
| **Frontier** | 95%+ | ~420 | ~390 | ~80% |
| Random Walk | 95%+ | ~1800 | ~1800 | ~10% |
| Wall Following | 85–95% | — | ~1200 | ~40% |

> Frontier Exploration reached the 95% coverage target using approximately **35% less total distance** than Random Walk, validating the cost-benefit scoring approach.

---

## Project Structure

```
robot-navigation-suite/
├── costmap_navigation.html     # Simulator 1 — Costmap + A*
├── frontier_exploration.html   # Simulator 2 — Exploration comparison
└── README.md
```

---

## Usage

1. Clone or download the repository.
2. Open either `.html` file directly in a modern browser (Chrome, Firefox, Edge).
3. No server, no npm, no dependencies required.

```bash
git clone https://github.com/YOUR_USERNAME/robot-navigation-suite.git
cd robot-navigation-suite
open costmap_navigation.html   # or double-click
```

---

## Controls

### Costmap Navigator
- **Click / drag** on the canvas to place obstacles
- **Erase mode** to remove obstacles
- **S / G buttons** to set Start and Goal
- Sliders adjust inflation radius and cost scaling factor in real time

### Frontier Explorer
- **▶ Start** — launches all three agents simultaneously on the same map
- **New Map** — regenerates the environment with current density setting
- **Speed slider** — from slow-motion to maximum tick rate
- Live comparison table updates every frame

---

## Technical Notes

### Why BFS for distance computation?
BFS on a grid guarantees shortest-path distances from the obstacle set in O(N) time — cheaper than running Dijkstra from each obstacle independently. The 8-directional BFS with diagonal cost √2 approximates Euclidean distance well for the cell sizes used here.

### Why A\* over BFS for navigation?
BFS expands uniformly in all directions regardless of goal position. A\* uses the Manhattan/Euclidean heuristic to bias expansion toward the target, reducing the number of nodes evaluated. On a 40×30 grid, this difference is measurable but not critical — at larger scales (e.g. 200×200) A\* becomes significantly faster.

### Frontier scoring rationale
A pure nearest-frontier strategy minimizes travel per step but ignores information gain — the robot may chase a frontier that reveals only one new cell. Weighting by `unknown_neighbors` biases the robot toward open unexplored regions. The dead-end penalty prevents the robot from committing to narrow passages early in exploration.

---

## Background

This project was developed as part of a robotics software internship. The simulators were built to deepen understanding of:

- Occupancy grid mapping and sensor modeling
- ROS-style costmap architecture (inflation layer, cost scaling)
- Classical path planning (BFS, A\*) and their trade-offs
- Autonomous exploration strategies and their empirical evaluation

The implementations are intentionally self-contained (vanilla HTML/CSS/JS) to maximize portability and to keep the algorithmic logic transparent without framework overhead.

---

## License

MIT — free to use, modify, and distribute.
