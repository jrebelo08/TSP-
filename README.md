# Travelling Salesman Problem (TSP) Solver

## Overview

The Travelling Salesman Problem (TSP) is a classic algorithmic problem in the fields of computer science and operations research. The objective is to find the shortest possible route that visits a set of given cities exactly once and returns to the origin city.

## Project Team

- João Rebelo
- David Ferreira

## Objectives

1. Explore different algorithms to solve the TSP.
2. Implement a solution and evaluate its performance.

## Project Description

### Key Concepts

**What is TSP?**

- **Input:** A list of cities or edges containing information on the source/destination and the distances between each pair of cities.
- **Output:** The shortest possible route that visits each city exactly once and returns to the starting point.

**Importance of TSP**

TSP has practical applications in various fields such as:
- Logistics
- Planning
- Manufacturing of microchips

### Algorithms Used to Solve TSP

**Exact Algorithms**

1. **Backtracking Approach:**
   - **Description:** Recursively explores all possible paths, marking nodes as visited or unvisited as it progresses.
   - **Process:** For each path, it calculates the total distance traveled and updates the shortest path found whenever a shorter complete tour is identified.
   - **Performance:** Computationally expensive; feasible only for toy graphs.

**Heuristic Algorithms**

1. **Triangular Approximation Heuristic:**
   - **Description:** Approximates the solution by considering triangular inequalities.

2. **Nearest Neighbour:**
   - **Description:** Starts from an arbitrary node and repeatedly visits the nearest node until all nodes are visited.
   - **Performance:** Offers a good balance between performance and efficiency.

3. **K-means Clustering Nearest Neighbour:**
   - **Description:** Combines clustering with the Nearest Neighbour heuristic.
   - **Process:** The graph nodes are divided into clusters, and the Nearest Neighbour heuristic is applied to each cluster. Results are combined to form a complete tour.

4. **Lin-Kernighan:**
   - **Description:** Iteratively improves the solution by making local changes until no further improvements can be found.
   - **Performance:** Feasible only for small graphs.

### Graph Types

- **Toy Graphs:** Simplified graphs suitable for exact algorithms.
- **Extra Fully Connected Graphs:** More complex graphs used for heuristic approaches.
