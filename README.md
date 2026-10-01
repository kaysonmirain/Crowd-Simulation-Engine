# Crowd Simulation Engine

A deterministic C++ simulation of autonomous crowd movement across a configurable navigation graph.

The focus is systems behavior—route-search cost, fixed-step execution, dynamic rerouting, and interoperable telemetry—rather than another mapping or retrospective mobility-analysis application.

## How It Works

The simulator builds a configurable arena grid, routes agents with A* or Dijkstra, advances every agent through a fixed-timestep simulation, and exports event telemetry for downstream analysis. A corridor closes during the run, forcing active agents to recalculate their routes.

The implementation is split into a reusable graph library and a simulation layer. Routing records expanded-node counts so A* and Dijkstra can be compared on the same network. The command-line application produces trip completion, distance, travel-time, and routing-effort summaries.

## Results

Each run writes a CSV containing the simulation time, agent, current segment, progress, and completion state. The output can feed the separate Java Telemetry Event Processor for concurrent validation, aggregation, and API delivery.

## Project Layout

```text
include/crowd/graph.hpp        Navigation graph and route-search API
include/crowd/simulation.hpp   Agent, telemetry, and simulation API
src/graph.cpp                  A* and Dijkstra implementation
src/simulation.cpp             Fixed-step movement and dynamic rerouting
src/main.cpp                   Command-line simulation
tests/test_main.cpp            Route, closure, and completion tests
```

## Build and Run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/crowd-sim --agents 100 --steps 600 --algorithm astar
```

Telemetry is written to `results/telemetry.csv` by default.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## Limitations

The current arena is synthetic and grid-based. Local collision avoidance, navmesh generation, animation, and multi-level geometry are planned extensions rather than presented as finished behavior.
