#include "crowd/graph.hpp"
#include "crowd/simulation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    {
        const auto graph = crowd::make_training_grid(4, 4);
        const auto dijkstra = graph.shortest_path(0, 15, crowd::SearchAlgorithm::dijkstra);
        const auto astar = graph.shortest_path(0, 15, crowd::SearchAlgorithm::astar);
        assert(dijkstra.found && astar.found);
        assert(std::abs(dijkstra.distance_km - 1.5) < 1e-9);
        assert(std::abs(dijkstra.distance_km - astar.distance_km) < 1e-9);
        assert(astar.expanded_nodes <= dijkstra.expanded_nodes);
    }
    {
        auto graph = crowd::make_training_grid(4, 4);
        graph.set_road_open(0, 1, false);
        const auto route = graph.shortest_path(0, 3, crowd::SearchAlgorithm::astar);
        assert(route.found);
        assert(route.distance_km > 0.75);
    }
    {
        crowd::Simulation simulation(crowd::make_training_grid(), crowd::SearchAlgorithm::astar);
        simulation.add_agent(1, 0, 63);
        simulation.run(500, 1.0, 20);
        assert(simulation.summary().completed == 1);
        assert(!simulation.telemetry().empty());
    }
    std::cout << "crowd simulation tests passed\n";
}
