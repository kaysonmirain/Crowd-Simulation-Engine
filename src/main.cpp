#include "crowd/simulation.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    int agent_count = 40;
    int steps = 600;
    std::string output = "results/telemetry.csv";
    crowd::SearchAlgorithm algorithm = crowd::SearchAlgorithm::astar;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--agents" && i + 1 < argc) agent_count = std::atoi(argv[++i]);
        else if (argument == "--steps" && i + 1 < argc) steps = std::atoi(argv[++i]);
        else if (argument == "--output" && i + 1 < argc) output = argv[++i];
        else if (argument == "--algorithm" && i + 1 < argc) {
            algorithm = std::string(argv[++i]) == "dijkstra"
                ? crowd::SearchAlgorithm::dijkstra
                : crowd::SearchAlgorithm::astar;
        }
    }

    crowd::Simulation simulation(crowd::make_training_grid(), algorithm);
    for (int id = 0; id < agent_count; ++id) {
        const int start = id % 8;
        const int destination = 56 + ((id * 3) % 8);
        simulation.add_agent(id + 1, start, destination);
    }

    simulation.run(steps, 1.0, 50);
    simulation.write_telemetry(output);
    const auto summary = simulation.summary();
    std::cout << "algorithm=" << crowd::algorithm_name(algorithm)
              << " agents=" << summary.agents
              << " completed=" << summary.completed
              << " simulated_seconds=" << summary.simulated_seconds
              << " average_trip_seconds=" << summary.average_trip_seconds
              << " total_distance_km=" << summary.total_distance_km
              << " routing_expansions=" << summary.routing_expansions
              << " telemetry=" << output << '\n';
}
