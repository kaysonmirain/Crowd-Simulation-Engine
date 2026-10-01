#include "crowd/simulation.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <stdexcept>

namespace crowd {

Simulation::Simulation(Graph graph, SearchAlgorithm algorithm)
    : graph_(std::move(graph)), algorithm_(algorithm) {}

void Simulation::add_agent(int id, int start, int destination) {
    const Route route = graph_.shortest_path(start, destination, algorithm_);
    if (!route.found) throw std::runtime_error("agent route could not be found");
    routing_expansions_ += route.expanded_nodes;
    agents_.push_back(Agent{id, destination, route.nodes});
}

void Simulation::reroute(Agent& agent, int current_node) {
    const Route route = graph_.shortest_path(current_node, agent.destination, algorithm_);
    routing_expansions_ += route.expanded_nodes;
    if (!route.found) return;
    agent.path = route.nodes;
    agent.path_index = 0;
    agent.segment_progress_km = 0.0;
}

void Simulation::close_road(int from, int to) {
    graph_.set_road_open(from, to, false);
    for (auto& agent : agents_) {
        if (agent.completed || agent.path.empty()) continue;
        const int current = agent.path[std::min(agent.path_index, agent.path.size() - 1)];
        reroute(agent, current);
    }
}

void Simulation::record(const Agent& agent) {
    const int from = agent.path.empty() ? -1 : agent.path[std::min(agent.path_index, agent.path.size() - 1)];
    const int to = agent.completed || agent.path_index + 1 >= agent.path.size()
        ? from
        : agent.path[agent.path_index + 1];
    double progress = 1.0;
    if (!agent.completed && from != to) {
        progress = agent.segment_progress_km / graph_.edge_distance(from, to);
    }
    telemetry_.push_back({time_seconds_, agent.id, from, to, std::clamp(progress, 0.0, 1.0), agent.completed});
}

void Simulation::tick(double delta_seconds) {
    time_seconds_ += delta_seconds;
    for (auto& agent : agents_) {
        if (agent.completed) {
            record(agent);
            continue;
        }
        agent.elapsed_seconds += delta_seconds;
        double remaining_km = 0.035 * delta_seconds / 3.6;  // 35 km/h converted to km/s.

        while (remaining_km > 0.0 && !agent.completed) {
            if (agent.path_index + 1 >= agent.path.size()) {
                agent.completed = true;
                break;
            }
            const int from = agent.path[agent.path_index];
            const int to = agent.path[agent.path_index + 1];
            const double segment = graph_.edge_distance(from, to);
            const double available = segment - agent.segment_progress_km;
            const double travelled = std::min(available, remaining_km);
            agent.segment_progress_km += travelled;
            agent.distance_travelled_km += travelled;
            remaining_km -= travelled;
            if (agent.segment_progress_km + 1e-9 >= segment) {
                ++agent.path_index;
                agent.segment_progress_km = 0.0;
                if (agent.path_index + 1 >= agent.path.size()) agent.completed = true;
            }
        }
        record(agent);
    }
}

void Simulation::run(int steps, double delta_seconds, int closure_step) {
    for (int step = 0; step < steps; ++step) {
        if (step == closure_step) close_road(27, 28);
        tick(delta_seconds);
    }
}

void Simulation::write_telemetry(const std::string& path) const {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream output(path);
    output << "time_seconds,agent_id,from_node,to_node,progress,completed\n";
    output << std::fixed << std::setprecision(3);
    for (const auto& row : telemetry_) {
        output << row.time_seconds << ',' << row.agent_id << ',' << row.from_node << ',' << row.to_node
               << ',' << row.progress << ',' << (row.completed ? 1 : 0) << '\n';
    }
}

SimulationSummary Simulation::summary() const {
    SimulationSummary result;
    result.agents = static_cast<int>(agents_.size());
    result.completed = static_cast<int>(std::count_if(agents_.begin(), agents_.end(), [](const Agent& a) {
        return a.completed;
    }));
    result.simulated_seconds = time_seconds_;
    result.total_distance_km = std::accumulate(agents_.begin(), agents_.end(), 0.0, [](double total, const Agent& a) {
        return total + a.distance_travelled_km;
    });
    double completed_time = 0.0;
    for (const auto& agent : agents_) {
        if (agent.completed) completed_time += agent.elapsed_seconds;
    }
    result.average_trip_seconds = result.completed == 0 ? 0.0 : completed_time / result.completed;
    result.routing_expansions = routing_expansions_;
    return result;
}

}  // namespace crowd
