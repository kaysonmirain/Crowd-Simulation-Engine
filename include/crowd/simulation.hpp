#pragma once

#include "crowd/graph.hpp"

#include <string>
#include <vector>

namespace crowd {

struct Agent {
    int id{};
    int destination{};
    std::vector<int> path;
    std::size_t path_index{};
    double segment_progress_km{};
    double distance_travelled_km{};
    double elapsed_seconds{};
    bool completed{};
};

struct TelemetryRecord {
    double time_seconds{};
    int agent_id{};
    int from_node{};
    int to_node{};
    double progress{};
    bool completed{};
};

struct SimulationSummary {
    int agents{};
    int completed{};
    double simulated_seconds{};
    double average_trip_seconds{};
    double total_distance_km{};
    std::size_t routing_expansions{};
};

class Simulation {
public:
    Simulation(Graph graph, SearchAlgorithm algorithm);

    void add_agent(int id, int start, int destination);
    void close_road(int from, int to);
    void tick(double delta_seconds);
    void run(int steps, double delta_seconds, int closure_step = -1);
    void write_telemetry(const std::string& path) const;

    [[nodiscard]] const std::vector<Agent>& agents() const { return agents_; }
    [[nodiscard]] const std::vector<TelemetryRecord>& telemetry() const { return telemetry_; }
    [[nodiscard]] SimulationSummary summary() const;

private:
    void reroute(Agent& agent, int current_node);
    void record(const Agent& agent);

    Graph graph_;
    SearchAlgorithm algorithm_;
    std::vector<Agent> agents_;
    std::vector<TelemetryRecord> telemetry_;
    double time_seconds_{};
    std::size_t routing_expansions_{};
};

}  // namespace crowd
