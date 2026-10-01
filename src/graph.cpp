#include "crowd/graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace crowd {

void Graph::add_node(int id, double x, double y) {
    nodes_[id] = Node{id, x, y};
    adjacency_.try_emplace(id);
}

void Graph::add_bidirectional_edge(int from, int to, double distance_km, double speed_kph) {
    if (!nodes_.contains(from) || !nodes_.contains(to)) {
        throw std::invalid_argument("both road endpoints must exist");
    }
    adjacency_[from].push_back(Edge{to, distance_km, speed_kph, true});
    adjacency_[to].push_back(Edge{from, distance_km, speed_kph, true});
}

void Graph::set_road_open(int from, int to, bool open) {
    auto update = [open](std::vector<Edge>& edges, int destination) {
        for (auto& edge : edges) {
            if (edge.to == destination) edge.open = open;
        }
    };
    update(adjacency_.at(from), to);
    update(adjacency_.at(to), from);
}

const Node& Graph::node(int id) const { return nodes_.at(id); }

const std::vector<Edge>& Graph::edges(int id) const { return adjacency_.at(id); }

double Graph::edge_distance(int from, int to) const {
    for (const auto& edge : edges(from)) {
        if (edge.to == to) return edge.distance_km;
    }
    throw std::out_of_range("edge not found");
}

Route Graph::shortest_path(int start, int goal, SearchAlgorithm algorithm) const {
    if (!nodes_.contains(start) || !nodes_.contains(goal)) return {};

    struct QueueItem {
        double priority;
        int node;
        bool operator>(const QueueItem& other) const { return priority > other.priority; }
    };

    auto heuristic = [&](int id) {
        if (algorithm == SearchAlgorithm::dijkstra) return 0.0;
        const auto& a = node(id);
        const auto& b = node(goal);
        return std::hypot(a.x - b.x, a.y - b.y);
    };

    const double infinity = std::numeric_limits<double>::infinity();
    std::unordered_map<int, double> distance;
    std::unordered_map<int, int> previous;
    for (const auto& [id, ignored] : nodes_) {
        (void)ignored;
        distance[id] = infinity;
    }

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> frontier;
    distance[start] = 0.0;
    frontier.push({heuristic(start), start});
    std::size_t expanded = 0;

    while (!frontier.empty()) {
        const int current = frontier.top().node;
        const double queued_priority = frontier.top().priority;
        frontier.pop();
        if (queued_priority > distance[current] + heuristic(current) + 1e-9) continue;
        ++expanded;
        if (current == goal) break;

        for (const auto& edge : edges(current)) {
            if (!edge.open) continue;
            const double candidate = distance[current] + edge.distance_km;
            if (candidate < distance[edge.to]) {
                distance[edge.to] = candidate;
                previous[edge.to] = current;
                frontier.push({candidate + heuristic(edge.to), edge.to});
            }
        }
    }

    if (!std::isfinite(distance[goal])) return Route{{}, 0.0, expanded, false};

    std::vector<int> path;
    for (int current = goal;; current = previous.at(current)) {
        path.push_back(current);
        if (current == start) break;
    }
    std::reverse(path.begin(), path.end());
    return Route{path, distance[goal], expanded, true};
}

Graph make_training_grid(int width, int height) {
    Graph graph;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int id = y * width + x;
            graph.add_node(id, x * 0.25, y * 0.25);
        }
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int id = y * width + x;
            if (x + 1 < width) graph.add_bidirectional_edge(id, id + 1, 0.25, 40.0);
            if (y + 1 < height) graph.add_bidirectional_edge(id, id + width, 0.25, 30.0);
        }
    }
    return graph;
}

std::string algorithm_name(SearchAlgorithm algorithm) {
    return algorithm == SearchAlgorithm::astar ? "astar" : "dijkstra";
}

}  // namespace crowd
