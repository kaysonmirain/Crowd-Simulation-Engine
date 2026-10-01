#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace crowd {

struct Node {
    int id{};
    double x{};
    double y{};
};

struct Edge {
    int to{};
    double distance_km{};
    double speed_kph{};
    bool open{true};
};

enum class SearchAlgorithm { dijkstra, astar };

struct Route {
    std::vector<int> nodes;
    double distance_km{};
    std::size_t expanded_nodes{};
    bool found{};
};

class Graph {
public:
    void add_node(int id, double x, double y);
    void add_bidirectional_edge(int from, int to, double distance_km, double speed_kph);
    void set_road_open(int from, int to, bool open);

    [[nodiscard]] const Node& node(int id) const;
    [[nodiscard]] const std::vector<Edge>& edges(int id) const;
    [[nodiscard]] Route shortest_path(int start, int goal, SearchAlgorithm algorithm) const;
    [[nodiscard]] double edge_distance(int from, int to) const;

private:
    std::unordered_map<int, Node> nodes_;
    std::unordered_map<int, std::vector<Edge>> adjacency_;
};

Graph make_training_grid(int width = 8, int height = 8);
std::string algorithm_name(SearchAlgorithm algorithm);

}  // namespace crowd
