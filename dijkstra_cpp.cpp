#include <iostream>
#include <unordered_map>
#include <queue>
#include <vector>
#include <limits>
#include <chrono>
#include <set>
#include "json.hpp"  // Include the nlohmann::json library

using namespace std;
using json = nlohmann::json;

// Define a type for the graph
using Graph = unordered_map<string, unordered_map<string, int>>;

// Function to perform Dijkstra's algorithm using a more optimized approach
pair<vector<string>, int> dijkstra(const Graph& graph, const string& start, const string& end) {
    unordered_map<string, int> distances;
    unordered_map<string, string> previous_nodes;
    vector<string> path;

    // Initialize distances to infinity for all nodes
    for (const auto& node : graph) {
        distances[node.first] = numeric_limits<int>::max();
    }
    distances[start] = 0;

    // Set to store nodes and their corresponding distances to maintain sorted order
    set<pair<int, string>> pq;  // {distance, node}
    pq.insert({0, start});

    while (!pq.empty()) {
        string current = pq.begin()->second;  // Get the node with the smallest distance
        int current_dist = pq.begin()->first;
        pq.erase(pq.begin());  // Remove the node from the priority queue

        // If we've reached the end node, we can stop
        if (current == end) break;

        for (const auto& neighbor : graph.at(current)) {
            int new_distance = current_dist + neighbor.second;
            if (new_distance < distances[neighbor.first]) {
                // Update the distance if a shorter path is found
                distances[neighbor.first] = new_distance;
                previous_nodes[neighbor.first] = current;

                // Add the updated node to the set
                pq.insert({new_distance, neighbor.first});
            }
        }
    }

    // Reconstruct the path by following previous nodes
    for (string at = end; at != ""; at = previous_nodes[at]) {
        path.push_back(at);
        if (at == start) break;
    }

    // The path is already constructed in reverse order, so no need to reverse it here
    return {path, distances[end]};
}

int main() {
    // Read the graph and parameters from stdin as JSON
    json graph_data;
    cin >> graph_data;

    // Extract the graph, start, and end nodes
    Graph graph = graph_data["graph"].get<Graph>();
    string start = graph_data["start"];
    string end = graph_data["end"];

    // Start timing the algorithm
    auto start_time = chrono::high_resolution_clock::now();

    // Run Dijkstra's algorithm
    auto [path, distance] = dijkstra(graph, start, end);

    // Stop timing the algorithm
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);

    // Output only the algorithm's execution time for easy parsing in Python
    cout << duration.count() << endl;

    return 0;
}
