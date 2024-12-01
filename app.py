from flask import Flask, render_template, request, jsonify
import networkx as nx
import random
import json
import subprocess
import os

app = Flask(__name__)

import random

def generate_random_graph(num_nodes, max_degree):
    graph = {}
    central_node = "Warehouse"
    graph[central_node] = {}
    
    # Create delivery locations
    nodes = [f"Location {i}" for i in range(1, num_nodes + 1)]

    # Connect warehouse to a random subset of locations
    connected_locations = random.sample(nodes, random.randint(1, min(len(nodes), max_degree)))
    for node in connected_locations:
        distance = random.randint(1, 20)
        graph[central_node][node] = distance
        graph[node] = {central_node: distance}  # Bidirectional

    # Ensure all nodes are connected
    unconnected_nodes = set(nodes) - set(connected_locations)
    if unconnected_nodes:
        previous_node = connected_locations[0]
        for node in unconnected_nodes:
            distance = random.randint(1, 20)
            if node not in graph:
                graph[node] = {}
            graph[previous_node][node] = distance
            graph[node][previous_node] = distance
            previous_node = node

    # Add edges while ensuring degrees remain within max_degree
    for node in nodes:
        current_neighbors = set(graph[node].keys())
        possible_neighbors = set(nodes) - current_neighbors - {node}
        remaining_degree = max(0, max_degree - len(current_neighbors))  # Ensure non-negative
        additional_edges = random.sample(list(possible_neighbors), min(len(possible_neighbors), remaining_degree))
        for neighbor in additional_edges:
            if len(graph[neighbor]) < max_degree:  # Check degree constraint
                distance = random.randint(1, 20)
                graph[node][neighbor] = distance
                graph[neighbor][node] = distance

    # Ensure the warehouse is within max degree
    if len(graph[central_node]) > max_degree:
        extra_edges = list(graph[central_node].keys())[max_degree:]
        for node in extra_edges:
            del graph[central_node][node]
            del graph[node][central_node]

    return graph




def compile_and_run_cpp(graph, start, end):

    # Get absolute paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cpp_path = os.path.join(script_dir, "dijkstra_cpp")
    json_path = os.path.join(script_dir, "graph_data.json")

    # Write JSON data to file
    graph_json = {"graph": graph, "start": start, "end": end}
    with open(json_path, "w") as f:
        json.dump(graph_json, f)

    # Compile C++ program
    compile_command = f"g++ {os.path.join(script_dir, 'dijkstra_cpp.cpp')} -o {cpp_path}"
    compile_result = subprocess.run(
        compile_command,
        shell=True,
        capture_output=True,
        text=True
    )
    if compile_result.returncode != 0:
        raise RuntimeError(f"Compilation failed: {compile_result.stderr}")

    # Run the compiled program
    with open(json_path, "r") as input_file:
        process = subprocess.Popen(
            cpp_path,
            stdin=input_file,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        stdout, stderr = process.communicate()
        if process.returncode != 0:
            raise RuntimeError(f"Execution failed: {stderr}")

    # Parse the output
    cpp_time_microseconds = int(stdout.strip())
    cpp_time_seconds = cpp_time_microseconds / 1_000_000  # Convert to seconds
    return {"cpp_time": cpp_time_seconds}

    
    
def dijkstra(graph, start, end):
    import heapq

    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == end:
            path = []
            while current_node:
                path.append(current_node)
                current_node = previous_nodes.get(current_node)
            return path[::-1], current_distance

        # Skip if node not in graph
        if current_node not in graph:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    return [], float('inf')  # No path found


@app.route('/')
def index():
    return render_template("index.html")

@app.route('/generate_graph', methods=['POST'])
def generate_graph():
    data = request.get_json()
    num_nodes = int(data['num_nodes'])
    max_degree = int(data['max_degree'])
    
    graph = generate_random_graph(num_nodes, max_degree)
    return jsonify(graph)



@app.route('/calculate', methods=['POST'])
def calculate():
    data = request.json
    graph = data['graph']
    start = data['start'].strip()
    end = data['end'].strip()

    if start not in graph or end not in graph:
        return jsonify({
            "error": f"Invalid nodes: {start} or {end} not in graph.",
            "path": [],
            "distance": None,
            "python_time": None,
            "cpp_time": None
        }), 400

    import time
    start_time = time.perf_counter()
    path, total_distance = dijkstra(graph, start, end)
    python_time = time.perf_counter() - start_time

    # C++ Execution
    cpp_result = compile_and_run_cpp(graph, start, end)

    if "error" in cpp_result:
        cpp_time = cpp_result["error"]
    else:
        cpp_time = cpp_result["cpp_time"]

    return jsonify({
        "path": path,
        "distance": total_distance,
        "python_time": python_time,
        "cpp_time": cpp_time
    })
    

if __name__ == '__main__':
    app.run(debug=True)
