import osmnx as ox
import networkx as nx
import random
import matplotlib.pyplot as plt
import heapq

def dijkstra(graph, start, goal):
    queue = [(0, start)]
    costs = {start: 0}
    parents = {start: None}

    while queue:
        current_cost, current_node = heapq.heappop(queue)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parents[current_node]
            return path[::-1], parents

        for neighbor in graph.neighbors(current_node):
            original_weight = graph[current_node][neighbor][0]['length']
            weight = original_weight * random.uniform(0.5, 2.0)
            cost = current_cost + weight

            if neighbor not in costs or cost < costs[neighbor]:
                costs[neighbor] = cost
                parents[neighbor] = current_node
                heapq.heappush(queue, (cost, neighbor))

    return None, None

def visualize_graph(graph, start, goal, parents):
    path = []
    current_node = goal
    while current_node is not None:
        path.append(current_node)
        current_node = parents[current_node]
    path = path[::-1]

    pos = {node: (data['x'], data['y']) for node, data in graph.nodes(data=True)}

    fig, ax = plt.subplots(figsize=(20, 20))
    
    node_colors = ['blue' if node != start and node != goal else 'green' if node == start else 'red' for node in graph.nodes()]
    
    nx.draw(graph, pos, node_size=10, node_color=node_colors, edge_color='gray', ax=ax)
    nx.draw(graph, pos, nodelist=path, node_size=50, node_color='red', ax=ax)
    nx.draw_networkx_edges(graph, pos, edgelist=list(zip(path, path[1:])), edge_color='red', width=2, ax=ax)
    plt.show()

def main():
    place_name = input("Enter a place name (e.g., 'Mahindra University, Hyderabad'): ")
    dist = 1000

    try:
        print(f"Fetching graph for {place_name}...")
        graph = ox.graph_from_address(place_name, dist=dist, network_type='drive')
    except Exception as e:
        print(f"Error fetching graph: {e}")
        print("Please try a different place name.")
        return

    nx_graph = nx.MultiDiGraph(graph)

    source = list(nx_graph.nodes())[random.randint(0, len(nx_graph.nodes()) - 1)]
    destination = list(nx_graph.nodes())[random.randint(0, len(nx_graph.nodes()) - 1)]
    while destination == source:
        destination = list(nx_graph.nodes())[random.randint(0, len(nx_graph.nodes()) - 1)]

    print(f"Source: {source}, Destination: {destination}")

    path, parent = dijkstra(nx_graph, source, destination)
    if path:
        print(f"Shortest path: {' -> '.join(map(str, path))}")
        visualize_graph(nx_graph, source, destination, parent)
    else:
        print("No path found")

if __name__ == "__main__":
    main()