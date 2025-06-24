import matplotlib.pyplot as plt
import networkx as nx
from collections import deque

# Graph from DLSU Food Map (adjacency list)
graph = {
    'A': ['R', 'T'],
    'B': ['Q', 'R'],
    'C': ['P', 'Q', 'R'],
    'D': ['E', 'F', 'G'],
    'E': ['D', 'O', 'P'],
    'F': ['D', 'H', 'O'],
    'G': ['D', 'H', 'I', 'L', 'N', 'O'],
    'H': ['F', 'G', 'I'],
    'I': ['G', 'H', 'L', 'M', 'N'],
    'J': ['J', 'L', 'M', 'N'],
    'K': ['J', 'M', 'N', 'U'],
    'L': ['G', 'I', 'J'],
    'M': ['I', 'J', 'K'],
    'N': ['G', 'I', 'J', 'K'],
    'O': ['E', 'F', 'G', 'P', 'S'],
    'P': ['C', 'E', 'O'],
    'Q': ['B', 'C'],
    'R': ['A', 'B', 'C', 'S'],
    'S': ['O', 'R', 'T'],
    'T': ['A', 'S'],
}

def bfs(start, goal):
    visited = set()
    queue = deque([[start]])

    if start == goal:
        return [start]

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node not in visited:
            visited.add(node)

            for neighbor in graph.get(node, []):
                new_path = path + [neighbor]
                if neighbor == goal:
                    return new_path
                queue.append(new_path)

    return None  # No path found

def visualize_graph(graph, path):
    G = nx.Graph()

    # Add edges to the graph
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            G.add_edge(node, neighbor)

    pos = nx.spring_layout(G, seed=42)  # Set fixed layout

    # Highlight path edges
    edge_colors = []
    path_edges = set(zip(path, path[1:])) if path else set()

    for edge in G.edges():
        if edge in path_edges or (edge[1], edge[0]) in path_edges:
            edge_colors.append('red')
        else:
            edge_colors.append('lightgray')

    # Draw nodes and edges
    nx.draw(G, pos, with_labels=True, node_color='lightgreen', edge_color=edge_colors, node_size=600, font_weight='bold')
    
    # Highlight start and goal
    if path:
        nx.draw_networkx_nodes(G, pos, nodelist=[path[0]], node_color='yellow', node_size=700)
        nx.draw_networkx_nodes(G, pos, nodelist=[path[-1]], node_color='orange', node_size=700)

    plt.title("DLSU Food Map - BFS Path")
    plt.show()

def main():
    print("DLSU Food Finder")
    start = input("Enter starting point (e.g., A): ").strip().upper()
    goal = input("Enter goal (e.g., T): ").strip().upper()

    print("\n[ BFS Search ]")
    bfs_path = bfs(start, goal)
    print(" -> ".join(bfs_path) if bfs_path else "No path found.")

    visualize_graph(graph, bfs_path)

if __name__ == "__main__":
    main()
