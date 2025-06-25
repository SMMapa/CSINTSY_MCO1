
import heapq
from collections import deque
from math import sqrt

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
    'U': ['K', 'J']
}

graph_astar = {
    'A': {'R': 6, 'T': 3},
    'B': {'Q': 11, 'R': 5},
    'C': {'P': 9, 'Q': 8, 'R': 7},
    'D': {'E': 3, 'F': 5, 'G': 10},
    'E': {'D': 3, 'O': 5, 'P': 5},
    'F': {'D': 5, 'H': 7, 'O': 5},
    'G': {'D': 10, 'H': 3, 'I': 2, 'L': 4, 'N': 5, 'O': 7},
    'H': {'F': 7, 'G': 3, 'I': 4},
    'I': {'G': 2, 'H': 4, 'L': 3, 'M': 7, 'N': 3},
    'J': {'J': 0, 'L': 2, 'M': 5, 'N': 8},
    'K': {'J': 5, 'M': 3, 'N': 5, 'U': 3},
    'L': {'G': 4, 'I': 3, 'J': 2},
    'M': {'I': 7, 'J': 5, 'K': 3},
    'N': {'G': 5, 'I': 3, 'J': 8, 'K': 5},
    'O': {'E': 5, 'F': 5, 'G': 7, 'P': 2, 'S': 3},
    'P': {'C': 9, 'E': 5, 'O': 2},
    'Q': {'B': 11, 'C': 8},
    'R': {'A': 6, 'B': 5, 'C': 7, 'S': 6},
    'S': {'O': 3, 'R': 6, 'T': 14},
    'T': {'A': 3, 'S': 14},
    'U': {'K': 3, 'J': 5},
}

# Coordinates
coordinates = {
    'A': (15, 4), 'B': (14, 4), 'C': (13, 3),
    'D': (8, 1), 'E': (7, 4), 'F': (6, 4),
    'G': (4, 4), 'H': (4, 1), 'I': (3, 4),
    'J': (2, 3), 'K': (1, 4), 'L': (3, 1),
    'M': (1, 7), 'N': (3, 7),
    'O': (7, 7), 'P': (9, 7),
    'Q': (10, 7), 'R': (12, 7),
    'S': (9, 8), 'T': (15, 7),
    'U': (0, 2)
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

# Heuristic
def euclidean(a, b):
    x1, y1 = coordinates[a]
    x2, y2 = coordinates[b]
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def astar(start, goal):
    open_set = [(euclidean(start, goal), 0, [start])]
    visited = {}

    while open_set:
        f, cost_so_far, path = heapq.heappop(open_set)
        current = path[-1]

        if current == goal:
            return path, cost_so_far

        if current in visited and visited[current] <= cost_so_far:
            continue
        visited[current] = cost_so_far

        for neighbor, edge_cost in graph_astar[current].items():
            new_cost = cost_so_far + edge_cost
            new_path = path + [neighbor]
            priority = new_cost + euclidean(neighbor, goal)
            heapq.heappush(open_set, (priority, new_cost, new_path))

    return None, float('inf')

# Cost breakdown for the final path
def breakdown(path, goal):
    segments = []
    total_cost = 0
    for i in range(len(path) - 1):
        a, b = path[i], path[i+1]
        edge_cost = graph_astar[a][b]
        total_cost += edge_cost
        heuristic = round(euclidean(b, goal))  # Heuristic of the NEXT node
        segments.append(f"{a} -> {b} ({edge_cost}) | h({b}) = {heuristic}")
    return segments, total_cost

def main():
    print("DLSU Food Finder")

    algo = input("[1] BFS [2] A*")
    print(algo)
    if not(algo == "1" or algo == "2"):
        print("Invalid option. Quitting")
        return
        
    
    start = input("Enter starting point (e.g., A): ").strip().upper()
    goal = input("Enter goal (e.g., T): ").strip().upper()
    if start not in graph or goal not in graph:
        print("Invalid start or goal.")
        return
    
    if algo == "1":
        print("\n[ BFS Search ]")
        bfs_path = bfs(start, goal)
        print(" -> ".join(bfs_path) if bfs_path else "No path found.")
    elif algo == "2":
        print("\n[ A* Search ]")
        path, total_cost = astar(start, goal)

        if path:
            print("\nPath found:")
            print(" -> ".join(path))
            segments, _ = breakdown(path, goal)
            print("\nStep-by-step breakdown:")
            for step in segments:
                print(" ", step)
            print(f"\nTotal cost: {total_cost}")
        else:
            print("No path found.")

if __name__ == "__main__":
    main()
