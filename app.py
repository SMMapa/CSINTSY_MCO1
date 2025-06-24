
import heapq
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


def main():
    print("DLSU Food Finder")
    start = input("Enter starting point (e.g., A): ").strip().upper()
    goal = input("Enter goal (e.g., T): ").strip().upper()
    
    print("\n[ BFS Search ]")
    bfs_path = bfs(start, goal)
    print(" -> ".join(bfs_path) if bfs_path else "No path found.")

if __name__ == "__main__":
    main()
