from queue import PriorityQueue
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

start_node = 'S'
target_node = 'G'

def depth_first_search(graph, start, target):
    stack = [(start, [])]
    visited = set()

    while stack:
        node, path = stack.pop()
        if node == target:
            return path + [node]

        if node not in visited:
            visited.add(node)
            neighbors = list(graph[node].keys())  # Ensure consistent order for DFS
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

    return None  # Return None if no path is found

# Define the graph


# Start and target nodes
start_node = 'S'
target_node = 'G'

# Call depth_first_search function
dfs_path = depth_first_search(graph, start_node, target_node)

if dfs_path:
    print("DFS Path:", dfs_path)
else:
    print("No path found from 'S' to 'G' using DFS.")



def breadth_first_search(graph, start, target):
    queue = [(start, [start])]
    visited = set()  
    while queue:
        (node, path) = queue.pop(0)
        visited.add(node) 
        if node == target:
            return path
        neighbors = graph[node]
        for neighbor in neighbors:
            if neighbor not in path and neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))

bfs_path = breadth_first_search(graph, start_node, target_node)
print("BFS Path:", bfs_path)


def uniform_cost_search(graph, start, target):
    priority_queue = [(0, start, [start])]
    visited = set()  # To keep track of visited nodes
    while priority_queue:
        (cost, node, path) = priority_queue.pop(0)
        visited.add(node)  # Mark the current node as visited
        if node == target:
            return path
        neighbors = graph[node]
        for neighbor, neighbor_cost in neighbors.items():
            if neighbor not in path and neighbor not in visited:
                new_cost = cost + neighbor_cost
                priority_queue.append((new_cost, neighbor, path + [neighbor]))
                priority_queue.sort(key=lambda x: x[0])

ucs_path = uniform_cost_search(graph, start_node, target_node)
print("UCS Path:", ucs_path)


def greedy_search(graph, start, target, heuristics):
    priority_queue = [(heuristics[start], start, [start])]
    visited = set()  # To keep track of visited nodes
    while priority_queue:
        (_, node, path) = priority_queue.pop(0)
        visited.add(node)  # Mark the current node as visited
        if node == target:
            return path
        neighbors = graph[node]
        for neighbor in sorted(neighbors, key=lambda x: heuristics[x]):
            if neighbor not in path and neighbor not in visited:
                priority_queue.append((heuristics[neighbor], neighbor, path + [neighbor]))
                priority_queue.sort(key=lambda x: x[0])

greedy_path = greedy_search(graph, start_node, target_node, heuristics)
print("Greedy Search Path:", greedy_path)




def a_star_search(graph, start, target, heuristics):
    priority_queue = [(heuristics[start], 0, start, [start])]
    visited = set()  # To keep track of visited nodes
    while priority_queue:
        (_, cost, node, path) = priority_queue.pop(0)
        visited.add(node)  # Mark the current node as visited
        if node == target:
            return path
        neighbors = graph[node]
        for neighbor, neighbor_cost in sorted(neighbors.items(), key=lambda x: heuristics[x[0]]):
            if neighbor not in path and neighbor not in visited:
                new_cost = cost + neighbor_cost
                priority_queue.append((new_cost + heuristics[neighbor], new_cost, neighbor, path + [neighbor]))
                priority_queue.sort(key=lambda x: x[0])

a_star_path = a_star_search(graph, start_node, target_node, heuristics)

print("A* Search Path:", a_star_path)
