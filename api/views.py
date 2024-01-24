from django.http import JsonResponse
from rest_framework.decorators import api_view
import time
import heapq

@api_view(['POST'])
def getPath(request):
    agent_name = request.data.get('agent')
    prices = request.data.get('prices')

    if agent_name:
        agent_name = agent_name.lower()
        if agent_name == 'aki':
            result = calculatePathAKI(prices)
        elif agent_name == 'jocke':
            result = calculatePathJOCKE(prices)
        elif agent_name == 'micko':
            result = calculatePathMICKO(prices)
        elif agent_name == 'uki':
            result = calculatePathUKI(prices)
        else:
            result = {"error": "Unknown agent"}

        return JsonResponse(result, safe=False)
    else:
        return JsonResponse({"error": "Agent name not provided"}, status=400)


def calculate_total_cost(path, matrix):
    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += matrix[path[i]][path[i+1]]
    return total_cost

def format_elapsed_time(elapsed_time):
    return round(elapsed_time, 6)


# Pohlepna pretraga po dubini - Agent Uki
def calculatePathUKI(prices):
    start_time = time.time()
    num_nodes = len(prices)
    start_node = 0
    visited = set()
    collected_gold = []

    def greedy_search(current_node):
        while True:
            unvisited_neighbors = (node for node in range(num_nodes) if node not in visited)

            try:
                next_node = min(unvisited_neighbors, key=lambda node: (prices[current_node][node], node))
            except ValueError:
                break

            collected_gold.append(next_node)
            visited.add(next_node)
            current_node = next_node

        return [start_node] + collected_gold[1:]

    end_time = time.time()
    elapsed_time = end_time - start_time

    path = greedy_search(start_node)
    return {"elapsedTime": format_elapsed_time(elapsed_time), "minPath": path, "pathCost": calculate_total_cost(path, prices)}

# BRUTE FORCE SHORTEST PATH - Agent Jocke
def calculatePathJOCKE(prices):
    start_time = time.time()
    num_nodes = len(prices)
    nodes = list(range(1, num_nodes))  # Exclude node 0

    # Generate permutations of the remaining nodes
    remaining_permutations = generate_permutations(nodes)

    min_cost = float('inf')
    min_path = None

    for remaining_permutation in remaining_permutations:
        path = [0] + remaining_permutation
        cost = calculate_total_cost(path, prices)

        if cost < min_cost:
            min_cost = cost
            min_path = path

    end_time = time.time()
    elapsed_time = end_time - start_time

    return {"elapsedTime": format_elapsed_time(elapsed_time), "minPath": min_path, "pathCost": min_cost}

def generate_permutations(elements):
    if len(elements) == 0:
        yield []
    else:
        for i in range(len(elements)):
            rest = elements[:i] + elements[i+1:]
            for p in generate_permutations(rest):
                yield [elements[i]] + p

# BRANCH & BOUND - Agent Aki
def calculatePathAKI(prices):
    num_nodes = len(prices)
    start_node = 0
    visited = set()
    collected_gold = []
    start_time = time.time()

    def branch_and_bound(current_node, current_cost, current_path):
        nonlocal visited, collected_gold

        visited.add(current_node)
        current_path.append(current_node)

        if len(visited) == num_nodes:
            current_path.append(start_node)
            if not collected_gold or len(current_path) > len(collected_gold[-1]):
                collected_gold = [current_path]
            elif len(current_path) == len(collected_gold[-1]):
                collected_gold.append(current_path)
            return

        unvisited_neighbors = (node for node in range(num_nodes) if node not in visited)
        sorted_neighbors = sorted(unvisited_neighbors, key=lambda node: (prices[current_node][node], node))

        for next_node in sorted_neighbors:
            next_cost = current_cost + prices[current_node][next_node]
            if not collected_gold or next_cost < len(collected_gold[-1]):
                branch_and_bound(next_node, next_cost, current_path.copy())

    branch_and_bound(start_node, 0, [])

    end_time = time.time()
    elapsed_time = end_time - start_time

    min_path = collected_gold[0] if collected_gold else [start_node]
    path_cost = calculate_total_cost(min_path, prices)

    return {
        "elapsedTime": format_elapsed_time(elapsed_time),
        "minPath": min_path,
        "pathCost": path_cost
    }

# A* - Agent Micko
def calculatePathMICKO(prices):
    num_nodes = len(prices)
    start_node = 0
    start_time = time.time()

    def minimum_spanning_tree_cost(visited_nodes, current_node):
        total_cost = 0
        priority_queue = [(0, current_node)]

        while priority_queue:
            cost, node = heapq.heappop(priority_queue)

            if node not in visited_nodes:
                visited_nodes.add(node)
                total_cost += cost

                for neighbor, neighbor_cost in enumerate(prices[node]):
                    heapq.heappush(priority_queue, (neighbor_cost, neighbor))

        return total_cost

    def astar_search():
        priority_queue = [(0, start_node, [start_node])]  # (f-value, current node, current path)

        while priority_queue:
            f_value, current_node, current_path = heapq.heappop(priority_queue)

            if len(set(current_path)) == num_nodes:
                return current_path

            for neighbor in range(num_nodes):
                if neighbor not in current_path:
                    g_value = sum(prices[current_path[i]][current_path[i+1]] for i in range(len(current_path) - 1))
                    h_value = minimum_spanning_tree_cost(set(current_path), neighbor)
                    f_value = g_value + h_value + prices[current_node][neighbor]

                    heapq.heappush(priority_queue, (f_value, neighbor, current_path + [neighbor]))

    path = astar_search()

    end_time = time.time()
    elapsed_time = end_time - start_time

    return {
        "elapsedTime": format_elapsed_time(elapsed_time),
        "minPath": path,
        "pathCost": calculate_total_cost(path, prices)
    }
