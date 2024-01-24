from django.http import JsonResponse
from rest_framework.decorators import api_view
import heapq
from itertools import permutations

@api_view(['POST'])
def get_path(request):
    agent_name = request.data.get('agent')
    prices = request.data.get('prices')

    if not agent_name or agent_name.lower() not in ['aki', 'jocke', 'micko', 'uki']:
        return JsonResponse({"error": "Unknown or missing agent name"}, status=400)

    agent_name = agent_name.lower()
    result = calculate_path(agent_name, prices)
    return JsonResponse(result, safe=False)

def calculate_path(agent_name, prices):
    num_nodes, start_node = len(prices), 0

    if agent_name == 'uki':
        return calculate_path_uki(prices)
    elif agent_name == 'jocke':
        return calculate_path_jocke(prices)
    elif agent_name == 'aki':
        return calculate_path_aki(prices)
    elif agent_name == 'micko':
        return calculate_path_micko(prices)

def calculate_total_cost(path, matrix):
    return sum(matrix[path[i]][path[i + 1]] for i in range(len(path) - 1))

def calculate_path_uki(prices):
    visited, collected_gold = {0}, []

    while len(visited) < len(prices):
        current_node = min(set(range(len(prices))) - visited, key=lambda node: (prices[0][node], node))
        visited.add(current_node)
        collected_gold.append(current_node)

    path = [0] + collected_gold
    return {"minPath": path, "pathCost": calculate_total_cost(path, prices)}

def calculate_path_jocke(prices):
    nodes = list(range(1, len(prices)))
    min_path = min(
        (calculate_total_cost([0] + perm, prices), [0] + perm) for perm in permutations(nodes)
    )

    return {"minPath": min_path[1], "pathCost": min_path[0]}

def calculate_path_aki(prices):
    visited, collected_gold = {0}, []

    def branch_and_bound(current_node, current_cost, current_path):
        nonlocal visited, collected_gold

        visited.add(current_node)
        current_path.append(current_node)

        if len(visited) == len(prices):
            current_path.append(0)
            if not collected_gold or len(current_path) > len(collected_gold[-1]):
                collected_gold.clear()
                collected_gold.append(current_path)
            elif len(current_path) == len(collected_gold[-1]):
                collected_gold.append(current_path)
            return

        unvisited_neighbors = set(range(len(prices))) - visited
        sorted_neighbors = sorted(unvisited_neighbors, key=lambda node: (prices[current_node][node], node))

        for next_node in sorted_neighbors:
            next_cost = current_cost + prices[current_node][next_node]
            if not collected_gold or next_cost < len(collected_gold[-1]):
                branch_and_bound(next_node, next_cost, current_path.copy())

    branch_and_bound(0, 0, [])
    min_path = collected_gold[0] if collected_gold else [0]
    return {"minPath": min_path, "pathCost": calculate_total_cost(min_path, prices)}

def calculate_path_micko(prices):
    return astar_search(prices)

def astar_search(prices):
    num_nodes = len(prices)
    priority_queue = [(0, 0, [0])]  # (f-value, current node, current path)

    while priority_queue:
        f_value, current_node, current_path = heapq.heappop(priority_queue)

        if len(set(current_path)) == num_nodes:
            return {"minPath": current_path, "pathCost": calculate_total_cost(current_path, prices)}

        for neighbor in set(range(num_nodes)) - set(current_path):
            g_value = sum(prices[current_path[i]][current_path[i + 1]] for i in range(len(current_path) - 1))
            h_value = minimum_spanning_tree_cost(set(current_path), neighbor, prices)
            f_value = g_value + h_value + prices[current_node][neighbor]

            heapq.heappush(priority_queue, (f_value, neighbor, current_path + [neighbor]))

def minimum_spanning_tree_cost(visited_nodes, current_node, prices):
    total_cost, priority_queue = 0, [(0, current_node)]

    while priority_queue:
        cost, node = heapq.heappop(priority_queue)

        if node not in visited_nodes:
            visited_nodes.add(node)
            total_cost += cost
            heapq.heappush(priority_queue, *((neighbor_cost, neighbor) for neighbor, neighbor_cost in enumerate(prices[node])))

    return total_cost
