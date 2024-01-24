from django.http import JsonResponse
from rest_framework.decorators import api_view
import heapq

@api_view(['POST'])
def get_path(request):
    agent_name = request.data.get('agent')
    prices = request.data.get('prices')

    if agent_name and agent_name.lower() in ['aki', 'jocke', 'micko', 'uki']:
        agent_name = agent_name.lower()

        if agent_name == 'uki':
            result = calculate_path_uki(prices)
        elif agent_name == 'jocke':
            result = calculate_path_jocke(prices)
        elif agent_name == 'aki':
            result = calculate_path_aki(prices)
        elif agent_name == 'micko':
            result = calculate_path_micko(prices)

        return JsonResponse(result, safe=False)
    else:
        return JsonResponse({"error": "Unknown or missing agent name"}, status=400)

def calculate_total_cost(path, matrix):
    return sum(matrix[path[i]][path[i+1]] for i in range(len(path) - 1))

# Agent Uki
def calculate_path_uki(prices):
    num_nodes = len(prices)
    start_node, visited = 0, set()
    collected_gold = []

    while len(visited) < num_nodes - 1:
        current_node = min((node for node in range(num_nodes) if node not in visited),
                           key=lambda node: (prices[start_node][node], node))

        visited.add(current_node)
        collected_gold.append(current_node)

    path = [start_node] + collected_gold
    return {"minPath": path, "pathCost": calculate_total_cost(path, prices)}

# Agent Jocke
def calculate_path_jocke(prices):
    num_nodes = len(prices)
    nodes = list(range(1, num_nodes))  # Exclude node 0

    min_cost, min_path = float('inf'), None

    for perm in generate_permutations(nodes):
        path = [0] + perm
        cost = calculate_total_cost(path, prices)

        if cost < min_cost:
            min_cost, min_path = cost, path

    return {"minPath": min_path, "pathCost": min_cost}

def generate_permutations(elements):
    if not elements:
        yield []
    else:
        for i in range(len(elements)):
            rest = elements[:i] + elements[i + 1:]
            for p in generate_permutations(rest):
                yield [elements[i]] + p

# Agent Aki
def calculate_path_aki(prices):
    num_nodes = len(prices)
    start_node, visited, collected_gold = 0, set(), []

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
    min_path = collected_gold[0] if collected_gold else [start_node]
    return {"minPath": min_path, "pathCost": calculate_total_cost(min_path, prices)}

# Agent Micko
def calculate_path_micko(prices):
    num_nodes = len(prices)
    start_node = 0

    def minimum_spanning_tree_cost(visited_nodes, current_node):
        total_cost, priority_queue = 0, [(0, current_node)]

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
                    g_value = sum(prices[current_path[i]][current_path[i + 1]] for i in range(len(current_path) - 1))
                    h_value = minimum_spanning_tree_cost(set(current_path), neighbor)
                    f_value = g_value + h_value + prices[current_node][neighbor]

                    heapq.heappush(priority_queue, (f_value, neighbor, current_path + [neighbor]))

    path = astar_search()
    return {"minPath": path, "pathCost": calculate_total_cost(path, prices)}
