from django.http import JsonResponse
from rest_framework.decorators import api_view
import itertools
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


# //! Pohlepna pretraga po dubini - Agent Uki
def calculatePathUKI(prices):
    start_time = time.time()
    num_nodes = len(prices)
    start_node = 0
    visited = set()
    collected_gold = []

    def greedy_search(current_node):
        while True:
            # //* Filter out already visited nodes
            unvisited_neighbors = [node for node in range(num_nodes) if node not in visited]

            if not unvisited_neighbors:
                break  # //* No more unvisited neighbors

            # //* Choose the next gold with the minimum cost
            next_node = min(unvisited_neighbors, key=lambda node: prices[current_node][node])

            # //* If there are multiple nodes with the same cost, choose the one with the smaller id
            next_node = min(unvisited_neighbors, key=lambda node: (prices[current_node][node], node))

            # //* Collect gold from the chosen node
            collected_gold.append(next_node)

            # //* Mark the chosen node as visited
            visited.add(next_node)

            # //* Move to the next node
            current_node = next_node

        return [start_node] + collected_gold[1:]
    
    end_time = time.time()
    elapsed_time = end_time - start_time

    path = greedy_search(start_node)
    return {"elapsedTime": format_elapsed_time(elapsed_time), "minPath": path, "pathCost": calculate_total_cost(path, prices)}

# //! BRUTE FORCE SHORTEST PATH - Agent Jocke
def calculatePathJOCKE(prices):
    start_time = time.time()
    num_nodes = len(prices)
    nodes = list(range(1, num_nodes))  # //* Exclude node 0

    # //* Generate permutations of the remaining nodes
    remaining_permutations = itertools.permutations(nodes)

    # //* Initialize variables to store the minimum cost and corresponding path
    min_cost = float('inf')
    min_path = None

    # //* Iterate through remaining permutations and calculate total cost
    for remaining_permutation in remaining_permutations:
        # //* Include node 0 at the beginning and end
        path = [0] + list(remaining_permutation)
        cost = calculate_total_cost(path, prices)

        # //* Update minimum cost and path if a shorter path is found
        if cost < min_cost:
            min_cost = cost
            min_path = path

    end_time = time.time()
    elapsed_time = end_time - start_time

    return {"elapsedTime": format_elapsed_time(elapsed_time), "minPath": min_path, "pathCost": min_cost}

# //! BRANCH & BOUND - Agent Aki
def calculatePathAKI(prices):
    num_nodes = len(prices)
    start_node = 0
    visited = set()
    collected_gold = []
    start_time = time.time()

    def branch_and_bound(current_node, current_cost, current_path):
        nonlocal visited, collected_gold

        # //* Mark the current node as visited
        visited.add(current_node)

        # //* Add the current node to the path
        current_path.append(current_node)

        # //* Check if all nodes are visited
        if len(visited) == num_nodes:
            # //* Complete the path by returning to the start node
            current_path.append(start_node)
            # //* Update the collected gold path
            if not collected_gold or len(current_path) > len(collected_gold[-1]):
                collected_gold = [current_path]
            elif len(current_path) == len(collected_gold[-1]):
                # //* Choose the path with more collected gold
                collected_gold.append(current_path)
            return

        # //* Generate possible next nodes
        unvisited_neighbors = [node for node in range(num_nodes) if node not in visited]

        # //* Sort the neighbors by the cost and then by the node id
        sorted_neighbors = sorted(unvisited_neighbors, key=lambda node: (prices[current_node][node], node))

        for next_node in sorted_neighbors:
            next_cost = current_cost + prices[current_node][next_node]
            if not collected_gold or next_cost < len(collected_gold[-1]):
                # //* Continue searching only if there is a chance to find a shorter path
                branch_and_bound(next_node, next_cost, current_path.copy())

    # //* Start the branch and bound search
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

# //! A* - Agent Micko
def calculatePathMICKO(prices):
    # //* Get the number of nodes in the graph
    num_nodes = len(prices)
    # //* Define the starting node
    start_node = 0

    # //* Record the start time for performance measurement
    start_time = time.time()

    def minimum_spanning_tree_cost(visited_nodes, current_node):
        # //* Prim's algorithm to calculate the cost of the minimum spanning tree
        total_cost = 0
        # //* Initialize the priority queue with the current node and cost 0
        priority_queue = [(0, current_node)]

        # //* Continue until the priority queue is empty
        while priority_queue:
            # //* Pop the node with the minimum cost from the priority queue
            cost, node = heapq.heappop(priority_queue)

            # //* If the node is not visited, mark it as visited and update the total cost
            if node not in visited_nodes:
                visited_nodes.add(node)
                total_cost += cost

                # //* Add neighbors to the priority queue with their respective costs
                for neighbor, neighbor_cost in enumerate(prices[node]):
                    heapq.heappush(priority_queue, (neighbor_cost, neighbor))

        # //* Return the total cost of the minimum spanning tree
        return total_cost

    def astar_search():
        # //* Initialize the priority queue with the starting node and an empty path
        priority_queue = [(0, start_node, [start_node])]  # //* (f-value, current node, current path)

        # //* Continue until the priority queue is empty
        while priority_queue:
            # //* Pop the node with the minimum f-value from the priority queue
            f_value, current_node, current_path = heapq.heappop(priority_queue)

            # //* If all nodes are visited, complete the path without returning to the start node
            if len(set(current_path)) == num_nodes:
                return current_path

            # //* Explore neighbors that are not in the current path
            for neighbor in range(num_nodes):
                if neighbor not in current_path:
                    # //* Calculate g, h, and f values
                    g_value = sum(prices[current_path[i]][current_path[i+1]] for i in range(len(current_path) - 1))
                    h_value = minimum_spanning_tree_cost(set(current_path), neighbor)
                    f_value = g_value + h_value + prices[current_node][neighbor]

                    # //* Add the new path to the priority queue
                    heapq.heappush(priority_queue, (f_value, neighbor, current_path + [neighbor]))

    # //* Start the A* search
    path = astar_search()

    # //* Record the end time for performance measurement
    end_time = time.time()
    elapsed_time = end_time - start_time

    # //* Return the result containing elapsed time, path, and path cost
    return {
        "elapsedTime": format_elapsed_time(elapsed_time),
        "minPath": path,
        "pathCost": calculate_total_cost(path, prices)
    }

