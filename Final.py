import heapq

def dijkstra1(graph,start):
     distances = {node: float('infinity') for node in graph}
     distances[start] = 0
     priority_queue = [(0, start)]

     while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush (priority_queue, (distance, neighbor))
     return distances

def dijkstra2(graph, start, end):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {node: None for node in graph}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                previous_nodes[neighbor] = current_node

    path = []
    current = end
    while current is not None:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()

    return distances[end] , path 


def dijkstra3(graph, start, end, via_points=None):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    visited = set()
    previous_nodes = {}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_node in visited:
            continue
        
        visited.add(current_node)
        
        if current_node == end:
            break
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                previous_nodes[neighbor] = current_node
    
    if via_points:
        path = []
        previous_node = end
        while previous_node != start:
            path.append(previous_node)
            previous_node = previous_nodes[previous_node]
        path.append(start)
        path.reverse()
        return distances[end], path

graph = {
    'A':{'B':5,'C':6},
    'B':{'A':5,'E':6,'D':4},
    'C':{'A':6,'E':7,'F':6},
    'D':{'B':4,'E':3,'G':12},
    'E':{'B':6,'C':7,'D':3,'G':6,'F':5},
    'F':{'C':6,'E':5,'G':8},
    'G':{'D':12,'E':6,'F':8}
}

print("""
Mode 1 , output the distance from chosen point to all the rest points
Mode 2 , output the distance between two chosen points
Mode 3 , can add pass through point in Mode 2
Choose the mode : 
""")

c = input("Mode:")

if c == '1':
    start_node = input(" Start: ")
    result = dijkstra1(graph, start_node)
    print(f"Shortest distance from {start_node}:{result}")

if c == '2':
    start_node = input("Start:")
    end_node = input("End:")
    shortest_distance , shortest_path = dijkstra2(graph, start_node, end_node)
    print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
    print(f"From {start_node} to {end_node} pass through {shortest_path}")

if c == "3":
    start_node = input("Start:")
    end_node = input("End:")
    mid_node_number = int(input("Node number:"))
    via_nodes = []
    for i in range(mid_node_number):
        via_nodes.append(input(f'Enter Node_{i+1}:'))
        print(via_nodes)

    total_distance, path = 0, []

    previous_node = start_node
    for via_node in via_nodes:
        distance, sub_path = dijkstra3(graph, previous_node, via_node, via_nodes)
        total_distance += distance
        path.extend(sub_path[:-1])
        previous_node = via_node

    distance_to_end, sub_path = dijkstra3(graph, previous_node, end_node, via_nodes)
    total_distance += distance_to_end
    path.extend(sub_path)

    print("Total distance:", total_distance)
    print(f"Path:", path)
