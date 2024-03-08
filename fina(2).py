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

def dijkstra4(graphs, start, end, main_graph_index):

    distances = [{node: float('infinity') for node in graph} for graph in graphs]
    distances[main_graph_index][start] = 0
    previous_nodes = [{node: None for node in graph} for graph in graphs]
    priority_queues = [[(0, start)] for _ in range(len(graphs))]

    while any(priority_queues):
        for i, queue in enumerate(priority_queues):
            if queue:
                current_distance, current_node = heapq.heappop(queue)

                if current_distance > distances[i][current_node]:
                    continue

                for neighbor, weight in graphs[i][current_node].items():
                    distance = current_distance + weight

                    if distance < distances[i][neighbor]:
                        distances[i][neighbor] = distance
                        heapq.heappush(queue, (distance, neighbor))
                        previous_nodes[i][neighbor] = current_node

    merged_distances = {}
    for node in distances[main_graph_index]:
        merged_distances[node] = 0.6 * distances[main_graph_index][node] + \
                                  0.2 * distances[(main_graph_index + 1) % 3][node] + \
                                  0.2 * distances[(main_graph_index + 2) % 3][node]

    shortest_distance = merged_distances[end]
    shortest_path = []
    current = end
    while current is not None:
        shortest_path.append(current)
        current = previous_nodes[main_graph_index][current]
    shortest_path.reverse()

    graph_distances = []
    for i, graph in enumerate(graphs):
        path_distance = sum(graph[shortest_path[j]][shortest_path[j + 1]] for j in range(len(shortest_path) - 1))
        graph_distances.append(path_distance)

    return shortest_distance, shortest_path, graph_distances

graph = {
    'Shanghai':{'Xiamen':50,'Tokyo':90,'Beijing':60,'Wuhan':70,'HongKong':88,'Seoul':45},
    'Xiamen':{'Shanghai':50,'Shenzheng':30},
    'HongKong':{'Shanghai':88,'Tokyo':100,'Taipei':35,'Singapore':60,'Frankfurt':220,'Beijing':95,'Guangzhou':25,'Shenzheng':20},
    'Wuhan':{'Beijing':30,'Shanghai':30,'Chengdu':35,'Chongqin':28,'Shenzheng':35,'Xian':20},
    'Beijing':{'Shanghai':60,'Frankfurt':180,'HongKong':120,'Wuhan':40,'Xian':25,'Haerbin':30,'Seoul':50,'Moscow':200,'Dubai':165},
    'Tokyo':{'Shanghai':90,'Taipei':40,'HongKong':75},
    'Taipei':{'Tokyo':40,'HongKong':35,'Singapore':90},
    'Singapore':{'HongKong':60,'Taipei':90,'Frankfurt':175,'Bombay':120,'Dubai':100},
    'Frankfurt':{'Beijing':180,'HongKong':220,'Singapore':175,'Chengdu':170,'Moscow':35},
    'Shenzheng':{'Wuhan':30,'Xiamen':30,'Guangzhou':15,'HongKong':20},
    'Guangzhou':{'HongKong':25,'Shenzheng':15,'Chongqin':30},
    'Chongqin':{'Guangzhou':30,'Chengdu':10,'Wuhan':28},
    'Chengdu':{'Chongqin':10,'Frankfurt':170,'Wuhan':35},
    'Lasa':{'Xian':30,'Bombay':105},
    'Xian':{'Beijing':25,'Wuhan':20,'Lasa':30},
    'Bombay':{'Lasa':105,'Singapore':120},
    'Seoul':{'Beijing':50,'Shanghai':45},
    'Haerbin':{'Beijing':30},
    'Moscow':{'Beijing':200,'Frankfurt':35,'Dubai':120},
    'Dubai':{'Beijing':185,'Moscow':120,'Singapore':100},
}

print("""
Mode 1 , output the distance from chosen point to all the rest points
Mode 2 , output the distance between two chosen points
Mode 3 , can add pass through points in Mode 2
Mode 4 , automaticly best choice
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

if c == "4":
    # Price of path between nodes (One thousands of RMB per GB)
    Mode_4_graph_2 = {
        'Shanghai':{'Xiamen':30,'Tokyo':80,'Beijing':1000,'Wuhan':40,'HongKong':60,'Seoul':50},
        'Xiamen':{'Shanghai':30,'Shenzheng':30},
        'HongKong':{'Shanghai':60,'Tokyo':70,'Taipei':30,'Singapore':60,'Frankfurt':100,'Beijing':50,'Guangzhou':10,'Shenzheng':10},
        'Wuhan':{'Beijing':20,'Shanghai':40,'Chengdu':10,'Chongqin':20,'Shenzheng':20,'Xian':20},
        'Beijing':{'Shanghai':20,'Frankfurt':120,'HongKong':50,'Wuhan':20,'Xian':25,'Haerbin':20,'Seoul':50,'Moscow':180,'Dubai':100},
        'Tokyo':{'Shanghai':60,'Taipei':50,'HongKong':65},
        'Taipei':{'Tokyo':50,'HongKong':30,'Singapore':80},
        'Singapore':{'HongKong':60,'Taipei':80,'Frankfurt':100,'Bombay':80,'Dubai':90},
        'Frankfurt':{'Beijing':120,'HongKong':100,'Singapore':100,'Chengdu':180,'Moscow':40},
        'Shenzheng':{'Wuhan':20,'Xiamen':30,'Guangzhou':5,'HongKong':10},
        'Guangzhou':{'HongKong':10,'Shenzheng':5,'Chongqin':25},
        'Chongqin':{'Guangzhou':25,'Chengdu':5,'Wuhan':20},
        'Chengdu':{'Chongqin':5,'Frankfurt':180,'Wuhan':10},
        'Lasa':{'Xian':30,'Bombay':70},
        'Xian':{'Beijing':25,'Wuhan':20,'Lasa':30},
        'Bombay':{'Lasa':70,'Singapore':80},
        'Seoul':{'Beijing':50,'Shanghai':50},
        'Haerbin':{'Beijing':20},
        'Moscow':{'Beijing':180,'Frankfurt':40,'Dubai':60},
        'Dubai':{'Beijing':100,'Moscow':60,'Singapore':90},
    }
    # Risk factor from 1 to 100.(The lower the factor, the lower the risk.)
    Mode_4_graph_3 = {
        'Shanghai':{'Xiamen':10,'Tokyo':40,'Beijing':5,'Wuhan':20,'HongKong':10,'Seoul':50},
        'Xiamen':{'Shanghai':10,'Shenzheng':10},
        'HongKong':{'Shanghai':10,'Tokyo':40,'Taipei':30,'Singapore':30,'Frankfurt':60,'Beijing':40,'Guangzhou':5,'Shenzheng':5},
        'Wuhan':{'Beijing':10,'Shanghai':20,'Chengdu':5,'Chongqin':5,'Shenzheng':10,'Xian':5},
        'Beijing':{'Shanghai':5,'Frankfurt':50,'HongKong':40,'Wuhan':10,'Xian':5,'Haerbin':10,'Seoul':15,'Moscow':40,'Dubai':40},
        'Tokyo':{'Shanghai':40,'Taipei':50,'HongKong':40},
        'Taipei':{'Tokyo':40,'HongKong':35,'Singapore':80},
        'Singapore':{'HongKong':30,'Taipei':80,'Frankfurt':65,'Bombay':45,'Dubai':25},
        'Frankfurt':{'Beijing':50,'HongKong':60,'Singapore':65,'Chengdu':90,'Moscow':25},
        'Shenzheng':{'Wuhan':10,'Xiamen':10,'Guangzhou':15,'HongKong':5},
        'Guangzhou':{'HongKong':5,'Shenzheng':5,'Chongqin':10},
        'Chongqin':{'Guangzhou':10,'Chengdu':4,'Wuhan':5},
        'Chengdu':{'Chongqin':4,'Frankfurt':90,'Wuhan':5},
        'Lasa':{'Xian':40,'Bombay':40},
        'Xian':{'Beijing':5,'Wuhan':5,'Lasa':40},
        'Bombay':{'Lasa':40,'Singapore':45},
        'Seoul':{'Beijing':15,'Shanghai':50},
        'Haerbin':{'Beijing':10},
        'Moscow':{'Beijing':40,'Frankfurt':25,'Dubai':30},
        'Dubai':{'Beijing':40,'Moscow':30,'Singapore':25},
    }

    graphs = [graph, Mode_4_graph_2, Mode_4_graph_3]

    choice = input("Enter the index of the main graph (0, 1, or 2): ")

    if choice in ['0', '1', '2']:
        start_node = input("Start:")
        end_node = input("End:")
        shortest_distance, shortest_path, graph_distances = dijkstra4(graphs, start_node, end_node, int(choice))
        print(f"Shortest distance from {start_node} to {end_node}: {shortest_distance}")
        print(f"From {start_node} to {end_node} pass through {shortest_path}")
        for i, distance in enumerate(graph_distances):
            if i == 0:
                print(f"The delay of this path: {distance} ms ")
            if i == 1:
                print(f"The cost of this path: {distance} thousands of RMB pre GB")
            if i == 2:
                print(f"The stabilize parameter of this path: {int(distance / len(shortest_path))} / 100")
    else:
        print("Invalid input for the main graph index.")
