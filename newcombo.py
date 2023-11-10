import numpy as np
from geopy.distance import geodesic
import folium
from folium import plugins

# node coordinates 
nodes = np.array([
    [34.154065, -84.213925],  # Node 0 - Return Flow Pump Station
    [34.146568, -84.192697],  # Node 1 
    [34.154557, -84.183972],  # Node 2 
    [34.157320, -84.176756],  # Node 3 
    [34.156553, -84.176347],  # Node 4 
    [34.158994, -84.169426],  # Node 5 
    [34.172135, -84.156470],  # Node 6 
    [34.177182, -84.145793],  # Node 7 
    [34.174252, -84.138848],  # Node 8 
    [34.179653, -84.133987],  # Node 9 
    [34.186777, -84.122250],  # Node 10 
    [34.184669, -84.105198],  # Node 11 
    [34.168472, -84.134327],  # Node 12 
    [34.159342, -84.145159],  # Node 13
    [34.156274, -84.200966],  # Node 14 
    [34.161183, -84.185670],  # Node 15 
    [34.165018, -84.170188],  # Node 16 
    [34.172919, -84.163792],  # Node 17 
    [34.180128, -84.152482],  # Node 18 
    [34.188410, -84.142285],  # Node 19 
    [34.196998, -84.135889],  # Node 20 
    [34.196998, -84.129863],  # Node 21 
    [34.194161, -84.118090],  # Node 22 
    [34.188103, -84.109190],  # Node 23 
    [34.182965, -84.133942],  # Node 24 
    [34.186263, -84.139133],  # Node 25 
    [34.156657, -84.129121],  # Node 26 
    [34.177674, -84.173155],  # Node 27 
    [34.144613, -84.154892],  # Node 28 
    [34.162257, -84.156839],  # Node 29 
    [34.170694, -84.160084],  # Node 30 
    [34.179744, -84.148125],  # Node 31 
    [34.183272, -84.083697],  # Node 32 
    [34.145687, -84.197257],  # Node 33 
    [34.137861, -84.180756],  # Node 34 
    [34.140009, -84.153131],  # Node 35 
    [34.196210, -84.096751],  # Node 36 - Lake Lanier Diffuser
])

# Initialize distance matrix
n = len(nodes)
dist_matrix = np.full((n, n), np.inf)  # Set all initial distances to infinity

# Define actual direct connections based on provided data
direct_connections = [
    (22, 36), (23, 36), (11, 36), (32, 36),
    (11, 32), (11, 23), (22, 23), (10, 23),
    (10, 22), (21, 20), (21, 22), (19, 25),
    (19, 31), (25, 24), (24, 9),  (9, 8),
    (8, 7),   (7, 31),  (18, 31), (18, 17),
    (6, 30),  (30, 17), (27, 17), (8, 13),
    (26, 13), (28, 13), (28, 35), (34, 28),
    (16, 17), (5, 16),  (3, 4),   (5, 4),
    (15, 16), (3, 2),   (2, 1),   (1, 33),
    (33, 14), (33, 0),  (14, 15), (14, 2),
    (34, 1),  (14, 0),  (20, 19), (24, 10),
    (29, 13), (29, 5),  (6, 29), (6,7)
]

# Set the distances for the direct connections and print them
print("Calculating direct distances between connected nodes:")
for (i, j) in direct_connections:
    direct_distance = geodesic(nodes[i], nodes[j]).kilometers
    print(f"Distance between node {i} and {j}: {direct_distance:.2f} km")
    dist_matrix[i, j] = direct_distance
    dist_matrix[j, i] = direct_distance 

# Dijkstra's algorithm initialization
unvisited = set(range(n))
distances = [np.inf] * n
distances[0] = 0  # Distance to starting node is 0
predecessors = [-1] * n

# Implementing Dijkstra's algorithm and printing the calculations
print("Running Dijkstra's algorithm...")
while unvisited:
    # Select the unvisited node with the smallest distance
    current = min(unvisited, key=lambda node: distances[node])
    # If distance is infinity, we are finished
    if distances[current] == np.inf:
        break
    print(f"Visiting node {current} with a distance of: {distances[current]:.2f} km")
    # Update the distances
    for neighbor in unvisited:
        new_dist = distances[current] + dist_matrix[current][neighbor]
        if new_dist < distances[neighbor]:
            distances[neighbor] = new_dist
            predecessors[neighbor] = current
            print(f"Updating distance for node {neighbor}: {new_dist:.2f} km")
    unvisited.remove(current)

# Path reconstruction
path = []
current_node = n - 1  # We want to end at the Lake Lanier Diffuser
while current_node != -1:
    path.insert(0, current_node)
    current_node = predecessors[current_node]

# Print the total distance of the path and the individual distances between nodes in the final path
print("The shortest path distance is:", distances[n - 1], "km")
print("Calculating distances for the final path between nodes...")
for i in range(len(path) - 1):
    start_node = path[i]
    end_node = path[i + 1]
    path_distance = dist_matrix[start_node][end_node]
    print(f"Distance from node {start_node} to node {end_node}: {path_distance:.2f} km")

# Creating a Folium map
m = folium.Map(location=nodes[0], zoom_start=12)

# Add lines for all direct connections
for i, j in direct_connections:
    folium.PolyLine([nodes[i], nodes[j]], color="gray", weight=1, opacity=0.5).add_to(m)

# Add a polyline for the actual path taken
folium.PolyLine([nodes[p] for p in path], color="blue", weight=2.5, opacity=0.8).add_to(m)

# Define a function that creates a custom HTML tooltip for a node
def custom_tooltip(text):
    return folium.Tooltip(f'<div style="font-size: 12pt; font-weight: bold">{text}</div>', permanent=True)

# Add markers for visited nodes on the path in green and unvisited in red
for i, node in enumerate(nodes):
    color = 'green' if i in path else 'red'
    tooltip_text = f'Node {i}'
    folium.CircleMarker(
        location=node,
        radius=5,
        color=color,
        fill=True,
        fill_opacity=0.7,
        tooltip=custom_tooltip(tooltip_text)
    ).add_to(m)

# Save the map to an HTML file
m.save('finalmap.html')