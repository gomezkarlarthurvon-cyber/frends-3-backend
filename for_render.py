import osmnx as ox

print("Downloading precise FRENDS 3.0 node map...")
center_point = (14.5704, 120.9915) 

G = ox.graph_from_point(
    center_point, 
    dist=1500, 
    network_type="drive"
)

print("Saving lightweight graph to file...")
ox.save_graphml(G, "metro_manila.graphml")
print("Done! Ready to push.")