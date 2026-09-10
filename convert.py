import pickle
import networkx as nx

def prune_graph_for_render(input_file="metro_manila.pkl", output_file="metro_manila_lite.pkl"):
    print(f"Loading heavy graph from {input_file}... (This may take a moment)")
    with open(input_file, "rb") as f:
        G = pickle.load(f)

    print(f"Original graph loaded: {len(G.nodes)} nodes, {len(G.edges)} edges.")
    print("Pruning useless metadata to save RAM...")

    # 1. Prune Nodes (Keep ONLY x and y)
    for n, data in G.nodes(data=True):
        keys_to_delete = [k for k in data.keys() if k not in ['x', 'y']]
        for k in keys_to_delete:
            del data[k]

    # 2. Prune Edges (Keep ONLY length, travel_time, and geometry)
    for u, v, k, data in G.edges(keys=True, data=True):
        keys_to_delete = [k for k in data.keys() if k not in ['length', 'travel_time', 'geometry']]
        for k in keys_to_delete:
            del data[k]

    print("Saving highly compressed lite graph...")
    # Using HIGHEST_PROTOCOL compresses it even further
    with open(output_file, "wb") as f:
        pickle.dump(G, f, protocol=pickle.HIGHEST_PROTOCOL)

    print(f"✅ Success! Upload '{output_file}' to Render.")

if __name__ == "__main__":
    prune_graph_for_render()