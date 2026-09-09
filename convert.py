import osmnx as ox
import networkx as nx
import numpy as np

def build_full_metro_map():
    ox.settings.log_console = True
    ox.settings.use_cache = True

    place_name = "National Capital Region, Philippines"
    
    print(f"⏳ Downloading street network for: {place_name}...")
    print("Wait lang, medyo matagal siguro 'to hehe")

    G = ox.graph_from_place(place_name, network_type='drive')

    print(f"✅ Download complete! Loaded {len(G.nodes)} nodes and {len(G.edges)} edges.")
    print("📦 Extracting topology and packing into flat NumPy arrays...")

    nodes = list(G.nodes())
    node_to_idx = {node: i for i, node in enumerate(nodes)}
    n_nodes = len(nodes)

    lats = np.zeros(n_nodes, dtype=np.float32)
    lons = np.zeros(n_nodes, dtype=np.float32)
    ranks = np.zeros(n_nodes, dtype=np.int32)

    for i, node in enumerate(nodes):
        data = G.nodes[node]
        lats[i] = data.get('y', 0.0)
        lons[i] = data.get('x', 0.0)
        ranks[i] = data.get('ch_rank', i)

    indptr = [0]
    indices = []
    baseline_weights = []

    for u in nodes:
        neighbors = list(G.successors(u)) if G.is_directed() else list(G.neighbors(u))
        for v in neighbors:
            edge_data = G.get_edge_data(u, v)
            w = min(d.get('travel_time', d.get('length', 1.0)) for d in edge_data.values())
            indices.append(node_to_idx[v])
            baseline_weights.append(float(w))
        indptr.append(len(indices))

    np.savez_compressed(
        "metro_manila.npz",
        indptr=np.array(indptr, dtype=np.int32),
        indices=np.array(indices, dtype=np.int32),
        weights=np.array(baseline_weights, dtype=np.float32),
        lats=lats,
        lons=lons,
        ranks=ranks
    )

    print("🎉 Success! Optimized flat-array map saved as metro_manila.npz")

if __name__ == "__main__":
    build_full_metro_map()