import osmnx as ox
import pickle

def build_full_metro_map():
    ox.settings.log_console = True
    ox.settings.use_cache = True

    # Using the strict OSM administrative name for the whole NCR
    place_name = "National Capital Region, Philippines"
    
    print(f"⏳ Downloading street network for: {place_name}...")
    print("Wait lang, medyo matagal siguro 'to hehe")

    # Download only drivable roads 
    G = ox.graph_from_place(place_name, network_type='drive')

    print(f"✅ Download complete! Loaded {len(G.nodes)} nodes and {len(G.edges)} edges.")
    print("📦 Packing into a Pickle file...")

    # Saving 
    with open("metro_manila_full.pkl", "wb") as f:
        pickle.dump(G, f)

    print("🎉 Success! Map saved as metro_manila_full.pkl")

if __name__ == "__main__":
    build_full_metro_map()