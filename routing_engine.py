import numpy as np
import scipy.sparse as sp
from scipy.sparse.csgraph import shortest_path
import requests
import random
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:
    def __init__(self, npz_file="metro_manila.npz"):
        print(f"⏳ Initializing Flat-Array Routing Engine...")
        try:
            data = np.load(npz_file)
            self.indptr = data['indptr']
            self.indices = data['indices']
            self.baseline_weights = data['weights']
            self.lats = data['lats']
            self.lons = data['lons']
            self.ranks = data['ranks']
            
            # Construct a SciPy CSR matrix wrapper for structural reference
            self.n_nodes = len(self.lats)
            self.csr_matrix = sp.csr_matrix(
                (self.baseline_weights, self.indices, self.indptr), 
                shape=(self.n_nodes, self.n_nodes)
            )
            print(f"✅ Loaded flat-array map successfully! {self.n_nodes} nodes active.")
        except Exception as e:
            print(f"❌ Failed to load flat map: {e}")
            self.csr_matrix = None

    def get_tomtom_traffic_multiplier(self, lat, lon, api_key):
        url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
        params = {'key': api_key, 'point': f"{lat},{lon}"}
        try:
            response = requests.get(url, params=params, timeout=1.0) 
            if response.status_code == 200:
                flow_data = response.json().get('flowSegmentData', {})
                current_speed = flow_data.get('currentSpeed')
                free_flow_speed = flow_data.get('freeFlowSpeed')
                if current_speed and free_flow_speed and current_speed > 0:
                    return min(free_flow_speed / current_speed, 5.0) 
        except Exception:
            pass
        return random.choice([1.0, 1.0, 1.8, 3.0])

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        return 2 * asin(sqrt(a)) * 6371000

    def find_nearest_node(self, lat, lon):
        """Vectorized spatial lookup avoiding OSMnx spatial index overhead."""
        distances = (self.lats - lat)**2 + (self.lons - lon)**2
        return int(np.argmin(distances))

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ Flat-Array Route requested: ({origin_lat}, {origin_lon}) -> ({dest_lat}, {dest_lon})")

        if self.csr_matrix is None:
            return {"status": "error", "message": "Backend Error: No valid CSR map loaded."}

        orig_node = self.find_nearest_node(origin_lat, origin_lon)
        dest_node = self.find_nearest_node(dest_lat, dest_lon)

        # Clone weights array to apply dynamic flood/traffic penalties in-place without reallocating memory
        active_weights = self.baseline_weights.copy()

        # Apply Flood Firewall directly on flat edge indices
        if flood_data:
            limits = {"LOW": 15, "MID": 30, "HIGH": 50}
            max_safe_depth = limits.get(vehicle_layer, 15)
            
            for node_id, node_container in flood_data.items():
                if not isinstance(node_container, dict): continue
                push_keys = sorted([k for k in node_container.keys() if str(k).startswith('-')])
                latest_data = node_container[push_keys[-1]] if push_keys else node_container
                
                water_level = float(latest_data.get('waterLevel', latest_data.get('depth', 0)))
                flat_node = self.find_nearest_node(float(latest_data.get('lat', 0)), float(latest_data.get('lng', latest_data.get('lon', 0))))
                
                if water_level >= max_safe_depth:
                    # Invalidate outgoing and incoming edges for flooded nodes in the flat weight array
                    start_idx = self.indptr[flat_node]
                    end_idx = self.indptr[flat_node + 1]
                    active_weights[start_idx:end_idx] = np.inf

        # Build dynamic SciPy matrix with modified weights
        dynamic_csr = sp.csr_matrix((active_weights, self.indices, self.indptr), shape=(self.n_nodes, self.n_nodes))

        # Run optimized Dijkstra via SciPy's C-backend
        try:
            distances, predecessors = shortest_path(
                dynamic_csr, 
                directed=True, 
                indices=orig_node, 
                return_predecessors=True
            )
            
            total_time = distances[dest_node]
            if total_time == np.inf:
                return {"status": "error", "message": f"No safe route available for {vehicle_layer} clearance. Path blocked by flood."}

            # Reconstruct path from predecessors array
            path = []
            curr = dest_node
            while curr != -9999 and curr != orig_node:
                path.append(curr)
                curr = predecessors[curr]
            path.append(orig_node)
            path.reverse()

        except Exception as e:
            return {"status": "error", "message": f"Routing computation failed: {e}"}

        # Compile payload
        route_coords, route_segments = [], []
        total_distance = 0.0
        
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            lat_u, lon_u = self.lats[u], self.lons[u]
            lat_v, lon_v = self.lats[v], self.lons[v]
            
            seg_dist = self.haversine_distance(lat_u, lon_u, lat_v, lon_v)
            total_distance += seg_dist
            
            route_coords.append({"latitude": lat_u, "longitude": lon_u})
            segment_coords = [{"latitude": lat_u, "longitude": lon_u}, {"latitude": lat_v, "longitude": lon_v}]
            route_segments.append({"coords": segment_coords, "color": "#3388ff"})
            
        route_coords.append({"latitude": self.lats[dest_node], "longitude": self.lons[dest_node]})

        return {
            "status": "success",
            "path": route_coords,       
            "segments": route_segments, 
            "distance": float(total_distance),
            "time": float(total_time)
        }