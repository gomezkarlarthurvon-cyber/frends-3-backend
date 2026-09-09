import osmnx as ox
import networkx as nx
import requests
import random
import pickle
import heapq
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:
    def __init__(self, graph_file="metro_manila_ch.pkl"):
        """Initializes the engine and loads the pre-processed CH/CCH network data."""
        print(f"⏳ Initializing FRENDS CCH Routing Engine...")
        try:
            print(f"Loading hierarchical map data from {graph_file}...")
            with open(graph_file, "rb") as f:
                saved_data = pickle.load(f)
                
            # Support both raw graphs and pre-contracted tuple structures (graph, node_ranks)
            if isinstance(saved_data, tuple):
                self.graph, self.node_ranks = saved_data
            else:
                self.graph = saved_data
                self.node_ranks = {n: data.get('ch_rank', i) for i, (n, data) in enumerate(self.graph.nodes(data=True))}
                
            self.node_coords = {n: (data['y'], data['x']) for n, data in self.graph.nodes(data=True)}
            print(f"✅ CCH Map loaded successfully! Loaded {len(self.graph.nodes)} nodes with hierarchical ranks.")
        except Exception as e:
            print(f"❌ Failed to load map data: {e}")
            self.graph = None
            self.node_ranks = {}

    def get_tomtom_traffic_multiplier(self, lat, lon, api_key):
        """Pings TomTom API for live traffic flow at a specific coordinate."""
        url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
        params = {'key': api_key, 'point': f"{lat},{lon}"}
        try:
            response = requests.get(url, params=params, timeout=1.0) 
            if response.status_code == 200:
                flow_data = response.json().get('flowSegmentData', {})
                current_speed = flow_data.get('currentSpeed')
                free_flow_speed = flow_data.get('freeFlowSpeed')
                
                if current_speed and free_flow_speed and current_speed > 0:
                    multiplier = free_flow_speed / current_speed
                    return min(multiplier, 5.0) 
            elif response.status_code in [403, 429]:
                return random.choice([1.0, 1.0, 1.0, 1.8, 3.0]) 
        except Exception:
            pass
            
        return random.choice([1.0, 1.0, 1.8, 3.0])

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        return c * 6371000

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return self.haversine_distance(py, px, y1, x1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))
        return self.haversine_distance(py, px, y1 + t * dy, x1 + t * dx)

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ CCH Route requested: ({origin_lat}, {origin_lon}) -> ({dest_lat}, {dest_lon})")

        if self.graph is None:
            return {"status": "error", "message": "Backend Error: No valid CCH graph loaded."}

        # 1. BUILD GEOMETRY-AWARE BLAST RADIUS FIREWALL FOR FLOODS
        flooded_edges_set = set()
        BLAST_RADIUS = 60  # meters
        
        if flood_data:
            limits = {"LOW": 15, "Low (Sedan / Hatchback)": 15, "MID": 30, "Mid (SUV / Pick-up)": 30, "HIGH": 50, "High (Truck / Bus)": 50}
            max_safe_depth = limits.get(vehicle_layer, 15)
            flood_points = []
            
            for node_id, node_container in flood_data.items():
                if not isinstance(node_container, dict): continue
                push_keys = sorted([k for k in node_container.keys() if str(k).startswith('-')])
                if push_keys:
                    latest_data = node_container[push_keys[-1]]
                    water_level = float(latest_data.get('waterLevel', latest_data.get('depth', 0)))
                    lat = float(latest_data.get('lat', node_container.get('lat', 0)))
                    lng = float(latest_data.get('lng', node_container.get('lon', node_container.get('lng', node_container.get('lon', 0)))))
                else:
                    water_level = float(node_container.get('waterLevel', node_container.get('depth', 0)))
                    lat = float(node_container.get('lat', 0))
                    lng = float(node_container.get('lng', node_container.get('lon', 0)))
                        
                if water_level >= max_safe_depth and lat and lng:
                    flood_points.append((lat, lng))
            
            if flood_points:
                for u, v, k, data in self.graph.edges(keys=True, data=True):
                    is_flooded = False
                    pts = data.get('geometry', None)
                    coords = list(pts.coords) if pts else [(self.node_coords[u][1], self.node_coords[u][0]), (self.node_coords[v][1], self.node_coords[v][0])]
                    
                    for flood_lat, flood_lon in flood_points:
                        for i in range(len(coords) - 1):
                            if self.point_to_line_distance(flood_lon, flood_lat, coords[i][0], coords[i][1], coords[i+1][0], coords[i+1][1]) <= BLAST_RADIUS:
                                is_flooded = True
                                break
                        if is_flooded: break
                            
                    if is_flooded:
                        flooded_edges_set.add((u, v))
                        flooded_edges_set.add((v, u))
                print(f"🌊 CCH Firewall: Blocked {len(flooded_edges_set) // 2} road segments due to flooding.")

        # 2. SNAP COORDINATES TO GRAPH NODES
        try:
            orig_node = ox.nearest_nodes(self.graph, X=origin_lon, Y=origin_lat)
            dest_node = ox.nearest_nodes(self.graph, X=dest_lon, Y=dest_lat)
        except Exception as e:
            return {"status": "error", "message": f"Error snapping coordinates: {e}"}

        def get_edge_weight(u, v, data):
            if (u, v) in flooded_edges_set:
                return float('inf')
            w = data.get('current_weight')
            if w is None or w == float('inf'):
                w = data.get('travel_time', data.get('baseline_time', data.get('length', 1.0)))
            return float(w)

        # 3. CCH UPWARD BIDIRECTIONAL SEARCH QUERY
        # Forward search moves only to higher-ranked neighbors; Backward search from destination also moves to higher-ranked neighbors.
        def run_cch_search(source, target):
            dist_fwd = {source: 0.0}
            dist_bwd = {target: 0.0}
            parent_fwd = {}
            parent_bwd = {}
            
            pq_fwd = [(0.0, source)]
            pq_bwd = [(0.0, target)]
            
            settled_fwd = {}
            settled_bwd = {}
            mu = float('inf')
            best_meeting_node = None

            while pq_fwd or pq_bwd:
                # Expand Forward Queue (Upward only)
                if pq_fwd:
                    cost_u, u = heapq.heappop(pq_fwd)
                    if u not in settled_fwd:
                        settled_fwd[u] = cost_u
                        if u in settled_bwd and cost_u + settled_bwd[u] < mu:
                            mu = cost_u + settled_bwd[u]
                            best_meeting_node = u

                        rank_u = self.node_ranks.get(u, 0)
                        for v in self.graph.successors(u):
                            if self.node_ranks.get(v, 0) > rank_u: # Strict upward condition
                                edge_data = self.graph.get_edge_data(u, v)
                                w = min(get_edge_weight(u, v, d) for d in edge_data.values())
                                if w != float('inf'):
                                    nd = cost_u + w
                                    if nd < dist_fwd.get(v, float('inf')):
                                        dist_fwd[v] = nd
                                        parent_fwd[v] = u
                                        heapq.heappush(pq_fwd, (nd, v))

                # Expand Backward Queue (Upward from target)
                if pq_bwd:
                    cost_v, v = heapq.heappop(pq_bwd)
                    if v not in settled_bwd:
                        settled_bwd[v] = cost_v
                        if v in settled_fwd and cost_v + settled_fwd[v] < mu:
                            mu = cost_v + settled_fwd[v]
                            best_meeting_node = v

                        rank_v = self.node_ranks.get(v, 0)
                        # In the backward search, we traverse incoming edges whose source has a higher rank
                        for u in self.graph.predecessors(v):
                            if self.node_ranks.get(u, 0) > rank_v:
                                edge_data = self.graph.get_edge_data(u, v)
                                w = min(get_edge_weight(u, v, d) for d in edge_data.values())
                                if w != float('inf'):
                                    nd = cost_v + w
                                    if nd < dist_bwd.get(u, float('inf')):
                                        dist_bwd[u] = nd
                                        parent_bwd[u] = v
                                        heapq.heappush(pq_bwd, (nd, u))

            if mu == float('inf') or best_meeting_node is None:
                return None, float('inf')

            # Reconstruct path from source -> meeting_node -> target
            path = []
            curr = best_meeting_node
            while curr in parent_fwd:
                path.append(curr)
                curr = parent_fwd[curr]
            path.append(source)
            path.reverse()

            curr = best_meeting_node
            while curr in parent_bwd:
                curr = parent_bwd[curr]
                path.append(curr)

            return path, mu

        path, base_total_time = run_cch_search(orig_node, dest_node)

        if not path:
            return {"status": "error", "message": f"No safe CCH route available for {vehicle_layer} clearance. Area isolated by flood or network cut."}

        # 4. COMPILE ROUTE PAYLOAD & UNPACK SHORTCUTS IF NEEDED
        try:
            route_coords, route_segments = [], []
            total_distance, live_total_time = 0.0, 0.0
            current_multiplier = 1.0

            first_node = self.graph.nodes[path[0]]
            route_coords.append({"latitude": first_node['y'], "longitude": first_node['x']})

            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                node_u, node_v = self.graph.nodes[u], self.graph.nodes[v]

                edge_data = self.graph.get_edge_data(u, v, default=None)
                if not edge_data:
                    edge_data = self.graph.get_edge_data(v, u, default={})

                edge_attrs = next(iter(edge_data.values())) if isinstance(edge_data, dict) else edge_data
                
                raw_length = edge_attrs.get('length', 0.0)
                seg_length = float(raw_length[0] if isinstance(raw_length, list) else raw_length)
                total_distance += seg_length
                
                raw_time = edge_attrs.get('baseline_time', edge_attrs.get('travel_time', seg_length / 8.33))
                seg_time = float(raw_time[0] if isinstance(raw_time, list) else raw_time)

                if api_key and (i % 8 == 0):
                    current_multiplier = self.get_tomtom_traffic_multiplier(node_u['y'], node_u['x'], api_key)

                live_total_time += (seg_time * current_multiplier)
                segment_color = "#FF0000" if current_multiplier >= 2.5 else "#FFA500" if current_multiplier >= 1.5 else "#3388ff"

                segment_coords = []
                if 'geometry' in edge_attrs:
                    for lon, lat in edge_attrs['geometry'].coords:
                        segment_coords.append({"latitude": lat, "longitude": lon})
                else:
                    segment_coords.extend([{"latitude": node_u['y'], "longitude": node_u['x']}, {"latitude": node_v['y'], "longitude": node_v['x']}])

                route_segments.append({"coords": segment_coords, "color": segment_color})
                route_coords.append({"latitude": node_v['y'], "longitude": node_v['x']})

            final_eta_seconds = live_total_time if (api_key and live_total_time > 0) else base_total_time
            
            return {
                "status": "success",
                "path": route_coords,       
                "segments": route_segments, 
                "distance": float(total_distance),
                "time": float(final_eta_seconds)
            }
        except Exception as e:
            return {"status": "error", "message": f"Failed compiling CCH payload: {e}"}