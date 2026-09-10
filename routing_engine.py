import osmnx as ox
import networkx as nx
import requests
import random
import pickle
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:
    def __init__(self, graph_file="metro_manila_lite.pkl"):
        """Initializes the engine and loads the pre-built street network."""
        print(f"⏳ Initializing FRENDS Routing Engine...")
        try:
            print(f"Loading pre-processed map data from {graph_file}...")
            with open(graph_file, "rb") as f:
                self.graph = pickle.load(f)
            print("✅ Map loaded successfully from Pickle!")
            
            self.node_coords = {n: (data['y'], data['x']) for n, data in self.graph.nodes(data=True)}
            print(f"✅ Map loaded successfully! Loaded {len(self.graph.nodes)} intersection nodes.")
        except Exception as e:
            print(f"❌ Failed to load map data: {e}")
            self.graph = None

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
        r = 6371000 
        return c * r

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return self.haversine_distance(py, px, y1, x1)
        
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return self.haversine_distance(py, px, closest_y, closest_x)

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ Route requested: ({origin_lat}, {origin_lon}) -> ({dest_lat}, {dest_lon})")

        if self.graph is None:
            return {"status": "error", "message": "Backend Error: No valid OSMnx graph loaded."}

        # 1. FLOAT SAFETY CASTING (Prevents silent math crashes from JSON strings)
        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided to backend."}

        # 2. SPATIAL BOUNDING BOX
        buffer = 0.08 
        min_lat, max_lat = min(origin_lat, dest_lat) - buffer, max(origin_lat, dest_lat) + buffer
        min_lon, max_lon = min(origin_lon, dest_lon) - buffer, max(origin_lon, dest_lon) + buffer

        def filter_node_bbox(n):
            lat, lon = self.node_coords.get(n, (0,0))
            return min_lat <= lat <= max_lat and min_lon <= lon <= max_lon
            
        local_graph = nx.subgraph_view(self.graph, filter_node=filter_node_bbox)

        # 3. BUILD GEOMETRY-AWARE BLAST RADIUS FIREWALL
        flooded_edges_set = set()
        BLAST_RADIUS = 60  
        
        if flood_data:
            limits = {"LOW": 15, "Low (Sedan / Hatchback)": 15, "MID": 30, "Mid (SUV / Pick-up)": 30, "HIGH": 50, "High (Truck / Bus)": 50}
            max_safe_depth = limits.get(vehicle_layer, 15)
            flood_points = []
            
            for node_id, node_container in flood_data.items():
                if not isinstance(node_container, dict): continue
                water_level, lat, lng = 0, None, None
                
                push_keys = sorted([k for k in node_container.keys() if str(k).startswith('-')])
                
                if push_keys:
                    latest_key = push_keys[-1]
                    latest_data = node_container[latest_key]
                    if isinstance(latest_data, dict):
                        water_level = float(latest_data.get('waterLevel', latest_data.get('depth', 0)))
                        
                        # SAFE EXTRACT: Prevents float(None) crashes
                        lat_val = latest_data.get('lat', node_container.get('lat'))
                        lng_val = latest_data.get('lng', latest_data.get('lon', node_container.get('lng', node_container.get('lon'))))
                        if lat_val and lng_val:
                            lat, lng = float(lat_val), float(lng_val)
                else:
                    water_level = float(node_container.get('waterLevel', node_container.get('depth', 0)))
                    lat_val = node_container.get('lat')
                    lng_val = node_container.get('lng', node_container.get('lon'))
                    if lat_val and lng_val:
                        lat, lng = float(lat_val), float(lng_val)
                        
                if water_level >= max_safe_depth and lat and lng:
                    flood_points.append((lat, lng))
                    print(f"🌊 Flooded node detected at ({lat}, {lng}) - Depth: {water_level}cm")
            
            if flood_points:
                print(f"🌊 Scanning {local_graph.number_of_edges()} local road segments for blast radius overlap...")
                for u, v, k, data in local_graph.edges(keys=True, data=True):
                    is_flooded = False
                    pts = data.get('geometry', None)
                    if pts:
                        coords = list(pts.coords)
                    else:
                        node_u_data, node_v_data = self.graph.nodes[u], self.graph.nodes[v]
                        coords = [(node_u_data['x'], node_u_data['y']), (node_v_data['x'], node_v_data['y'])]
                    
                    for flood_lat, flood_lon in flood_points:
                        for i in range(len(coords) - 1):
                            lon1, lat1 = coords[i]
                            lon2, lat2 = coords[i+1]
                            dist = self.point_to_line_distance(flood_lon, flood_lat, lon1, lat1, lon2, lat2)
                            if dist <= BLAST_RADIUS:
                                is_flooded = True
                                break
                        if is_flooded: break
                            
                    if is_flooded:
                        flooded_edges_set.add((u, v))
                        flooded_edges_set.add((v, u))
                        
                print(f"🌊 Firewall complete: Blocked {len(flooded_edges_set)} directional road segments.")

        # 4. APPLY FILTER
        def filter_edge_strict(u, v, k):
            return (u, v) not in flooded_edges_set 
        
        safe_graph = nx.subgraph_view(local_graph, filter_edge=filter_edge_strict)

        # 5. SNAP & ROUTE
        try:
            orig_node = ox.nearest_nodes(self.graph, X=origin_lon, Y=origin_lat)
            dest_node = ox.nearest_nodes(self.graph, X=dest_lon, Y=dest_lat)
        except Exception as e:
            return {"status": "error", "message": f"Error snapping coordinates: {e}"}

        def get_edge_weight(u, v, data):
            w = data.get('current_weight')
            if w is None or w == float('inf'):
                w = data.get('travel_time', data.get('baseline_time', data.get('length', 1.0)))
            return float(w)

        path = None
        base_total_time = 0.0

        try:
            # Attempt routing within the bounding box
            base_total_time, path = nx.bidirectional_dijkstra(
                safe_graph, source=orig_node, target=dest_node, weight=get_edge_weight
            )
        except Exception as e:
            print(f"⚠️ Local subgraph failed ({e}). Complex one-ways likely cut off. Falling back to full graph...")
            try:
                # FALLBACK: If the bbox cut off the only U-turn (common near Taft), use the full graph
                base_total_time, path = nx.bidirectional_dijkstra(
                    self.graph, source=orig_node, target=dest_node, weight=get_edge_weight
                )
            except Exception as ex:
                return {"status": "error", "message": f"No safe route available. Destination isolated. ({ex})"}

        if not path or len(path) < 2: 
            return {"status": "error", "message": "Failed to generate a valid drivable path array."}

        # 6. COMPILE ROUTE PAYLOAD
        try:
            route_coords, route_segments = [], []
            total_distance, live_total_time = 0.0, 0.0
            current_multiplier = 1.0

            first_node = self.graph.nodes[path[0]]
            route_coords.append({"latitude": first_node['y'], "longitude": first_node['x']})

            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                node_u, node_v = self.graph.nodes[u], self.graph.nodes[v]

                edge_data = {}
                if self.graph.has_edge(u, v): edge_data = self.graph.get_edge_data(u, v)
                elif self.graph.has_edge(v, u): edge_data = self.graph.get_edge_data(v, u)
                else: continue

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
            return {"status": "error", "message": f"Failed compiling payload: {e}"}