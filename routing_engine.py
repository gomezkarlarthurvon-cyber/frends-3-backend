import networkx as nx
import requests
import random
import sqlite3
import json
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:
    def __init__(self, db_file="metro_manila.db"):
        self.db_file = db_file
        print(f"⏳ FRENDS JIT Routing Engine Initialized linked to {self.db_file}.")

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
            elif response.status_code in [403, 429]:
                return random.choice([1.0, 1.0, 1.8, 3.0]) 
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
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0: return self.haversine_distance(py, px, y1, x1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))
        return self.haversine_distance(py, px, y1 + t * dy, x1 + t * dx)

    def nearest_node_sql(self, lat, lon, cursor):
        cursor.execute('''
            SELECT id FROM nodes 
            ORDER BY ((lat - ?) * (lat - ?) + (lon - ?) * (lon - ?)) ASC LIMIT 1
        ''', (lat, lat, lon, lon))
        result = cursor.fetchone()
        return result[0] if result else None

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ JIT Route requested: ({origin_lat}, {origin_lon}) -> ({dest_lat}, {dest_lon})")

        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided."}

        buffer = 0.08 
        min_lat, max_lat = min(origin_lat, dest_lat) - buffer, max(origin_lat, dest_lat) + buffer
        min_lon, max_lon = min(origin_lon, dest_lon) - buffer, max(origin_lon, dest_lon) + buffer

        try:
            conn = sqlite3.connect(self.db_file)
            c = conn.cursor()

            orig_node = self.nearest_node_sql(origin_lat, origin_lon, c)
            dest_node = self.nearest_node_sql(dest_lat, dest_lon, c)

            if not orig_node or not dest_node:
                conn.close()
                return {"status": "error", "message": "Origin or Destination is completely off the map grid."}

            c.execute('''
                SELECT u, v, length, time, geometry 
                FROM edges 
                WHERE u IN (SELECT id FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?)
                  AND v IN (SELECT id FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?)
            ''', (min_lat, max_lat, min_lon, max_lon, min_lat, max_lat, min_lon, max_lon))
            
            edges = c.fetchall()
            
            c.execute('SELECT id, lat, lon FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?', 
                      (min_lat, max_lat, min_lon, max_lon))
            nodes_data = {row[0]: (row[1], row[2]) for row in c.fetchall()}
            conn.close()
            
        except Exception as e:
            return {"status": "error", "message": f"Database error: {e}"}

        if not edges:
            return {"status": "error", "message": "Route exceeds bounding box limits or no roads found."}

        local_graph = nx.DiGraph()
        for edge in edges:
            u, v, length, time, geom_str = edge
            geom = json.loads(geom_str) if geom_str else []
            local_graph.add_edge(u, v, length=length, travel_time=time, geometry=geom)
        
        flooded_edges_set = set()
        BLAST_RADIUS = 60  
        
        if flood_data:
            limits = {"LOW": 15, "MID": 30, "HIGH": 50}
            max_safe_depth = limits.get(vehicle_layer, 15)
            flood_points = []
            
            for node_id, node_container in flood_data.items():
                if not isinstance(node_container, dict): continue
                water_level, lat, lng = 0, None, None
                
                push_keys = sorted([k for k in node_container.keys() if str(k).startswith('-')])
                if push_keys:
                    latest_data = node_container[push_keys[-1]]
                    if isinstance(latest_data, dict):
                        water_level = float(latest_data.get('waterLevel', latest_data.get('depth', 0)))
                        lat, lng = float(latest_data.get('lat', 0)), float(latest_data.get('lng', latest_data.get('lon', 0)))
                else:
                    water_level = float(node_container.get('waterLevel', node_container.get('depth', 0)))
                    lat, lng = float(node_container.get('lat', 0)), float(node_container.get('lng', node_container.get('lon', 0)))
                        
                if water_level >= max_safe_depth and lat and lng:
                    flood_points.append((lat, lng))
            
            if flood_points:
                for u, v, data in local_graph.edges(data=True):
                    is_flooded = False
                    coords = data.get('geometry', [])
                    if not coords:
                        if u in nodes_data and v in nodes_data:
                            coords = [(nodes_data[u][1], nodes_data[u][0]), (nodes_data[v][1], nodes_data[v][0])]
                    
                    for flood_lat, flood_lon in flood_points:
                        for i in range(len(coords) - 1):
                            lon1, lat1 = coords[i]
                            lon2, lat2 = coords[i+1]
                            if self.point_to_line_distance(flood_lon, flood_lat, lon1, lat1, lon2, lat2) <= BLAST_RADIUS:
                                is_flooded = True
                                break
                        if is_flooded: break
                            
                    if is_flooded:
                        flooded_edges_set.add((u, v))
                        flooded_edges_set.add((v, u))

        def filter_edge_strict(u, v):
            return (u, v) not in flooded_edges_set 
        safe_graph = nx.subgraph_view(local_graph, filter_edge=filter_edge_strict)

        def get_edge_weight(u, v, data):
            # PHYSICS FALLBACK: If time is 0, estimate based on length / ~30kmh
            t = data.get('travel_time')
            t = float(t) if t is not None else 0.0
            if t <= 0:
                t = float(data.get('length', 1.0)) / 8.33
            return t

        try:
            base_total_time, path = nx.bidirectional_dijkstra(safe_graph, source=orig_node, target=dest_node, weight=get_edge_weight)
        except Exception as e:
            return {"status": "error", "message": "No safe route available. Destination isolated by traffic/flood bounds."}

        if not path or len(path) < 2: 
            return {"status": "error", "message": "Failed to generate a valid drivable path array."}

        try:
            route_coords, route_segments = [], []
            total_distance, live_total_time = 0.0, 0.0
            current_multiplier = 1.0

            first_node_data = nodes_data.get(path[0])
            route_coords.append({"latitude": first_node_data[0], "longitude": first_node_data[1]})

            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                node_u_data, node_v_data = nodes_data.get(u), nodes_data.get(v)
                edge_attrs = local_graph.get_edge_data(u, v)
                
                seg_length = float(edge_attrs.get('length', 0.0))
                total_distance += seg_length
                
                # PHYSICS FALLBACK FOR PAYLOAD
                raw_time = edge_attrs.get('travel_time')
                seg_time = float(raw_time) if raw_time is not None else 0.0
                if seg_time <= 0:
                    seg_time = seg_length / 8.33

                if api_key and (i % 8 == 0):
                    current_multiplier = self.get_tomtom_traffic_multiplier(node_u_data[0], node_u_data[1], api_key)

                live_total_time += (seg_time * current_multiplier)
                segment_color = "#FF0000" if current_multiplier >= 2.5 else "#FFA500" if current_multiplier >= 1.5 else "#3388ff"

                segment_coords = []
                coords = edge_attrs.get('geometry', [])
                if coords:
                    for lon, lat in coords:
                        segment_coords.append({"latitude": lat, "longitude": lon})
                else:
                    segment_coords.extend([{"latitude": node_u_data[0], "longitude": node_u_data[1]}, {"latitude": node_v_data[0], "longitude": node_v_data[1]}])

                route_segments.append({"coords": segment_coords, "color": segment_color})
                route_coords.append({"latitude": node_v_data[0], "longitude": node_v_data[1]})

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