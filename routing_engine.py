import sqlite3
import json
import math
import heapq
import random
import requests
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:

    DEFAULT_SPEED_MPS = 8.33

    def __init__(self, db_file="metro_manila.db"):
        self.db_file = db_file
        print(f" FRENDS JIT-CCH Engine initialized: {self.db_file}")

    def get_tomtom_traffic_multiplier(self, lat, lon, api_key):
        url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
        params = {"key": api_key, "point": f"{lat},{lon}"}
        try:
            response = requests.get(url, params=params, timeout=1.0)
            if response.status_code == 200:
                data = response.json().get("flowSegmentData", {})
                current_speed = data.get("currentSpeed")
                free_flow_speed = data.get("freeFlowSpeed")
                if current_speed and free_flow_speed and current_speed > 0:
                    return min(free_flow_speed / current_speed, 5.0)
            if response.status_code in (403, 429):
                return random.choice([1.0, 1.0, 1.35, 2.2])
        except Exception:
            pass
        return 1.0

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * asin(sqrt(a))
        return c * 6371000.0

    @staticmethod
    def point_to_line_distance(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return FRENDSRoutingEngine.haversine_distance(py, px, y1, x1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return FRENDSRoutingEngine.haversine_distance(py, px, closest_y, closest_x)

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlon))
        initial_bearing = math.atan2(x, y)
        return (math.degrees(initial_bearing) + 360) % 360

    def get_turn_penalty(self, u, v, w, nodes):
        lat1, lon1 = nodes[u]
        lat2, lon2 = nodes[v]
        lat3, lon3 = nodes[w]

        heading_in = self.calculate_heading(lat1, lon1, lat2, lon2)
        heading_out = self.calculate_heading(lat2, lon2, lat3, lon3)

        angle_diff = (heading_out - heading_in) % 360
        if angle_diff > 180: angle_diff -= 360

        abs_angle = abs(angle_diff)
        if abs_angle > 135: return 1800.0 
        if -130 < angle_diff < -65: return 20.0 
        return 0.0

    def nearest_node_sql(self, lat, lon, cursor):
        delta = 0.05
        cursor.execute(
            """
            SELECT id FROM nodes
            WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?
            ORDER BY ((lat - ?) * (lat - ?)) + ((lon - ?) * (lon - ?)) ASC LIMIT 1
            """,
            (lat - delta, lat + delta, lon - delta, lon + delta, lat, lat, lon, lon)
        )
        result = cursor.fetchone()
        if result: return result[0]

        cursor.execute(
            "SELECT id FROM nodes ORDER BY ((lat - ?) * (lat - ?)) + ((lon - ?) * (lon - ?)) ASC LIMIT 1",
            (lat, lat, lon, lon)
        )
        result = cursor.fetchone()
        return result[0] if result else None

    # Accepts dynamic buffer radius for the expansion strategy
    def load_local_graph(self, origin_lat, origin_lon, dest_lat, dest_lon, buffer_radius):
        min_lat = min(origin_lat, dest_lat) - buffer_radius
        max_lat = max(origin_lat, dest_lat) + buffer_radius
        min_lon = min(origin_lon, dest_lon) - buffer_radius
        max_lon = max(origin_lon, dest_lon) + buffer_radius

        conn = sqlite3.connect(self.db_file)
        cursor = conn.cursor()

        try:
            origin_node = self.nearest_node_sql(origin_lat, origin_lon, cursor)
            destination_node = self.nearest_node_sql(dest_lat, dest_lon, cursor)

            if origin_node is None or destination_node is None:
                return None, None, None

            cursor.execute(
                "SELECT id, lat, lon FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?",
                (min_lat, max_lat, min_lon, max_lon)
            )
            nodes = {node_id: (float(lat), float(lon)) for node_id, lat, lon in cursor}

            cursor.execute(
                """
                SELECT u, v, length, time, geometry FROM edges
                WHERE u IN (SELECT id FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?)
                AND v IN (SELECT id FROM nodes WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?)
                """,
                (min_lat, max_lat, min_lon, max_lon, min_lat, max_lat, min_lon, max_lon)
            )

            edges = []
            for u, v, length, time_value, geometry in cursor:
                if u not in nodes or v not in nodes: continue
                try: geometry_data = json.loads(geometry) if geometry else None
                except Exception: geometry_data = None

                edges.append({
                    "u": u, "v": v, 
                    "length": float(length or 0), 
                    "time": float(time_value or 0),
                    "geometry": geometry_data, 
                    "blocked": False, 
                    "shortcut": False, 
                    "children": None
                })

            return nodes, edges, (origin_node, destination_node)
        finally:
            cursor.close()
            conn.close()

    def build_adjacency(self, nodes, edges):
        forward, backward = {n: [] for n in nodes}, {n: [] for n in nodes}
        for edge in edges:
            u, v = edge["u"], edge["v"]
            forward[u].append(edge)
            backward[v].append(edge)
        return forward, backward

    def customize_for_floods(self, nodes, edges, flood_data, vehicle_layer):
        flooded_edges = set()
        limits = {"LOW": 15.24, "MID": 45.27, "HIGH": 60.96}
        max_safe_depth = limits.get(str(vehicle_layer).upper(), 25)
        flood_points = []

        if not flood_data: return flooded_edges

        for _, container in flood_data.items():
            if not isinstance(container, dict): continue
            push_keys = sorted([k for k in container if str(k).startswith("-")])
            latest = container[push_keys[-1]] if push_keys else container
            if not isinstance(latest, dict): continue

            try:
                water_level = float(latest.get("waterLevel", latest.get("depth", 0)))
                lat, lon = float(latest.get("lat", 0)), float(latest.get("lng", latest.get("lon", 0)))
                
                if water_level > max_safe_depth and lat and lon:
                    flood_points.append((lat, lon))
            except (TypeError, ValueError):
                continue

        if not flood_points: return flooded_edges

        # Tighter 15-meter blast radius. Safely blocks the flooded intersection 
        # WITHOUT bleeding over and destroying the safe parallel streets!
        BLAST_RADIUS = 15.0
        
        for edge in edges:
            if edge["shortcut"]: continue
            u, v = edge["u"], edge["v"]
            geometry = edge.get("geometry")
            if not geometry:
                if u not in nodes or v not in nodes: continue
                geometry = [(nodes[u][1], nodes[u][0]), (nodes[v][1], nodes[v][0])]

            blocked = False
            for flood_lat, flood_lon in flood_points:
                for i in range(len(geometry) - 1):
                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]
                    dist = self.point_to_line_distance(flood_lon, flood_lat, lon1, lat1, lon2, lat2)
                    if dist <= BLAST_RADIUS:
                        blocked = True
                        break
                if blocked: break

            if blocked:
                edge["blocked"] = True
                flooded_edges.add((u, v))

        if len(flood_points) > 0:
            print(f"🌊 HAZARD DETECTED: {len(flood_points)} floods over {max_safe_depth}cm. {len(flooded_edges)} roads cut.")

        return flooded_edges

    def apply_cch_contraction(self, nodes, edges, source, target):
        forward, backward = self.build_adjacency(nodes, edges)
        new_edges = list(edges)
        contracted = set()

        for node in nodes:
            if node == source or node == target: continue

            in_edges = [e for e in backward.get(node, []) if not e["blocked"]]
            out_edges = [e for e in forward.get(node, []) if not e["blocked"]]

            if len(in_edges) == 1 and len(out_edges) == 1:
                in_edge = in_edges[0]
                out_edge = out_edges[0]
                u, w = in_edge["u"], out_edge["v"]

                if u == w or u in contracted or w in contracted: continue

                shortcut_time = in_edge["time"] + out_edge["time"]
                geom_in = in_edge.get("geometry") or [[nodes[in_edge["u"]][1], nodes[in_edge["u"]][0]], [nodes[in_edge["v"]][1], nodes[in_edge["v"]][0]]]
                geom_out = out_edge.get("geometry") or [[nodes[out_edge["u"]][1], nodes[out_edge["u"]][0]], [nodes[out_edge["v"]][1], nodes[out_edge["v"]][0]]]
                
                shortcut = {
                    "u": u, "v": w,
                    "length": in_edge["length"] + out_edge["length"],
                    "time": shortcut_time,
                    "geometry": geom_in + geom_out[1:], 
                    "blocked": False, 
                    "shortcut": True,
                    "children": (in_edge, out_edge),
                    "first_v": in_edge.get("first_v", in_edge["v"]),
                    "last_u": out_edge.get("last_u", out_edge["u"])
                }

                new_edges.append(shortcut)
                forward.setdefault(u, []).append(shortcut)
                backward.setdefault(w, []).append(shortcut)
                
                in_edge["blocked"] = True
                out_edge["blocked"] = True
                contracted.add(node)

        return new_edges

    def cch_query(self, nodes, edges, source, target):
        graph = {}
        for edge in edges:
            if edge.get("blocked"): continue
            graph.setdefault(edge["u"], []).append(edge)

        target_lat, target_lon = nodes[target]

        def heuristic(u):
            u_lat, u_lon = nodes[u]
            return (self.haversine_distance(u_lat, u_lon, target_lat, target_lon) / 22.2) 

        queue = [(heuristic(source), 0.0, source, None, None)]
        visited = {source: (0.0, None, None)}

        while queue:
            f_score, current_time, u, prev_u, prev_edge = heapq.heappop(queue)

            if u == target:
                path_edges = []
                curr = u
                while curr != source:
                    _, parent, e = visited[curr]
                    path_edges.append(e)
                    curr = parent
                return current_time, path_edges[::-1]

            if current_time > visited.get(u, (float('inf'), None, None))[0]:
                continue

            for edge in graph.get(u, []):
                v = edge["v"]
                new_time = current_time + edge["time"]
                
                if prev_u is not None:
                    immediate_v = edge.get("first_v", edge["v"])
                    immediate_prev_u = prev_edge.get("last_u", prev_edge["u"]) if prev_edge else prev_u
                    new_time += self.get_turn_penalty(immediate_prev_u, u, immediate_v, nodes)

                if new_time < visited.get(v, (float('inf'), None, None))[0]:
                    visited[v] = (new_time, u, edge)
                    f = new_time + heuristic(v)
                    heapq.heappush(queue, (f, new_time, v, u, edge))

        raise Exception("Target unreachable")

    def unpack_edge(self, edge):
        unpacked = []
        stack = [edge]
        while stack:
            curr = stack.pop()
            if curr.get("shortcut") and curr.get("children"):
                stack.append(curr["children"][1])
                stack.append(curr["children"][0])
            else:
                unpacked.append(curr)
        return unpacked

    def build_osrm_segments(self, osrm_coords, api_key, is_city=False):
        if not osrm_coords or len(osrm_coords) < 2:
            return [], 0.0

        # Dynamic Traffic Chunker. Caps TomTom requests at 10 to completely eliminate Render server timeouts
        total_dist = 0
        for i in range(len(osrm_coords)-1):
            total_dist += self.haversine_distance(osrm_coords[i][1], osrm_coords[i][0], osrm_coords[i+1][1], osrm_coords[i+1][0])
        chunk_size = max(350.0, total_dist / 10.0) 

        segments = []
        traffic_cache = {}
        chunk = []
        accumulated_dist = 0.0
        total_duration = 0.0
        base_speed = 8.33 if is_city else 16.6 

        for i in range(len(osrm_coords) - 1):
            lon1, lat1 = osrm_coords[i]
            lon2, lat2 = osrm_coords[i+1]
            dist = self.haversine_distance(lat1, lon1, lat2, lon2)
            accumulated_dist += dist

            chunk.append({"latitude": lat1, "longitude": lon1})

            if accumulated_dist >= chunk_size or i == len(osrm_coords) - 2:
                chunk.append({"latitude": lat2, "longitude": lon2})
                mid_lat = (chunk[0]["latitude"] + chunk[-1]["latitude"]) / 2.0
                mid_lon = (chunk[0]["longitude"] + chunk[-1]["longitude"]) / 2.0
                cache_key = (round(mid_lat, 3), round(mid_lon, 3))

                multiplier = 1.0
                if api_key and api_key != "undefined":
                    if cache_key not in traffic_cache:
                        traffic_cache[cache_key] = self.get_tomtom_traffic_multiplier(mid_lat, mid_lon, api_key)
                    multiplier = traffic_cache[cache_key]

                if multiplier >= 2.2: color = "#EF4444"
                elif multiplier >= 1.25: color = "#F59E0B"
                else: color = "#00A3FF"

                segments.append({"coords": list(chunk), "color": color})
                total_duration += (accumulated_dist / base_speed) * multiplier

                chunk = [{"latitude": lat2, "longitude": lon2}]
                accumulated_dist = 0.0

        return segments, total_duration

    def build_route_payload(self, nodes, route_edges, api_key):
        route_coords = []
        total_distance = 0.0

        for edge in route_edges:
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue
            
            total_distance += float(edge.get("length", 0))

            geometry = edge.get("geometry")
            if geometry:
                coords = [{"latitude": lat, "longitude": lon} for lon, lat in geometry]
            else:
                coords = [{"latitude": nodes[u][0], "longitude": nodes[u][1]}, {"latitude": nodes[v][0], "longitude": nodes[v][1]}]

            route_coords.extend(coords)

        cleaned_coords, previous = [], None
        for point in route_coords:
            current = (point["latitude"], point["longitude"])
            if current == previous: continue
            cleaned_coords.append(point)
            previous = current

        osrm_format_coords = [[p["longitude"], p["latitude"]] for p in cleaned_coords]
        route_segments, live_total_time = self.build_osrm_segments(osrm_format_coords, api_key, is_city=True)

        return {
            "status": "success",
            "path": cleaned_coords,
            "segments": route_segments,
            "distance": float(total_distance),
            "time": float(live_total_time)
        }
        
    def fetch_osrm_fallback(self, origin_lat, origin_lon, dest_lat, dest_lon, api_key):
        print(f"⚠️ Out of Bounds: Delegating {origin_lat},{origin_lon} -> {dest_lat},{dest_lon} to OSRM Fallback...")
        # Standard OSRM public API endpoint for driving routes
        url = f"http://router.project-osrm.org/route/v1/driving/{origin_lon},{origin_lat};{dest_lon},{dest_lat}?overview=full&geometries=geojson"
        
        try:
            response = requests.get(url, timeout=5.0)
            if response.status_code == 200:
                data = response.json()
                if data.get("code") == "Ok":
                    route = data["routes"][0]
                    total_distance = route["distance"]
                    
                    # OSRM returns GeoJSON format: [longitude, latitude]
                    coordinates = route["geometry"]["coordinates"]
                    
                    # Reformat for the frontend MapSection.jsx
                    cleaned_coords = [{"latitude": p[1], "longitude": p[0]} for p in coordinates]
                    
                    # Pass the OSRM coordinates through the existing chunker 
                    # to seamlessly apply TomTom traffic coloring to the fallback route
                    route_segments, live_total_time = self.build_osrm_segments(coordinates, api_key, is_city=False)
                    
                    return {
                        "status": "success",
                        "path": cleaned_coords,
                        "segments": route_segments,
                        "distance": float(total_distance),
                        "time": float(live_total_time)
                    }
        except Exception as e:
            print(f"🚨 OSRM Fallback encountered an error: {e}")
            
        return {"status": "error", "message": "Location is outside coverage area and fallback routing failed."}

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided."}

        # 🌟 THE MASTER FIX: DYNAMIC EXPANSION STRATEGY
        # Starts with a blazing-fast 1.5km grid. If floods block the detour, it automatically expands 
        # up to an 8km radius to find side-streets, preventing both "No Path" errors AND "Timeouts"!
        buffer_stages = [0.015, 0.035, 0.07] 
        route_edges = []
        base_time = 0
        
        for attempt, buf in enumerate(buffer_stages):
            try:
                nodes, edges, endpoints = self.load_local_graph(origin_lat, origin_lon, dest_lat, dest_lon, buf)
            except Exception as e:
                return {"status": "error", "message": f"Database error: {e}"}

            if not nodes or not edges: 
                if attempt == len(buffer_stages) - 1:
                    # 🔥 GRACEFUL DEGRADATION: Trigger OSRM instead of crashing
                    return self.fetch_osrm_fallback(origin_lat, origin_lon, dest_lat, dest_lon, api_key)
                continue
                
            source, target = endpoints

            try: 
                self.customize_for_floods(nodes, edges, flood_data, vehicle_layer)
                cch_edges = self.apply_cch_contraction(nodes, edges, source, target)
                base_time, route_edges = self.cch_query(nodes, cch_edges, source, target)
                
                # If we succeed, break out of the expansion loop!
                print(f"✅ Safe detour successfully mapped using radius stage {attempt+1} ({buf})")
                break 
                
            except Exception:
                # Target unreachable (isolated by the current grid size)
                if attempt == len(buffer_stages) - 1:
                    return {"status": "error", "message": "No safe route available. Destination isolated by floods."}
                print(f"⚠️ Detour blocked by floods. Automatically expanding search grid to radius {buffer_stages[attempt+1]}...")

        # Build payload using the successfully detoured edges
        unpacked = []
        for edge in route_edges: unpacked.extend(self.unpack_edge(edge))

        unique_edges, seen = [], set()
        for edge in unpacked:
            key = (edge["u"], edge["v"], id(edge))
            if key in seen: continue
            seen.add(key)
            unique_edges.append(edge)

        try:
            result = self.build_route_payload(nodes, unique_edges, api_key)
            if not result["path"] or len(result["path"]) < 2:
                return {"status": "error", "message": "Failed to generate a valid route."}
            if not api_key: result["time"] = float(base_time)
            return result
        except Exception as e:
            return {"status": "error", "message": f"Failed compiling payload: {e}"}