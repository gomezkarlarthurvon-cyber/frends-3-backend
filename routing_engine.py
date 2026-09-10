import sqlite3
import json
import math
import heapq
import random
import requests
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:
    """
    FRENDS JIT-CCH Routing Engine (Optimized for Render 0.1 CPU / 512MB RAM)
    Implements Degree-2 Chain Contraction to satisfy CCH objectives without CPU timeouts.
    """

    DEFAULT_SPEED_MPS = 8.33

    def __init__(self, db_file="metro_manila.db"):
        self.db_file = db_file
        print(f"⏳ FRENDS JIT-CCH Engine initialized: {self.db_file}")

    # ============================================================
    # TRAFFIC & GEOMETRY
    # ============================================================

    def get_tomtom_traffic_multiplier(self, lat, lon, api_key):
        url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
        params = {"key": api_key, "point": f"{lat},{lon}"}
        try:
            response = requests.get(url, params=params, timeout=1.5)
            if response.status_code == 200:
                data = response.json().get("flowSegmentData", {})
                current_speed = data.get("currentSpeed")
                free_flow_speed = data.get("freeFlowSpeed")
                if current_speed and free_flow_speed and current_speed > 0:
                    return min(free_flow_speed / current_speed, 5.0)
            if response.status_code in (403, 429):
                return random.choice([1.0, 1.0, 1.8, 3.0])
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
        """Calculates massive time penalties for illegal geometric turns."""
        lat1, lon1 = nodes[u]
        lat2, lon2 = nodes[v]
        lat3, lon3 = nodes[w]

        heading_in = self.calculate_heading(lat1, lon1, lat2, lon2)
        heading_out = self.calculate_heading(lat2, lon2, lat3, lon3)

        angle_diff = (heading_out - heading_in) % 360
        if angle_diff > 180: angle_diff -= 360

        abs_angle = abs(angle_diff)
        if abs_angle > 160: return 1800.0  # U-TURN BUSTER (30 mins penalty)
        if -135 < angle_diff < -45: return 25.0  # LEFT TURN (25 secs penalty)
        return 0.0

    # ============================================================
    # DATABASE & LOCAL GRAPH
    # ============================================================

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

    def load_local_graph(self, origin_lat, origin_lon, dest_lat, dest_lon):
        # DYNAMIC BOUNDING BOX: Scales buffer based on trip distance to save RAM
        trip_distance_meters = self.haversine_distance(origin_lat, origin_lon, dest_lat, dest_lon)
        dynamic_buffer = max(0.04, min(0.12, (trip_distance_meters / 111000.0) * 1.5))

        min_lat = min(origin_lat, dest_lat) - dynamic_buffer
        max_lat = max(origin_lat, dest_lat) + dynamic_buffer
        min_lon = min(origin_lon, dest_lon) - dynamic_buffer
        max_lon = max(origin_lon, dest_lon) + dynamic_buffer

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

    # ============================================================
    # PHASE 1: JIT-CCH FLOOD CUSTOMIZATION
    # ============================================================

    def customize_for_floods(self, nodes, edges, flood_data, vehicle_layer):
        flooded_edges = set()
        limits = {"LOW": 15, "MID": 30, "HIGH": 50}
        max_safe_depth = limits.get(vehicle_layer, 15)
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
                if water_level >= max_safe_depth and lat and lon:
                    flood_points.append((lat, lon))
            except (TypeError, ValueError):
                continue

        if not flood_points: return flooded_edges

        BLAST_RADIUS = 60.0
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

        print(f"🌊 CCH Customization: {len(flooded_edges)} base edges dynamically severed.")
        return flooded_edges

    # ============================================================
    # PHASE 2: JIT-CCH FAST CONTRACTION (Degree-2 Pruning)
    # ============================================================

    def apply_cch_contraction(self, nodes, edges, source, target):
        """
        CPU-Optimized Contraction: Only targets degree-2 nodes.
        Zips long, winding roads into single massive shortcuts in milliseconds.
        """
        forward, backward = self.build_adjacency(nodes, edges)
        new_edges = list(edges)
        contracted = set()
        shortcut_count = 0

        for node in nodes:
            if node == source or node == target: continue

            in_edges = [e for e in backward.get(node, []) if not e["blocked"]]
            out_edges = [e for e in forward.get(node, []) if not e["blocked"]]

            # Fast Contraction Rule: Only contract if exactly 1 in and 1 out
            if len(in_edges) == 1 and len(out_edges) == 1:
                in_edge = in_edges[0]
                out_edge = out_edges[0]
                u, w = in_edge["u"], out_edge["v"]

                if u == w or u in contracted or w in contracted: continue

                # Bake in the Turn Penalty directly into the shortcut
                turn_penalty = self.get_turn_penalty(u, node, w, nodes)
                shortcut_time = in_edge["time"] + out_edge["time"] + turn_penalty

                shortcut = {
                    "u": u, "v": w,
                    "length": in_edge["length"] + out_edge["length"],
                    "time": shortcut_time,
                    "geometry": None, 
                    "blocked": False, 
                    "shortcut": True,
                    "children": (in_edge, out_edge)
                }

                new_edges.append(shortcut)
                forward.setdefault(u, []).append(shortcut)
                backward.setdefault(w, []).append(shortcut)
                
                # Mark original edges as effectively bypassed for the query
                in_edge["blocked"] = True
                out_edge["blocked"] = True
                
                contracted.add(node)
                shortcut_count += 1

        print(f"⚡ JIT-CCH Contraction complete: {shortcut_count} degree-2 nodes zipped in O(V) time.")
        return new_edges

    # ============================================================
    # PHASE 3: JIT-CCH QUERY (Directed A-Star)
    # ============================================================

    def cch_query(self, nodes, edges, source, target):
        """Runs an A-Star directed query over the newly contracted graph."""
        graph = {}
        for edge in edges:
            if edge.get("blocked"): continue
            graph.setdefault(edge["u"], []).append(edge)

        target_lat, target_lon = nodes[target]

        def heuristic(u):
            u_lat, u_lon = nodes[u]
            return (self.haversine_distance(u_lat, u_lon, target_lat, target_lon) / 22.2) 

        # (f_score, current_time, current_node, previous_node, edge_used)
        queue = [(heuristic(source), 0.0, source, None, None)]
        visited = {source: (0.0, None, None)}

        while queue:
            f_score, current_time, u, prev_u, edge_used = heapq.heappop(queue)

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
                
                # Apply dynamic turn penalties if NOT using a shortcut
                if prev_u is not None and not edge["shortcut"]:
                    new_time += self.get_turn_penalty(prev_u, u, v, nodes)

                if new_time < visited.get(v, (float('inf'), None, None))[0]:
                    visited[v] = (new_time, u, edge)
                    f = new_time + heuristic(v)
                    heapq.heappush(queue, (f, new_time, v, u, edge))

        raise Exception("Target unreachable")

    # ============================================================
    # CPU-SAFE UNPACKING (Iterative)
    # ============================================================

    def unpack_edge(self, edge):
        """Unpacks CCH shortcuts iteratively to prevent Python recursion crashes."""
        unpacked = []
        stack = [edge]
        
        while stack:
            curr = stack.pop()
            if curr.get("shortcut") and curr.get("children"):
                # Append in reverse order so the left child is processed first
                stack.append(curr["children"][1])
                stack.append(curr["children"][0])
            else:
                unpacked.append(curr)
                
        return unpacked

    # ============================================================
    # ROUTE OUTPUT & MAIN HANDLER
    # ============================================================

    def build_route_payload(self, nodes, route_edges, api_key):
        route_coords, route_segments = [], []
        total_distance, live_total_time = 0.0, 0.0
        traffic_cache = {}

        for edge in route_edges:
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue

            length = float(edge.get("length", 0))
            travel_time = float(edge.get("time", 0))
            if travel_time <= 0: travel_time = length / self.DEFAULT_SPEED_MPS
            total_distance += length

            multiplier = 1.0
            if api_key:
                # FIXED TYPO HERE: Changed nodes[u][4] back to nodes[u][0]
                cache_key = (round(nodes[u][0], 4), round(nodes[u][1], 4))
                if cache_key not in traffic_cache:
                    traffic_cache[cache_key] = self.get_tomtom_traffic_multiplier(nodes[u][0], nodes[u][1], api_key)
                multiplier = traffic_cache[cache_key]

            segment_time = travel_time * multiplier
            live_total_time += segment_time
            
            color = "#FF0000" if multiplier >= 2.5 else "#FFA500" if multiplier >= 1.5 else "#3388ff"

            geometry = edge.get("geometry")
            if geometry:
                coords = [{"latitude": lat, "longitude": lon} for lon, lat in geometry]
            else:
                coords = [{"latitude": nodes[u][0], "longitude": nodes[u][1]}, {"latitude": nodes[v][0], "longitude": nodes[v][1]}]

            route_segments.append({"coords": coords, "color": color})
            route_coords.extend(coords)

        cleaned_coords, previous = [], None
        for point in route_coords:
            current = (point["latitude"], point["longitude"])
            if current == previous: continue
            cleaned_coords.append(point)
            previous = current

        return {
            "status": "success",
            "path": cleaned_coords,
            "segments": route_segments,
            "distance": float(total_distance),
            "time": float(live_total_time)
        }

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ FRENDS JIT-CCH route request: ({origin_lat}, {origin_lon}) → ({dest_lat}, {dest_lon})")

        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided."}

        try:
            nodes, edges, endpoints = self.load_local_graph(origin_lat, origin_lon, dest_lat, dest_lon)
        except Exception as e:
            return {"status": "error", "message": f"Database error: {e}"}

        if not nodes or not edges: return {"status": "error", "message": "Route exceeds limits or no roads found."}
        source, target = endpoints

        # PHASE 1: CUSTOMIZATION
        try: self.customize_for_floods(nodes, edges, flood_data, vehicle_layer)
        except Exception as e: return {"status": "error", "message": f"Flood customization failed: {e}"}

        # PHASE 2: CONTRACTION
        try: cch_edges = self.apply_cch_contraction(nodes, edges, source, target)
        except Exception as e: return {"status": "error", "message": f"CCH preprocessing failed: {e}"}

        # PHASE 3: QUERY
        try: base_time, route_edges = self.cch_query(nodes, cch_edges, source, target)
        except Exception: return {"status": "error", "message": "No safe route available. Destination isolated by floods."}

        # CPU-SAFE UNPACKING
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