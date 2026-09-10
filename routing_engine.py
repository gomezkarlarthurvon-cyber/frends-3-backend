import sqlite3
import json
import math
import heapq
import random
import requests
from math import radians, cos, sin, asin, sqrt


class FRENDSRoutingEngine:
    """
    FRENDS Memory-Efficient CCH Routing Engine
    Designed for Render's 512 MB RAM limit.
    """

    DEFAULT_SPEED_MPS = 8.33

    def __init__(self, db_file="metro_manila.db"):
        self.db_file = db_file
        print(f"⏳ FRENDS Memory-Efficient CCH Engine initialized: {self.db_file}")

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

        # U-TURN BUSTER: Adds 30 minutes to force the graph to reject this shortcut
        if abs_angle > 160: return 1800.0
        # HARD LEFT TURN: Adds 25 seconds for crossing traffic
        if -135 < angle_diff < -45: return 25.0
        
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
        # FIXED: Expanded buffer to 0.10 to comfortably route long distances (e.g., Cavite to Manila)
        buffer = 0.10

        min_lat = min(origin_lat, dest_lat) - buffer
        max_lat = max(origin_lat, dest_lat) + buffer
        min_lon = min(origin_lon, dest_lon) - buffer
        max_lon = max(origin_lon, dest_lon) + buffer

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
        forward, backward, edge_lookup = {n: [] for n in nodes}, {n: [] for n in nodes}, {}
        for edge in edges:
            u, v = edge["u"], edge["v"]
            forward[u].append(edge)
            backward[v].append(edge)
            edge_lookup[(u, v)] = edge
        return forward, backward, edge_lookup

    def calculate_order(self, nodes, forward, backward, source, target):
        importance = []
        for node in nodes:
            if node == source or node == target: continue
            degree = len(forward.get(node, ())) + len(backward.get(node, ()))
            importance.append((degree, node))
        importance.sort(key=lambda x: x[0])
        return [node for _, node in importance]

    # ============================================================
    # CCH CONTRACTION WITH BAKED-IN TURN PENALTIES
    # ============================================================

    def witness_search(self, node, incoming, outgoing, contracted, nodes):
        if not incoming or not outgoing: return []
        shortcuts = []

        for in_edge in incoming:
            u = in_edge["u"]
            if u in contracted: continue

            for out_edge in outgoing:
                w = out_edge["v"]
                if w in contracted or u == w: continue

                # FIXED: Mathematically bakes the Turn Penalty into the CCH Shortcut!
                turn_penalty = self.get_turn_penalty(u, node, w, nodes)
                shortcut_time = in_edge["time"] + out_edge["time"] + turn_penalty

                shortcuts.append((u, w, shortcut_time, in_edge, out_edge))

        return shortcuts

    def apply_cch_contraction(self, nodes, edges, source, target):
        forward, backward, edge_lookup = self.build_adjacency(nodes, edges)
        order = self.calculate_order(nodes, forward, backward, source, target)
        rank = {node: i for i, node in enumerate(order)}
        rank[source], rank[target] = len(order) + 1, len(order) + 2

        contracted, new_edges, shortcut_count = set(), list(edges), 0

        for node in order:
            incoming = [e for e in backward.get(node, ()) if not e["blocked"] and e["u"] not in contracted]
            outgoing = [e for e in forward.get(node, ()) if not e["blocked"] and e["v"] not in contracted]

            candidates = self.witness_search(node, incoming, outgoing, contracted, nodes)

            for u, w, shortcut_time, in_edge, out_edge in candidates:
                existing = edge_lookup.get((u, w))
                if existing is not None and existing["time"] <= shortcut_time: continue

                shortcut = {
                    "u": u, "v": w,
                    "length": in_edge["length"] + out_edge["length"],
                    "time": shortcut_time,
                    "geometry": None, "blocked": False, "shortcut": True,
                    "children": (in_edge, out_edge)
                }
                new_edges.append(shortcut)
                edge_lookup[(u, w)] = shortcut
                forward.setdefault(u, []).append(shortcut)
                backward.setdefault(w, []).append(shortcut)
                shortcut_count += 1

            contracted.add(node)

        print(f"⚡ Local CCH contraction complete: {len(order)} nodes contracted, {shortcut_count} shortcuts created.")
        return new_edges, rank

    # ============================================================
    # FLOOD CUSTOMIZATION
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

        for edge in edges:
            if not edge["shortcut"]: continue
            children = edge.get("children")
            if not children: continue
            if children[0].get("blocked") or children[1].get("blocked"):
                edge["blocked"] = True

        print(f"🌊 CCH flood customization: {len(flooded_edges)} base edges blocked.")
        return flooded_edges

    # ============================================================
    # CCH BIDIRECTIONAL QUERY
    # ============================================================

    def cch_query(self, nodes, edges, rank, source, target):
        forward, backward = {}, {}
        for edge in edges:
            if edge.get("blocked"): continue
            u, v = edge["u"], edge["v"]
            ru, rv = rank.get(u, 0), rank.get(v, 0)
            if ru <= rv: forward.setdefault(u, []).append(edge)
            if rv <= ru: backward.setdefault(v, []).append(edge)

        forward_dist, forward_parent = {source: 0.0}, {}
        pq_forward = [(0.0, source)]

        backward_dist, backward_parent = {target: 0.0}, {}
        pq_backward = [(0.0, target)]

        best, meeting = float("inf"), None

        while pq_forward or pq_backward:
            if pq_forward:
                dist_u, u = heapq.heappop(pq_forward)
                if dist_u == forward_dist.get(u):
                    if dist_u > best: pq_forward.clear()
                    else:
                        if u in backward_dist:
                            if dist_u + backward_dist[u] < best:
                                best = dist_u + backward_dist[u]
                                meeting = u
                        for edge in forward.get(u, ()):
                            v = edge["v"]
                            new_dist = dist_u + edge["time"]
                            if new_dist < forward_dist.get(v, float("inf")):
                                forward_dist[v] = new_dist
                                forward_parent[v] = (u, edge)
                                heapq.heappush(pq_forward, (new_dist, v))

            if pq_backward:
                dist_u, u = heapq.heappop(pq_backward)
                if dist_u == backward_dist.get(u):
                    if dist_u > best: pq_backward.clear()
                    else:
                        if u in forward_dist:
                            if dist_u + forward_dist[u] < best:
                                best = dist_u + forward_dist[u]
                                meeting = u
                        for edge in backward.get(u, ()):
                            v = edge["u"]
                            new_dist = dist_u + edge["time"]
                            if new_dist < backward_dist.get(v, float("inf")):
                                backward_dist[v] = new_dist
                                backward_parent[v] = (u, edge)
                                heapq.heappush(pq_backward, (new_dist, v))

        if meeting is None: raise RuntimeError("Target unreachable")

        left, node = [], meeting
        while node != source:
            parent, edge = forward_parent[node]
            left.append(edge)
            node = parent
        left.reverse()

        right, node = [], meeting
        while node != target:
            parent, edge = backward_parent[node]
            right.append(edge)
            node = parent

        route_edges = left + right
        if not route_edges: raise RuntimeError("No valid CCH path.")
        return best, route_edges

    def unpack_edge(self, edge):
        if not edge.get("shortcut") or not edge.get("children"): return [edge]
        return self.unpack_edge(edge["children"][0]) + self.unpack_edge(edge["children"][1])

    # ============================================================
    # ROUTE OUTPUT
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

    # ============================================================
    # MAIN ROUTER
    # ============================================================

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None, flood_data=None):
        print(f"\n🗺️ FRENDS CCH route request: ({origin_lat}, {origin_lon}) → ({dest_lat}, {dest_lon})")

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

        try: cch_edges, rank = self.apply_cch_contraction(nodes, edges, source, target)
        except Exception as e: return {"status": "error", "message": f"CCH preprocessing failed: {e}"}

        try: self.customize_for_floods(nodes, cch_edges, flood_data, vehicle_layer)
        except Exception as e: return {"status": "error", "message": f"Flood customization failed: {e}"}

        try: base_time, route_edges = self.cch_query(nodes, cch_edges, rank, source, target)
        except Exception: return {"status": "error", "message": "No safe route available. Destination isolated by floods."}

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