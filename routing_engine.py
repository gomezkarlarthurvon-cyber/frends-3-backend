import sqlite3
import json
import math
import heapq
import random
import re
import requests
from math import radians, cos, sin, asin, sqrt

class FRENDSRoutingEngine:

    DEFAULT_SPEED_MPS = 8.33

    # --- Lane / carriageway recognition ---------------------------------
    LANE_TOLERANCE_DEG = 65.0                    # max angle between a road segment and a lane bound
    UTURN_PENALTY_S = 1800.0                     # in-place reversal (real crossovers cost far less, see turn penalty)
    LANE_SNAP_STEPS_DEG = (0.0005, 0.0015, 0.004)  # ~55 m, ~165 m, ~440 m search boxes for lane snapping
    ALONG_EDGE_SLACK_M = 25.0                    # GPS noise allowance before we treat the start node as "already passed"

    _BOUND_BEARINGS = {
        "N": 0.0, "NB": 0.0, "NORTH": 0.0, "NORTHBOUND": 0.0,
        "NE": 45.0, "NEB": 45.0, "NORTHEAST": 45.0, "NORTHEASTBOUND": 45.0,
        "E": 90.0, "EB": 90.0, "EAST": 90.0, "EASTBOUND": 90.0,
        "SE": 135.0, "SEB": 135.0, "SOUTHEAST": 135.0, "SOUTHEASTBOUND": 135.0,
        "S": 180.0, "SB": 180.0, "SOUTH": 180.0, "SOUTHBOUND": 180.0,
        "SW": 225.0, "SWB": 225.0, "SOUTHWEST": 225.0, "SOUTHWESTBOUND": 225.0,
        "W": 270.0, "WB": 270.0, "WEST": 270.0, "WESTBOUND": 270.0,
        "NW": 315.0, "NWB": 315.0, "NORTHWEST": 315.0, "NORTHWESTBOUND": 315.0,
    }
    _ALL_LANE_WORDS = {"ALL", "BOTH", "ANY", "EITHER", "BIDIRECTIONAL", "TWOWAY"}

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
    def project_point_on_segment(px, py, x1, y1, x2, y2):
        """Project point (px=lon, py=lat) onto segment (x1,y1)-(x2,y2).
        Returns (distance_m, t) where t in [0, 1] is the position along the segment."""
        k = cos(radians((y1 + y2) / 2.0))          # shrink longitude so the projection is not skewed
        dx = (x2 - x1) * k
        dy = y2 - y1
        denom = dx * dx + dy * dy
        if denom == 0:
            t = 0.0
        else:
            t = (((px - x1) * k) * dx + (py - y1) * dy) / denom
            t = max(0.0, min(1.0, t))
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        return FRENDSRoutingEngine.haversine_distance(py, px, closest_y, closest_x), t

    @staticmethod
    def point_to_line_distance(px, py, x1, y1, x2, y2):
        return FRENDSRoutingEngine.project_point_on_segment(px, py, x1, y1, x2, y2)[0]

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlon))
        initial_bearing = math.atan2(x, y)
        return (math.degrees(initial_bearing) + 360) % 360

    # ------------------------------------------------------------------
    # LANE / BOUND RECOGNITION
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_bearing(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):             # float("nan") parses fine and silently breaks every comparison
            return None
        return value % 360.0

    @classmethod
    def parse_lane_direction(cls, value):
        """Turn any lane/bound description into a bearing in degrees [0, 360).

        Accepts degrees (90, "90"), bounds ("NB", "southbound", "N/B", "NORTH BOUND",
        "Northbound (NB) lane", "SB lane") and returns None when there is no single
        direction ("ALL", "BOTH LANES", "NB/SB", empty, unknown, NaN)."""
        if value is None or isinstance(value, bool):
            return None
        if isinstance(value, (int, float)):
            return cls._normalize_bearing(value)

        text = str(value).strip().upper()
        if not text:
            return None
        try:
            return cls._normalize_bearing(float(text))
        except ValueError:
            pass

        words = re.findall(r"[A-Z]+", text)
        if any(w in cls._ALL_LANE_WORDS for w in words):
            return None
        bearings = {cls._BOUND_BEARINGS[w] for w in words if w in cls._BOUND_BEARINGS}
        return bearings.pop() if len(bearings) == 1 else None

    def is_same_lane_direction(self, edge_bearing, target_direction, tolerance=None):
        """True if a road segment flowing at `edge_bearing` belongs to the lane/bound `target_direction`.
        No usable direction means "affects both lanes" (fail-safe for floods and hazards)."""
        target_bearing = self.parse_lane_direction(target_direction)
        if target_bearing is None:
            return True
        tol = self.LANE_TOLERANCE_DEG if tolerance is None else tolerance
        angle_diff = abs((edge_bearing - target_bearing + 180) % 360 - 180)
        return angle_diff <= tol

    def resolve_lane_bearing(self, user_heading=None, lane_direction=None, switch_lane=False):
        """Decide which bound the trip starts in.
          1. lane_direction  - explicit user request ("NB"/"SB"/degrees); wins over GPS
          2. user_heading    - GPS heading
          3. switch_lane     - flips whichever of the above was chosen to the opposite carriageway"""
        bearing = self.parse_lane_direction(lane_direction)
        if bearing is None:
            bearing = self.parse_lane_direction(user_heading)
        if bearing is not None and switch_lane:
            bearing = (bearing + 180.0) % 360.0
        return bearing

    @staticmethod
    def _first_present(record, keys):
        """First non-empty value among keys. dict.get(a, dict.get(b)) stops at a key that exists but is null."""
        for key in keys:
            val = record.get(key)
            if val is not None and str(val).strip() != "":
                return val
        return None

    @staticmethod
    def orient_geometry(geometry, u_lat, u_lon, v_lat, v_lon):
        """Make sure an edge polyline ([lon, lat] points) runs u -> v.
        A reversed polyline makes every per-segment bearing point the wrong way, which flips
        NB/SB matching for floods, hazards and snapping."""
        if not geometry or len(geometry) < 2:
            return geometry
        try:
            first, last = geometry[0], geometry[-1]
            hav = FRENDSRoutingEngine.haversine_distance
            as_is = hav(u_lat, u_lon, first[1], first[0]) + hav(v_lat, v_lon, last[1], last[0])
            flipped = hav(v_lat, v_lon, first[1], first[0]) + hav(u_lat, u_lon, last[1], last[0])
            if flipped < as_is:
                return list(reversed(geometry))
        except (TypeError, IndexError, ValueError):
            pass
        return geometry

    def _edge_polyline(self, geometry, u_lat, u_lon, v_lat, v_lon):
        """Parse a stored geometry (JSON text or list) into a u -> v oriented [lon, lat] polyline."""
        data = None
        if geometry:
            try:
                data = json.loads(geometry) if isinstance(geometry, (str, bytes)) else geometry
            except Exception:
                data = None
        if not data or len(data) < 2:
            return [[u_lon, u_lat], [v_lon, v_lat]]
        return self.orient_geometry(data, u_lat, u_lon, v_lat, v_lon)

    def heading_change_penalty(self, heading_in, heading_out):
        angle_diff = (heading_out - heading_in) % 360
        if angle_diff > 180: angle_diff -= 360

        abs_angle = abs(angle_diff)
        if abs_angle > 135: return self.UTURN_PENALTY_S   # in-place reversal
        if -130 < angle_diff < -65: return 20.0           # left turn / crossover entry
        return 0.0

    def get_turn_penalty(self, u, v, w, nodes):
        lat1, lon1 = nodes[u]
        lat2, lon2 = nodes[v]
        lat3, lon3 = nodes[w]

        heading_in = self.calculate_heading(lat1, lon1, lat2, lon2)
        heading_out = self.calculate_heading(lat2, lon2, lat3, lon3)
        return self.heading_change_penalty(heading_in, heading_out)

    def _snap_to_lane(self, lat, lon, cursor, target_bearing, prefer="ahead"):
        """Snap a GPS point onto the nearest road segment that flows in `target_bearing`.

        Works on the real (oriented) edge geometry and perpendicular distance, and widens the
        search box in steps. Highway edges can be hundreds of metres long, so a fixed ~55 m
        box around *nodes* often found nothing and silently fell back to a lane-blind snap.

        prefer="ahead"   -> node the vehicle is driving toward (use for the trip origin)
        prefer="nearest" -> closest end of the matched edge   (use for the destination)"""
        for delta in self.LANE_SNAP_STEPS_DEG:
            box = (lat - delta, lat + delta, lon - delta, lon + delta)
            cursor.execute(
                """
                SELECT e.u, e.v, n1.lat, n1.lon, n2.lat, n2.lon, e.geometry
                FROM edges e
                JOIN nodes n1 ON e.u = n1.id
                JOIN nodes n2 ON e.v = n2.id
                WHERE (n1.lat BETWEEN ? AND ? AND n1.lon BETWEEN ? AND ?)
                   OR (n2.lat BETWEEN ? AND ? AND n2.lon BETWEEN ? AND ?)
                """,
                box + box
            )

            best, best_score = None, float("inf")
            for u_id, v_id, u_lat, u_lon, v_lat, v_lon, geom in cursor.fetchall():
                polyline = self._edge_polyline(geom, u_lat, u_lon, v_lat, v_lon)
                seg_lengths = [
                    self.haversine_distance(polyline[i][1], polyline[i][0], polyline[i + 1][1], polyline[i + 1][0])
                    for i in range(len(polyline) - 1)
                ]
                total = sum(seg_lengths)
                travelled = 0.0
                for i, seg_len in enumerate(seg_lengths):
                    lon1, lat1 = polyline[i][0], polyline[i][1]
                    lon2, lat2 = polyline[i + 1][0], polyline[i + 1][1]
                    if seg_len > 0:
                        seg_bearing = self.calculate_heading(lat1, lon1, lat2, lon2)
                        angle_diff = abs((seg_bearing - target_bearing + 180) % 360 - 180)
                        if angle_diff <= self.LANE_TOLERANCE_DEG:
                            dist, t = self.project_point_on_segment(lon, lat, lon1, lat1, lon2, lat2)
                            score = dist + 0.4 * angle_diff     # prefer close AND well-aligned
                            if score < best_score:
                                best_score = score
                                best = (u_id, v_id, travelled + t * seg_len, total)
                    travelled += seg_len

            if best is not None:
                u_id, v_id, along, total = best
                if prefer == "nearest":
                    return u_id if along <= total / 2.0 else v_id
                # "ahead": the vehicle can't drive back to u, so start from v unless it is basically still at u
                return u_id if along <= self.ALONG_EDGE_SLACK_M else v_id
        return None

    def origin_candidates(self, lat, lon, cursor, lane_bearing=None):
        """Ordered start options as [(node_id, start_bearing), ...].
          1. the lane the vehicle is in / the user asked for
          2. the opposite lane            (used when the requested lane is dead-ended, e.g. by a flood)
          3. nearest node, lane-agnostic  (last resort)"""
        if lane_bearing is None:
            node = self.nearest_node_sql(lat, lon, cursor)
            return [(node, None)] if node is not None else []

        opposite = (lane_bearing + 180.0) % 360.0
        candidates = []

        def add(node, bearing):
            if node is not None and all(node != existing for existing, _ in candidates):
                candidates.append((node, bearing))

        try:
            add(self._snap_to_lane(lat, lon, cursor, lane_bearing), lane_bearing)
            add(self._snap_to_lane(lat, lon, cursor, opposite), opposite)
        except (sqlite3.Error, TypeError, ValueError):
            pass
        add(self.nearest_node_sql(lat, lon, cursor), None)
        return candidates

    def resolve_endpoints(self, origin_lat, origin_lon, dest_lat, dest_lon, lane_bearing=None, dest_bearing=None):
        conn = sqlite3.connect(self.db_file)
        cursor = conn.cursor()
        try:
            sources = self.origin_candidates(origin_lat, origin_lon, cursor, lane_bearing)
            target = self.nearest_node_sql(dest_lat, dest_lon, cursor, user_heading=dest_bearing, prefer="nearest")
            return sources, target
        finally:
            cursor.close()
            conn.close()

    def nearest_node_sql(self, lat, lon, cursor, user_heading=None, prefer="ahead"):
        # user_heading may be degrees OR a bound ("NB", "southbound", ...). It used to go through
        # float() only, so any bound string silently disabled lane matching.
        target_bearing = self.parse_lane_direction(user_heading)
        if target_bearing is not None:
            try:
                node = self._snap_to_lane(lat, lon, cursor, target_bearing, prefer)
                if node is not None:
                    return node
            except (sqlite3.Error, TypeError, ValueError):
                pass

        # Standard fallback when stationary, no lane matched, or planning a route manually
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

    # Accepts dynamic buffer radius for the expansion strategy + optional user_heading
    def load_local_graph(self, origin_lat, origin_lon, dest_lat, dest_lon, buffer_radius, user_heading=None,
                         dest_heading=None, origin_node=None, destination_node=None):
        min_lat = min(origin_lat, dest_lat) - buffer_radius
        max_lat = max(origin_lat, dest_lat) + buffer_radius
        min_lon = min(origin_lon, dest_lon) - buffer_radius
        max_lon = max(origin_lon, dest_lon) + buffer_radius

        conn = sqlite3.connect(self.db_file)
        cursor = conn.cursor()

        try:
            # Pre-resolved endpoints (from compute_route) skip snapping entirely
            if origin_node is None:
                origin_node = self.nearest_node_sql(origin_lat, origin_lon, cursor, user_heading=user_heading)
            if destination_node is None:
                destination_node = self.nearest_node_sql(dest_lat, dest_lon, cursor, user_heading=dest_heading, prefer="nearest")

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
                if geometry_data:
                    geometry_data = self.orient_geometry(
                        geometry_data, nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])

                edge_length = float(length or 0)
                edge_time = float(time_value or 0)

                # 🏎️ EXPRESSWAY MAGNET: 
                # If base speed >= 15 m/s (approx 54 km/h), it is a major highway.
                # Artificially discount the time by 40% so the A* engine prioritizes it.
                if edge_time > 0 and (edge_length / edge_time) >= 15.0:
                    edge_time *= 0.60 

                edges.append({
                    "u": u, "v": v, 
                    "length": edge_length, 
                    "time": edge_time,
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

    # 🌟 UPGRADED: MMDA Standards + Lane-Aware Flood Customization
    def customize_for_floods(self, nodes, edges, flood_data, vehicle_layer):
        flooded_edges = set()
        # Official MMDA Flood Gauge Thresholds (13 in = 33.02cm, 19 in = 48.26cm, 26 in = 66.04cm)
        limits = {"LOW": 33.02, "MID": 48.26, "HIGH": 66.04}
        max_safe_depth = limits.get(str(vehicle_layer).upper(), 33.02)
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
                # 🌟 Read optional lane direction ("NB", "SB", "EB", "WB", or heading degrees)
                lane_dir = self._first_present(latest, ("direction", "bound", "lane", "heading"))
                
                if water_level > max_safe_depth and lat and lon:
                    flood_points.append((lat, lon, lane_dir))
            except (TypeError, ValueError):
                continue

        if not flood_points: return flooded_edges

        # Tighter 15-meter blast radius. Safely blocks the flooded intersection 
        # WITHOUT bleeding over and destroying the safe parallel streets!
        BLAST_RADIUS = 15.0
        
        for edge in edges:
            if edge["shortcut"]: continue
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue

            # Calculate overall directional bearing of this directed road edge (u -> v)
            edge_bearing = self.calculate_heading(nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])

            geometry = edge.get("geometry")
            if not geometry:
                geometry = [(nodes[u][1], nodes[u][0]), (nodes[v][1], nodes[v][0])]

            blocked = False
            for flood_lat, flood_lon, lane_dir in flood_points:
                for i in range(len(geometry) - 1):
                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]
                    dist = self.point_to_line_distance(flood_lon, flood_lat, lon1, lat1, lon2, lat2)
                    if dist <= BLAST_RADIUS:
                        # Use exact sub-segment bearing if coordinates differ, otherwise overall edge bearing
                        seg_bearing = self.calculate_heading(lat1, lon1, lat2, lon2) if (lat1 != lat2 or lon1 != lon2) else edge_bearing
                        if self.is_same_lane_direction(seg_bearing, lane_dir):
                            blocked = True
                            break
                if blocked: break

            if blocked:
                edge["blocked"] = True
                flooded_edges.add((u, v))

        if len(flood_points) > 0:
            print(f"🌊 HAZARD DETECTED: {len(flood_points)} floods over {max_safe_depth}cm. {len(flooded_edges)} lane-matched roads cut.")

        return flooded_edges

    # 🌟 UPGRADED: Lane-Aware Crowdsourced Hazard Time Penalties
    def apply_hazard_penalties(self, nodes, edges, reports_data):
        if not reports_data: return

        # Define time penalties in seconds based on hazard severity
        hazard_penalties = {
            "Accident": 600.0,      # +10 mins (Severe choke point)
            "Construction": 300.0,  # +5 mins (Lane reduction)
            "Hazard": 180.0,        # +3 mins (Slowing down)
            "Police": 60.0          # +1 min (Minor rubbernecking delay)
        }
        
        active_hazards = []
        for _, report in reports_data.items():
            if not isinstance(report, dict): continue
            
            try:
                lat = float(report.get("lat", 0))
                lon = float(report.get("lng", report.get("lon", 0)))
                hazard_type = report.get("type", "Hazard")
                penalty = hazard_penalties.get(hazard_type, 180.0)
                # 🌟 Extract reporter's heading or bound ("NB", "SB", etc.) so opposite lane isn't penalized
                lane_dir = self._first_present(report, ("heading", "direction", "bound", "lane"))
                
                if lat and lon:
                    active_hazards.append((lat, lon, penalty, lane_dir))
            except (TypeError, ValueError):
                continue

        if not active_hazards: return

        # 50-meter collision radius for hazards
        HAZARD_BLAST_RADIUS = 50.0  
        
        affected_edges = 0
        for edge in edges:
            if edge.get("blocked") or edge.get("shortcut"): continue
            
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue

            edge_bearing = self.calculate_heading(nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])

            geometry = edge.get("geometry")
            if not geometry:
                geometry = [(nodes[u][1], nodes[u][0]), (nodes[v][1], nodes[v][0])]

            # Check spatial collision AND directional alignment
            for hazard_lat, hazard_lon, penalty, lane_dir in active_hazards:
                for i in range(len(geometry) - 1):
                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]
                    dist = self.point_to_line_distance(hazard_lon, hazard_lat, lon1, lat1, lon2, lat2)
                    
                    if dist <= HAZARD_BLAST_RADIUS:
                        seg_bearing = self.calculate_heading(lat1, lon1, lat2, lon2) if (lat1 != lat2 or lon1 != lon2) else edge_bearing
                        if self.is_same_lane_direction(seg_bearing, lane_dir):
                            edge["time"] += penalty
                            affected_edges += 1
                            break  # Apply penalty once per edge to prevent stacking the same incident

        if affected_edges > 0:
            print(f"⚠️ JIT-CCH PENALTIES APPLIED: {affected_edges} lane-matched road segments delayed by user incident reports.")

    def apply_cch_contraction(self, nodes, edges, source, target, protected=None):
        forward, backward = self.build_adjacency(nodes, edges)
        new_edges = list(edges)
        contracted = set()
        # Every candidate start node must survive contraction, or a lane switch has nothing to start from
        keep = set(protected or ())
        keep.update((source, target))

        for node in nodes:
            if node in keep: continue

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

    def cch_query(self, nodes, edges, source, target, start_bearing=None):
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
                elif start_bearing is not None:
                    # First leg: the vehicle is already travelling in its lane. Leaving the source
                    # against that lane is a reversal, not a free choice.
                    first_v = edge.get("first_v", edge["v"])
                    out_heading = self.calculate_heading(nodes[u][0], nodes[u][1], nodes[first_v][0], nodes[first_v][1])
                    new_time += self.heading_change_penalty(start_bearing, out_heading)

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

    def query_with_lane_fallback(self, nodes, cch_edges, sources, target):
        """Try each start option in order (requested lane -> opposite lane -> lane-agnostic).
        Returns (time, edges, index_of_source_used, start_bearing_used)."""
        last_error = Exception("Target unreachable")
        for idx, (src_node, bearing) in enumerate(sources):
            try:
                base_time, route_edges = self.cch_query(nodes, cch_edges, src_node, target, start_bearing=bearing)
                return base_time, route_edges, idx, bearing
            except Exception as exc:
                last_error = exc
        raise last_error

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None,
                      flood_data=None, reports_data=None, user_heading=None,
                      lane_direction=None, switch_lane=False, dest_direction=None):
        """
        Lane controls (all optional):
          user_heading   GPS heading in degrees, or a bound ("NB", "southbound", ...).
          lane_direction explicit user request ("NB"/"SB"/"EB"/"WB"/degrees). Overrides GPS heading.
          switch_lane    True = flip to the opposite carriageway of whatever lane was resolved above.
          dest_direction lane the destination sits on, so the trip ends on the right carriageway.
        If the chosen lane is dead-ended (e.g. flooded ahead) the engine automatically switches to the
        opposite lane; the response reports it in "lane_switched" / "lane_bearing".
        """
        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided."}

        lane_bearing = self.resolve_lane_bearing(user_heading, lane_direction, switch_lane)
        dest_bearing = self.parse_lane_direction(dest_direction)

        try:
            sources, target = self.resolve_endpoints(
                origin_lat, origin_lon, dest_lat, dest_lon, lane_bearing, dest_bearing
            )
        except Exception as e:
            return {"status": "error", "message": f"Database error: {e}"}

        if not sources or target is None:
            return self.fetch_osrm_fallback(origin_lat, origin_lon, dest_lat, dest_lon, api_key)

        # 🌟 THE MASTER FIX: DYNAMIC EXPANSION STRATEGY
        # Starts with a blazing-fast 1.5km grid. If floods block the detour, it automatically expands
        # up to an 8km radius to find side-streets, preventing both "No Path" errors AND "Timeouts"!
        buffer_stages = [0.04, 0.08, 0.15]
        route_edges = []
        base_time = 0
        lane_switched, used_bearing = False, lane_bearing

        for attempt, buf in enumerate(buffer_stages):
            try:
                nodes, edges, _ = self.load_local_graph(
                    origin_lat, origin_lon, dest_lat, dest_lon, buf,
                    origin_node=sources[0][0], destination_node=target
                )
            except Exception as e:
                return {"status": "error", "message": f"Database error: {e}"}

            if not nodes or not edges:
                if attempt == len(buffer_stages) - 1:
                    return self.fetch_osrm_fallback(origin_lat, origin_lon, dest_lat, dest_lon, api_key)
                continue

            try:
                # 🌟 THE JIT-CCH CUSTOMIZATION PIPELINE
                self.customize_for_floods(nodes, edges, flood_data, vehicle_layer)
                self.apply_hazard_penalties(nodes, edges, reports_data) # Inject live traffic delays

                active = [(n, b) for n, b in sources if n in nodes]
                if not active or target not in nodes:
                    raise KeyError("Start or destination lies outside the local graph")

                cch_edges = self.apply_cch_contraction(
                    nodes, edges, active[0][0], target, protected=[n for n, _ in active]
                )
                base_time, route_edges, used_idx, used_bearing = self.query_with_lane_fallback(
                    nodes, cch_edges, active, target
                )
                lane_switched = used_idx > 0 and lane_bearing is not None

                if lane_switched:
                    print(f"🔀 Lane switched: requested bound {lane_bearing:.0f}° was dead-ended, routing from the opposite lane.")
                print(f"✅ Safe detour successfully mapped using radius stage {attempt+1} ({buf})")
                break

            except Exception:
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
            result["lane_bearing"] = used_bearing
            result["lane_switched"] = bool(lane_switched)
            return result
        except Exception as e:
            return {"status": "error", "message": f"Failed compiling payload: {e}"}