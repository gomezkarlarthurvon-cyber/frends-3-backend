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
    MIN_HEADING_SPEED_MPS = 1.5                  # below this a GPS heading is noise, not a direction of travel
    AUTO_LANE_SNAP_MAX_M = 80.0                  # auto lane choice only considers carriageways this close to the origin

    # Official MMDA flood passability thresholds (Table 1). Depth >= threshold is IMPASSABLE.
    #   >= 13 in (33.02 cm): sedans/hatchbacks (LOW)   >= 19 in (48.26 cm): SUVs (MID)   >= 26 in (66.04 cm): trucks/buses (HIGH)
    MMDA_THRESHOLDS_CM = {"LOW": 33.02, "MID": 48.26, "HIGH": 66.04}
    FLOOD_BLAST_RADIUS_M = 15.0                  # road segments within this distance of a flooded sensor are cut
    FLOOD_OFFROAD_REACH_M = 30.0                 # safety net: a sensor with NO road within 15 m still cuts its nearest road
    HAZARD_RADIUS_M = 50.0                       # crowdsourced hazards act within 50 m of the report
    HAZARD_TTL_S = 4 * 60 * 60                   # reports older than 4 h are ignored (road weight returns to normal)
    HAZARD_PENALTIES_S = {"Accident": 600.0, "Construction": 300.0, "Hazard": 180.0, "Police": 60.0}
    BBOX_STAGES_DEG = (0.0135, 0.027, 0.072)     # ~1.5 km corridor first, then ~3 km, then ~8 km if floods trap the route
    MAX_TRAFFIC_SEGMENTS = 10                    # Dynamic Traffic Chunker cap (protects the Render server)
    TRAFFIC_CACHE_TTL_S = 120.0

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
        self.last_flood_zones = []
        self.last_flood_info = {"nodes_checked": 0, "flooded_nodes": 0, "edges_blocked": 0}
        self._traffic_cache = {}                 # (lat3, lon3) -> (timestamp, multiplier)
        print(f" FRENDS JIT-CCH Engine initialized: {self.db_file}")

    def get_tomtom_traffic_multiplier(self, lat, lon, api_key):
        """Live congestion multiplier (free-flow speed / current speed) from TomTom Traffic Flow.
        Cached per ~100 m cell for TRAFFIC_CACHE_TTL_S. On any failure the multiplier is a neutral 1.0:
        the old code invented random congestion when TomTom rate-limited, i.e. fake red/yellow roads."""
        import time as _time
        key = (round(lat, 3), round(lon, 3))
        hit = self._traffic_cache.get(key)
        if hit and _time.time() - hit[0] < self.TRAFFIC_CACHE_TTL_S:
            return hit[1]
        url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json"
        params = {"key": api_key, "point": f"{lat},{lon}"}
        multiplier = 1.0
        try:
            response = requests.get(url, params=params, timeout=1.5)
            if response.status_code == 200:
                data = response.json().get("flowSegmentData", {})
                current_speed = data.get("currentSpeed")
                free_flow_speed = data.get("freeFlowSpeed")
                if current_speed and free_flow_speed and current_speed > 0:
                    multiplier = min(free_flow_speed / current_speed, 5.0)
                    self._traffic_cache[key] = (_time.time(), multiplier)
        except Exception:
            pass
        return multiplier

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

    def _snap_to_lane(self, lat, lon, cursor, target_bearing, prefer="ahead", max_dist=None):
        """Snap a GPS point onto the nearest road segment that flows in `target_bearing`.

        Works on the real (oriented) edge geometry and perpendicular distance, and widens the
        search box in steps. Uses a two-tier angular tolerance (65°, then 90° hemisphere fallback)
        so diagonal avenues still snap cleanly to NB/SB carriageways."""
        steps = self.LANE_SNAP_STEPS_DEG
        if max_dist is not None:   # one box just big enough for the cap is all we need
            steps = (next((d for d in steps if d * 111000.0 >= max_dist), steps[-1]),)

        for tol in (self.LANE_TOLERANCE_DEG, 90.0):
            for delta in steps:
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
                            if angle_diff <= tol:
                                dist, t = self.project_point_on_segment(lon, lat, lon1, lat1, lon2, lat2)
                                score = dist + 0.4 * angle_diff     # prefer close AND well-aligned
                                if (max_dist is None or dist <= max_dist) and score < best_score:
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

    def origin_candidates(self, lat, lon, cursor, lane_bearing=None, dest=None):
        """Ordered start options as [(node_id, start_bearing), ...].

        Lane known (user request / moving GPS heading):
          1. the lane the vehicle is in / the user asked for
          2. the opposite lane            (used when the requested lane is dead-ended, e.g. by a flood)
          3. nearest node, lane-agnostic  (last resort)
        No lane known (pinned place, searched address, stationary GPS) -> "auto":
          the nearest node plus the carriageways heading TOWARD and AWAY from the destination.
          None carries a start bearing, so nothing forces a loop around the block, and
          compute_route keeps whichever start gives the fastest trip."""
        if lane_bearing is None:
            plain = self.nearest_node_sql(lat, lon, cursor)
            candidates = [(plain, None)] if plain is not None else []
            if dest is not None:
                toward = self.calculate_heading(lat, lon, dest[0], dest[1])
                for bearing in (toward, (toward + 180.0) % 360.0):
                    try:
                        node = self._snap_to_lane(lat, lon, cursor, bearing, max_dist=self.AUTO_LANE_SNAP_MAX_M)
                    except (sqlite3.Error, TypeError, ValueError):
                        node = None
                    if node is not None and all(node != n for n, _ in candidates):
                        candidates.append((node, None))
            return candidates

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
            sources = self.origin_candidates(origin_lat, origin_lon, cursor, lane_bearing, dest=(dest_lat, dest_lon))
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

    # ------------------------------------------------------------------
    # FLOOD NODE PARSING (mirrors readFloodNode() in MapSection.jsx)
    # ------------------------------------------------------------------
    @staticmethod
    def _num(value):
        try:
            if value is None or isinstance(value, bool) or str(value).strip() == "":
                return None
            f = float(value)
            return f if math.isfinite(f) else None
        except (TypeError, ValueError):
            return None

    @classmethod
    def _read_flood_node(cls, container):
        """Return (lat, lon, depth_cm, lane) for one Firebase node, or None.

        Handles flat nodes, push-key histories (latest entry wins) and the field aliases the
        app/IoT firmware use (lat/lng/lon/latitude, waterLevel/water_level/depth/level).
        If the latest push entry has no coordinates, the node's own lat/lng is used -
        the old code skipped such nodes, so a flood shown on the map never blocked a road."""
        if not isinstance(container, dict):
            return None
        push_keys = sorted(k for k in container if str(k).startswith("-") and isinstance(container[k], dict))
        latest = container[push_keys[-1]] if push_keys else None
        sources = [src for src in (latest, container) if isinstance(src, dict)]

        def pick(keys):
            for src in sources:
                for key in keys:
                    n = cls._num(src.get(key))
                    if n is not None:
                        return n
            return None

        lat = pick(("lat", "latitude"))
        lon = pick(("lng", "lon", "longitude"))
        depth = pick(("waterLevel", "water_level", "depth", "level", "floodDepth", "flood_depth"))
        if lat is None or lon is None or depth is None or (lat == 0 and lon == 0):
            return None
        lane = None
        for src in sources:
            lane = cls._first_present(src, ("direction", "bound", "lane", "heading"))
            if lane is not None:
                break
        return lat, lon, depth, lane

    # 🌟 UPGRADED: MMDA Standards + Lane-Aware Flood Customization
    def customize_for_floods(self, nodes, edges, flood_data, vehicle_layer):
        flooded_edges = set()
        self.last_flood_info = {"nodes_checked": 0, "flooded_nodes": 0, "edges_blocked": 0}
        # Official MMDA Flood Gauge Thresholds (13 in = 33.02cm, 19 in = 48.26cm, 26 in = 66.04cm)
        max_safe_depth = self.MMDA_THRESHOLDS_CM.get(str(vehicle_layer).upper(), self.MMDA_THRESHOLDS_CM["LOW"])

        self.last_flood_zones = []
        # Firebase returns a LIST (not a dict) when node keys are 0,1,2...
        if isinstance(flood_data, list):
            flood_data = {str(i): v for i, v in enumerate(flood_data) if v is not None}
        if not flood_data or not isinstance(flood_data, dict):
            return flooded_edges

        flood_points = []
        for _, container in flood_data.items():
            reading = self._read_flood_node(container)
            if reading is None:
                continue
            self.last_flood_info["nodes_checked"] += 1
            lat, lon, depth, lane_dir = reading
            # >= matches the frontend (depth >= limit); it used to be a strict >.
            if depth >= max_safe_depth:
                flood_points.append((lat, lon, lane_dir))

        self.last_flood_info["flooded_nodes"] = len(flood_points)
        if not flood_points:
            return flooded_edges

        # Blast radius: the original tight 15 m, so a flood blocks its own intersection WITHOUT bleeding
        # over and cutting the safe parallel streets. Only a flood pinned off the road (nearest lane-matched
        # road farther than 15 m, which the old fixed radius missed entirely) reaches out to the road,
        # up to MAX_R, so it still blocks something.
        BLAST_RADIUS, MAX_R, SLACK = self.FLOOD_BLAST_RADIUS_M, self.FLOOD_OFFROAD_REACH_M, 4.0
        nearest = [float("inf")] * len(flood_points)
        hits = []  # (edge, flood_index, distance)

        for edge in edges:
            if edge["shortcut"]: continue
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue

            edge_bearing = self.calculate_heading(nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])
            geometry = edge.get("geometry")
            if not geometry:
                geometry = [(nodes[u][1], nodes[u][0]), (nodes[v][1], nodes[v][0])]

            for fi, (flood_lat, flood_lon, lane_dir) in enumerate(flood_points):
                best = float("inf")
                for i in range(len(geometry) - 1):
                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]
                    dist = self.point_to_line_distance(flood_lon, flood_lat, lon1, lat1, lon2, lat2)
                    if dist > MAX_R or dist >= best:
                        continue
                    seg_bearing = self.calculate_heading(lat1, lon1, lat2, lon2) if (lat1 != lat2 or lon1 != lon2) else edge_bearing
                    if self.is_same_lane_direction(seg_bearing, lane_dir):
                        best = dist
                if best <= MAX_R:
                    hits.append((edge, fi, best))
                    if best < nearest[fi]:
                        nearest[fi] = best

        radii = [BLAST_RADIUS if d <= BLAST_RADIUS else min(MAX_R, d + SLACK) for d in nearest]
        for edge, fi, dist in hits:
            if dist <= radii[fi]:
                edge["blocked"] = True
                flooded_edges.add((edge["u"], edge["v"]))

        self.last_flood_info["edges_blocked"] = len(flooded_edges)
        # Zones (lat, lon, lane, radius) so start/end points can be checked against them too
        self.last_flood_zones = [(f[0], f[1], f[2], radii[i]) for i, f in enumerate(flood_points)]
        print(f"🌊 HAZARD DETECTED: {len(flood_points)} floods over {max_safe_depth}cm. {len(flooded_edges)} lane-matched roads cut.")
        return flooded_edges

    # 🌟 UPGRADED: Lane-Aware Crowdsourced Hazard Time Penalties
    @staticmethod
    def _report_age_seconds(report, now=None):
        """Age of a crowdsourced report in seconds, or None when it carries no usable timestamp.
        Accepts epoch milliseconds (the app), epoch seconds, or an ISO-8601 string."""
        import time as _time
        from datetime import datetime, timezone
        now = _time.time() if now is None else now
        ts = None
        for key in ("timestamp", "time", "createdAt", "created_at"):
            if report.get(key) not in (None, ""):
                ts = report.get(key); break
        if ts is None:
            return None
        try:
            ts = float(ts)
            if ts > 1e11: ts /= 1000.0           # milliseconds -> seconds
            return now - ts
        except (TypeError, ValueError):
            try:
                return now - datetime.fromisoformat(str(ts).replace("Z", "+00:00")).astimezone(timezone.utc).timestamp()
            except Exception:
                return None

    # 🌟 Lane-aware crowdsourced hazards = SOFT constraints (time penalties, never closures)
    def apply_hazard_penalties(self, nodes, edges, reports_data):
        if not reports_data: return
        if isinstance(reports_data, list):
            reports_data = {str(i): v for i, v in enumerate(reports_data) if v is not None}
        if not isinstance(reports_data, dict): return

        active_hazards, expired = [], 0
        for _, report in reports_data.items():
            if not isinstance(report, dict): continue
            try:
                # Time-to-Live: stale reports must not distort routes
                age = self._report_age_seconds(report)
                if age is not None and age > self.HAZARD_TTL_S:
                    expired += 1
                    continue
                lat = float(report.get("lat", 0))
                lon = float(report.get("lng", report.get("lon", 0)))
                penalty = self.HAZARD_PENALTIES_S.get(report.get("type", "Hazard"), 180.0)
                # Reporter's heading or bound ("NB", "SB", ...) so the opposite carriageway isn't penalised
                lane_dir = self._first_present(report, ("bound", "direction", "lane", "heading"))
                if lat and lon:
                    active_hazards.append((lat, lon, penalty, lane_dir))
            except (TypeError, ValueError):
                continue

        if expired:
            print(f"🕓 {expired} hazard report(s) older than 4 h ignored (TTL).")
        if not active_hazards: return

        affected_edges = 0
        for edge in edges:
            if edge.get("blocked") or edge.get("shortcut"): continue
            u, v = edge["u"], edge["v"]
            if u not in nodes or v not in nodes: continue

            edge_bearing = self.calculate_heading(nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1])
            geometry = edge.get("geometry") or [[nodes[u][1], nodes[u][0]], [nodes[v][1], nodes[v][0]]]

            # Penalty is applied ONCE per edge (the most severe matching incident), never stacked
            edge_penalty = 0.0
            for hazard_lat, hazard_lon, penalty, lane_dir in active_hazards:
                if penalty <= edge_penalty: continue
                for i in range(len(geometry) - 1):
                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]
                    dist = self.point_to_line_distance(hazard_lon, hazard_lat, lon1, lat1, lon2, lat2)
                    if dist <= self.HAZARD_RADIUS_M:
                        seg_bearing = self.calculate_heading(lat1, lon1, lat2, lon2) if (lat1 != lat2 or lon1 != lon2) else edge_bearing
                        if self.is_same_lane_direction(seg_bearing, lane_dir):
                            edge_penalty = penalty
                            break
            if edge_penalty > 0:
                edge["time"] += edge_penalty
                affected_edges += 1

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

    def cch_query(self, nodes, edges, source, target, start_bearing=None, dest_bearing=None):
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

                if v == target and dest_bearing is not None:
                    # Final leg: penalize arriving at the destination against the target carriageway bound
                    last_u = edge.get("last_u", u)
                    in_heading = self.calculate_heading(nodes[last_u][0], nodes[last_u][1], nodes[v][0], nodes[v][1])
                    new_time += self.heading_change_penalty(in_heading, dest_bearing)

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
        """Dynamic Traffic Chunker: split a route into at most MAX_TRAFFIC_SEGMENTS pieces, look up TomTom
        congestion once per unique ~100 m cell (cached, fetched concurrently) and colour each piece."""
        if not osrm_coords or len(osrm_coords) < 2:
            return [], 0.0

        total_dist = 0.0
        for i in range(len(osrm_coords) - 1):
            total_dist += self.haversine_distance(osrm_coords[i][1], osrm_coords[i][0], osrm_coords[i + 1][1], osrm_coords[i + 1][0])
        chunk_size = max(350.0, total_dist / float(self.MAX_TRAFFIC_SEGMENTS))
        base_speed = 8.33 if is_city else 16.6

        raw, chunk, acc = [], [], 0.0            # raw: [(coords, distance_m)]
        for i in range(len(osrm_coords) - 1):
            lon1, lat1 = osrm_coords[i]
            lon2, lat2 = osrm_coords[i + 1]
            acc += self.haversine_distance(lat1, lon1, lat2, lon2)
            chunk.append({"latitude": lat1, "longitude": lon1})
            if acc >= chunk_size or i == len(osrm_coords) - 2:
                chunk.append({"latitude": lat2, "longitude": lon2})
                raw.append((list(chunk), acc))
                chunk, acc = [{"latitude": lat2, "longitude": lon2}], 0.0

        # Hard cap: fold any overflow into the last allowed segment
        while len(raw) > self.MAX_TRAFFIC_SEGMENTS:
            c2, d2 = raw.pop()
            c1, d1 = raw.pop()
            raw.append((c1 + c2[1:], d1 + d2))

        def mid_of(coords):
            return ((coords[0]["latitude"] + coords[-1]["latitude"]) / 2.0,
                    (coords[0]["longitude"] + coords[-1]["longitude"]) / 2.0)

        multipliers = {}
        if api_key and api_key != "undefined":
            cells = {}
            for coords, _ in raw:
                m_lat, m_lon = mid_of(coords)
                cells[(round(m_lat, 3), round(m_lon, 3))] = (m_lat, m_lon)
            try:
                from concurrent.futures import ThreadPoolExecutor
                with ThreadPoolExecutor(max_workers=min(5, max(1, len(cells)))) as pool:
                    futures = {k: pool.submit(self.get_tomtom_traffic_multiplier, ll[0], ll[1], api_key) for k, ll in cells.items()}
                    multipliers = {k: f.result() for k, f in futures.items()}
            except Exception:
                multipliers = {}

        segments, total_duration = [], 0.0
        for coords, dist in raw:
            m_lat, m_lon = mid_of(coords)
            multiplier = multipliers.get((round(m_lat, 3), round(m_lon, 3)), 1.0)
            if multiplier >= 2.2: color = "#EF4444"
            elif multiplier >= 1.25: color = "#F59E0B"
            else: color = "#00A3FF"
            segments.append({"coords": coords, "color": color})
            total_duration += (dist / base_speed) * multiplier
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

    # ------------------------------------------------------------------
    # FLOOD-AWARE START / END POINTS
    # ------------------------------------------------------------------
    def _stub_hits_flood(self, p_lat, p_lon, n_lat, n_lon, zones):
        """True if the short piece of road between a pin and the graph node it snaps to crosses a flood.
        The graph only contains whole edges, so a flood on the partial stretch between the pin and its
        junction used to be invisible: the flooded edge was removed, the route simply started (or ended)
        on the far side of it, and the drawn line cut straight through the water."""
        bearing = self.calculate_heading(p_lat, p_lon, n_lat, n_lon) if (p_lat != n_lat or p_lon != n_lon) else None
        for z_lat, z_lon, lane, radius in zones:
            if self.point_to_line_distance(z_lon, z_lat, p_lon, p_lat, n_lon, n_lat) <= radius:
                if bearing is None or self.is_same_lane_direction(bearing, lane):
                    return True
        return False

    def _clear_endpoint_candidates(self, lat, lon, nodes, base, zones, max_extra=8, radius_m=700.0):
        """base: [(node_id, bearing)] chosen by lane snapping. Drop those whose stub crosses a flood and
        append the nearest other junctions (any direction => the vehicle may turn around) whose stub is clear.
        If everything is flooded (the pin itself is in the water) the original list is returned unchanged."""
        if not zones:
            return [(n, b) for n, b in base if n in nodes]
        kept = [(n, b) for n, b in base
                if n in nodes and not self._stub_hits_flood(lat, lon, nodes[n][0], nodes[n][1], zones)]
        have = {n for n, _ in kept}
        near = []
        for n_id, (n_lat, n_lon) in nodes.items():
            if n_id in have:
                continue
            d = self.haversine_distance(lat, lon, n_lat, n_lon)
            if d <= radius_m:
                near.append((d, n_id))
        near.sort()
        for _, n_id in near:
            if len(kept) >= len(have) + max_extra:
                break
            if not self._stub_hits_flood(lat, lon, nodes[n_id][0], nodes[n_id][1], zones):
                kept.append((n_id, None))
        return kept or [(n, b) for n, b in base if n in nodes]

    def query_with_lane_fallback(self, nodes, cch_edges, sources, target, pick_best=False, origin=None, dest_bearing=None):
        """pick_best=False: first start that reaches the target wins (requested lane, then opposite, then any).
        pick_best=True : no trustworthy lane was given, so try every start and keep the fastest
                         (including the time to reach that start node from the origin point).
        Returns (time, edges, index_of_source_used, start_bearing_used)."""
        last_error = Exception("Target unreachable")
        best = None
        for idx, (src_node, bearing) in enumerate(sources):
            try:
                base_time, route_edges = self.cch_query(
                    nodes, cch_edges, src_node, target, start_bearing=bearing, dest_bearing=dest_bearing
                )
            except Exception as exc:
                last_error = exc
                continue
            if not pick_best:
                return base_time, route_edges, idx, bearing
            reach = 0.0
            if origin is not None and src_node in nodes:
                reach = self.haversine_distance(origin[0], origin[1], nodes[src_node][0], nodes[src_node][1]) / self.DEFAULT_SPEED_MPS
            if best is None or base_time + reach < best[0]:
                best = (base_time + reach, base_time, route_edges, idx, bearing)
        if best is None:
            raise last_error
        return best[1], best[2], best[3], best[4]

    def compute_route(self, origin_lat, origin_lon, dest_lat, dest_lon, vehicle_layer="LOW", api_key=None,
                      flood_data=None, reports_data=None, user_heading=None,
                      lane_direction=None, switch_lane=False, dest_direction=None, user_speed=None):
        """
        Lane controls (all optional):
          user_heading   GPS heading in degrees, or a bound ("NB", "southbound", ...).
          lane_direction explicit user request ("NB"/"SB"/"EB"/"WB"/degrees). Overrides GPS heading.
          switch_lane    True = flip to the opposite carriageway of whatever lane was resolved above.
          dest_direction lane the destination sits on, so the trip ends on the right carriageway.
          user_speed     GPS speed in m/s. Under 1.5 the heading is treated as noise and ignored.
        """
        try:
            origin_lat, origin_lon = float(origin_lat), float(origin_lon)
            dest_lat, dest_lon = float(dest_lat), float(dest_lon)
        except (ValueError, TypeError):
            return {"status": "error", "message": "Invalid coordinates provided."}

        # A GPS heading is only a direction of travel while the vehicle is actually moving.
        stationary = False
        try:
            speed = float(user_speed)
            stationary = math.isfinite(speed) and speed < self.MIN_HEADING_SPEED_MPS
        except (TypeError, ValueError):
            pass
        gps_heading = user_heading if (not stationary or switch_lane) else None
        lane_bearing = self.resolve_lane_bearing(gps_heading, lane_direction, switch_lane)
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
        buffer_stages = list(self.BBOX_STAGES_DEG)
        route_edges = []
        base_time = 0
        lane_switched, used_bearing = False, lane_bearing
        flood_info = {}

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
                flood_info = dict(getattr(self, "last_flood_info", {}))
                self.apply_hazard_penalties(nodes, edges, reports_data)
            except Exception as e:
                # Never route on un-customised data: that sends drivers straight through floods.
                print(f"🚨 Flood customization failed: {e}")
                return {"status": "error", "message": f"Flood data processing failed: {e}"}

            try:
                zones = getattr(self, "last_flood_zones", [])
                active = self._clear_endpoint_candidates(origin_lat, origin_lon, nodes, sources, zones)
                targets = self._clear_endpoint_candidates(dest_lat, dest_lon, nodes, [(target, dest_bearing)], zones, max_extra=6)
                if not active or not targets:
                    raise KeyError("Start or destination lies outside the local graph")

                cch_edges = self.apply_cch_contraction(
                    nodes, edges, active[0][0], targets[0][0],
                    protected=[n for n, _ in active] + [n for n, _ in targets]
                )
                last_exc = Exception("Target unreachable")
                solved = None
                for tgt, tgt_bearing in targets:
                    try:
                        solved = self.query_with_lane_fallback(
                            nodes, cch_edges, active, tgt,
                            pick_best=(lane_bearing is None or len(active) > 1 and active[0][1] is None),
                            origin=(origin_lat, origin_lon), dest_bearing=tgt_bearing
                        )
                        break
                    except Exception as exc:
                        last_exc = exc
                if solved is None:
                    raise last_exc
                base_time, route_edges, used_idx, used_bearing = solved
                lane_switched = lane_bearing is not None and used_bearing is not None and used_bearing != lane_bearing

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
            result["flood_info"] = flood_info
            return result
        except Exception as e:
            return {"status": "error", "message": f"Failed compiling payload: {e}"}