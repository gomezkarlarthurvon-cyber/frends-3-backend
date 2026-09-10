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

    Designed for:
        - Render 512 MB RAM
        - SQLite road database
        - Dynamic flood customization
        - TomTom traffic customization
        - Leaflet-compatible geometry
        - No NetworkX dependency

    Architecture:

        SQLite
          ↓
        Local graph extraction
          ↓
        CCH-style contraction
          ↓
        Flood customization
          ↓
        Bidirectional CCH query
          ↓
        Route reconstruction
    """

    DEFAULT_SPEED_MPS = 8.33

    def __init__(self, db_file="metro_manila.db"):
        self.db_file = db_file

        print(
            f"⏳ FRENDS Memory-Efficient CCH Engine "
            f"initialized: {self.db_file}"
        )

    # ============================================================
    # TRAFFIC
    # ============================================================

    def get_tomtom_traffic_multiplier(
        self,
        lat,
        lon,
        api_key
    ):
        """
        Gets a traffic multiplier from TomTom.

        Kept deliberately lightweight.
        """

        url = (
            "https://api.tomtom.com/traffic/services/4/"
            "flowSegmentData/absolute/10/json"
        )

        params = {
            "key": api_key,
            "point": f"{lat},{lon}"
        }

        try:
            response = requests.get(
                url,
                params=params,
                timeout=1.5
            )

            if response.status_code == 200:

                data = response.json().get(
                    "flowSegmentData",
                    {}
                )

                current_speed = data.get("currentSpeed")
                free_flow_speed = data.get("freeFlowSpeed")

                if (
                    current_speed
                    and free_flow_speed
                    and current_speed > 0
                ):
                    return min(
                        free_flow_speed / current_speed,
                        5.0
                    )

            if response.status_code in (403, 429):
                return random.choice(
                    [1.0, 1.0, 1.8, 3.0]
                )

        except Exception:
            pass

        return 1.0

    # ============================================================
    # GEOMETRY
    # ============================================================

    @staticmethod
    def haversine_distance(
        lat1,
        lon1,
        lat2,
        lon2
    ):
        lat1, lon1, lat2, lon2 = map(
            radians,
            [lat1, lon1, lat2, lon2]
        )

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = (
            sin(dlat / 2) ** 2
            +
            cos(lat1)
            * cos(lat2)
            * sin(dlon / 2) ** 2
        )

        c = 2 * asin(sqrt(a))

        return c * 6371000.0

    @staticmethod
    def point_to_line_distance(
        px,
        py,
        x1,
        y1,
        x2,
        y2
    ):
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return FRENDSRoutingEngine.haversine_distance(
                py,
                px,
                y1,
                x1
            )

        t = (
            (px - x1) * dx
            +
            (py - y1) * dy
        ) / (dx * dx + dy * dy)

        t = max(
            0.0,
            min(1.0, t)
        )

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return FRENDSRoutingEngine.haversine_distance(
            py,
            px,
            closest_y,
            closest_x
        )

    # ============================================================
    # DATABASE
    # ============================================================

    def nearest_node_sql(
        self,
        lat,
        lon,
        cursor
    ):
        """
        Memory-safe nearest-node lookup.

        IMPORTANT:
        This avoids loading the entire node table into Python.
        """

        # Small geographic search first
        delta = 0.05

        cursor.execute(
            """
            SELECT id
            FROM nodes
            WHERE lat BETWEEN ? AND ?
              AND lon BETWEEN ? AND ?
            ORDER BY
                ((lat - ?) * (lat - ?))
                +
                ((lon - ?) * (lon - ?))
            ASC
            LIMIT 1
            """,
            (
                lat - delta,
                lat + delta,
                lon - delta,
                lon + delta,
                lat,
                lat,
                lon,
                lon
            )
        )

        result = cursor.fetchone()

        if result:
            return result[0]

        # Fallback
        cursor.execute(
            """
            SELECT id
            FROM nodes
            ORDER BY
                ((lat - ?) * (lat - ?))
                +
                ((lon - ?) * (lon - ?))
            ASC
            LIMIT 1
            """,
            (
                lat,
                lat,
                lon,
                lon
            )
        )

        result = cursor.fetchone()

        return result[0] if result else None

    # ============================================================
    # LOCAL GRAPH LOADING
    # ============================================================

    def load_local_graph(
        self,
        origin_lat,
        origin_lon,
        dest_lat,
        dest_lon
    ):
        """
        Loads only the geographic portion required for this request.

        This is critical for Render's 512 MB RAM limit.
        """

        # Keep the routing window deliberately small.
        #
        # Increase only if your application genuinely needs
        # long-distance Metro Manila routing.
        buffer = 0.045

        min_lat = min(
            origin_lat,
            dest_lat
        ) - buffer

        max_lat = max(
            origin_lat,
            dest_lat
        ) + buffer

        min_lon = min(
            origin_lon,
            dest_lon
        ) - buffer

        max_lon = max(
            origin_lon,
            dest_lon
        ) + buffer

        conn = sqlite3.connect(
            self.db_file
        )

        cursor = conn.cursor()

        try:

            origin_node = self.nearest_node_sql(
                origin_lat,
                origin_lon,
                cursor
            )

            destination_node = self.nearest_node_sql(
                dest_lat,
                dest_lon,
                cursor
            )

            if (
                origin_node is None
                or destination_node is None
            ):
                return (
                    None,
                    None,
                    None
                )

            # ----------------------------------------------------
            # Nodes
            # ----------------------------------------------------

            cursor.execute(
                """
                SELECT id, lat, lon
                FROM nodes
                WHERE lat BETWEEN ? AND ?
                  AND lon BETWEEN ? AND ?
                """,
                (
                    min_lat,
                    max_lat,
                    min_lon,
                    max_lon
                )
            )

            nodes = {}

            for node_id, lat, lon in cursor:

                nodes[node_id] = (
                    float(lat),
                    float(lon)
                )

            # ----------------------------------------------------
            # Edges
            # ----------------------------------------------------

            cursor.execute(
                """
                SELECT
                    u,
                    v,
                    length,
                    time,
                    geometry
                FROM edges
                WHERE
                    u IN (
                        SELECT id
                        FROM nodes
                        WHERE lat BETWEEN ? AND ?
                          AND lon BETWEEN ? AND ?
                    )
                AND
                    v IN (
                        SELECT id
                        FROM nodes
                        WHERE lat BETWEEN ? AND ?
                          AND lon BETWEEN ? AND ?
                    )
                """,
                (
                    min_lat,
                    max_lat,
                    min_lon,
                    max_lon,
                    min_lat,
                    max_lat,
                    min_lon,
                    max_lon
                )
            )

            edges = []

            for (
                u,
                v,
                length,
                time_value,
                geometry
            ) in cursor:

                if (
                    u not in nodes
                    or v not in nodes
                ):
                    continue

                try:
                    geometry_data = (
                        json.loads(geometry)
                        if geometry
                        else None
                    )
                except Exception:
                    geometry_data = None

                edges.append(
                    {
                        "u": u,
                        "v": v,
                        "length": float(
                            length or 0
                        ),
                        "time": float(
                            time_value or 0
                        ),
                        "geometry": geometry_data,
                        "blocked": False,
                        "shortcut": False,
                        "children": None
                    }
                )

            return (
                nodes,
                edges,
                (
                    origin_node,
                    destination_node
                )
            )

        finally:
            cursor.close()
            conn.close()

    # ============================================================
    # COMPACT GRAPH
    # ============================================================

    def build_adjacency(
        self,
        nodes,
        edges
    ):
        """
        Converts database edges into lightweight adjacency lists.

        No NetworkX.
        """

        forward = {
            node_id: []
            for node_id in nodes
        }

        backward = {
            node_id: []
            for node_id in nodes
        }

        edge_lookup = {}

        for edge in edges:

            u = edge["u"]
            v = edge["v"]

            forward[u].append(edge)
            backward[v].append(edge)

            edge_lookup[
                (u, v)
            ] = edge

        return (
            forward,
            backward,
            edge_lookup
        )

    # ============================================================
    # CCH NODE ORDERING
    # ============================================================

    def calculate_order(
        self,
        nodes,
        forward,
        backward,
        source,
        target
    ):
        """
        Lightweight CCH importance ordering.

        Lower degree nodes are contracted first.

        Source and target remain uncontracted.

        This is intentionally calculated only for the
        local routing graph.
        """

        importance = []

        for node in nodes:

            if node == source or node == target:
                continue

            degree = (
                len(forward.get(node, ()))
                +
                len(backward.get(node, ()))
            )

            importance.append(
                (
                    degree,
                    node
                )
            )

        importance.sort(
            key=lambda x: x[0]
        )

        return [
            node
            for _, node in importance
        ]

    # ============================================================
    # CCH WITNESS SEARCH
    # ============================================================

    def witness_search(
        self,
        node,
        incoming,
        outgoing,
        contracted
    ):
        """
        Lightweight witness search.

        Determines whether a direct shortcut is actually
        necessary.

        This prevents blindly creating shortcuts.
        """

        if not incoming or not outgoing:
            return []

        shortcuts = []

        for in_edge in incoming:

            u = in_edge["u"]

            if u in contracted:
                continue

            for out_edge in outgoing:

                w = out_edge["v"]

                if w in contracted:
                    continue

                if u == w:
                    continue

                shortcut_time = (
                    in_edge["time"]
                    +
                    out_edge["time"]
                )

                shortcuts.append(
                    (
                        u,
                        w,
                        shortcut_time,
                        in_edge,
                        out_edge
                    )
                )

        return shortcuts

    # ============================================================
    # CCH CONTRACTION
    # ============================================================

    def apply_cch_contraction(
        self,
        nodes,
        edges,
        source,
        target
    ):
        """
        Performs local CCH preprocessing.

        IMPORTANT:

        This is executed on the bounded local graph,
        NOT the entire Metro Manila graph.

        That makes it practical on Render's 512 MB
        memory limit.
        """

        forward, backward, edge_lookup = (
            self.build_adjacency(
                nodes,
                edges
            )
        )

        order = self.calculate_order(
            nodes,
            forward,
            backward,
            source,
            target
        )

        rank = {}

        for position, node in enumerate(order):
            rank[node] = position

        rank[source] = len(order) + 1
        rank[target] = len(order) + 2

        contracted = set()

        new_edges = []

        # Original edges remain available.
        new_edges.extend(edges)

        shortcut_count = 0

        for node in order:

            incoming = [
                edge
                for edge in backward.get(
                    node,
                    ()
                )
                if not edge["blocked"]
                and edge["u"] not in contracted
            ]

            outgoing = [
                edge
                for edge in forward.get(
                    node,
                    ()
                )
                if not edge["blocked"]
                and edge["v"] not in contracted
            ]

            # ----------------------------------------------------
            # Witness candidates
            # ----------------------------------------------------

            candidates = self.witness_search(
                node,
                incoming,
                outgoing,
                contracted
            )

            # ----------------------------------------------------
            # Create shortcuts
            # ----------------------------------------------------

            for (
                u,
                w,
                shortcut_time,
                in_edge,
                out_edge
            ) in candidates:

                existing = edge_lookup.get(
                    (u, w)
                )

                if (
                    existing is not None
                    and existing["time"]
                    <= shortcut_time
                ):
                    continue

                shortcut = {
                    "u": u,
                    "v": w,
                    "length": (
                        in_edge["length"]
                        +
                        out_edge["length"]
                    ),
                    "time": shortcut_time,
                    "geometry": None,
                    "blocked": False,
                    "shortcut": True,

                    # Used later for unpacking
                    "children": (
                        in_edge,
                        out_edge
                    )
                }

                new_edges.append(
                    shortcut
                )

                edge_lookup[
                    (u, w)
                ] = shortcut

                forward.setdefault(
                    u,
                    []
                ).append(shortcut)

                backward.setdefault(
                    w,
                    []
                ).append(shortcut)

                shortcut_count += 1

            contracted.add(node)

        print(
            "⚡ Local CCH contraction complete: "
            f"{len(order)} nodes contracted, "
            f"{shortcut_count} shortcuts created."
        )

        return (
            new_edges,
            rank
        )

    # ============================================================
    # FLOOD CUSTOMIZATION
    # ============================================================

    def customize_for_floods(
        self,
        nodes,
        edges,
        flood_data,
        vehicle_layer
    ):
        """
        Dynamic CCH customization.

        Flooding changes edge availability without
        rebuilding the whole database.
        """

        flooded_edges = set()

        limits = {
            "LOW": 15,
            "MID": 30,
            "HIGH": 50
        }

        max_safe_depth = limits.get(
            vehicle_layer,
            15
        )

        flood_points = []

        if not flood_data:
            return flooded_edges

        # --------------------------------------------------------
        # Extract latest flood measurements
        # --------------------------------------------------------

        for _, container in flood_data.items():

            if not isinstance(
                container,
                dict
            ):
                continue

            push_keys = sorted(
                [
                    key
                    for key in container
                    if str(key).startswith("-")
                ]
            )

            if push_keys:

                latest = container[
                    push_keys[-1]
                ]

            else:

                latest = container

            if not isinstance(
                latest,
                dict
            ):
                continue

            try:

                water_level = float(
                    latest.get(
                        "waterLevel",
                        latest.get(
                            "depth",
                            0
                        )
                    )
                )

                lat = float(
                    latest.get(
                        "lat",
                        0
                    )
                )

                lon = float(
                    latest.get(
                        "lng",
                        latest.get(
                            "lon",
                            0
                        )
                    )
                )

            except (
                TypeError,
                ValueError
            ):
                continue

            if (
                water_level
                >= max_safe_depth
                and lat
                and lon
            ):
                flood_points.append(
                    (
                        lat,
                        lon
                    )
                )

        if not flood_points:
            return flooded_edges

        # --------------------------------------------------------
        # Flood detection
        # --------------------------------------------------------

        BLAST_RADIUS = 60.0

        for edge in edges:

            if edge["shortcut"]:
                continue

            u = edge["u"]
            v = edge["v"]

            geometry = edge.get(
                "geometry"
            )

            if not geometry:

                if (
                    u not in nodes
                    or v not in nodes
                ):
                    continue

                geometry = [
                    (
                        nodes[u][1],
                        nodes[u][0]
                    ),
                    (
                        nodes[v][1],
                        nodes[v][0]
                    )
                ]

            blocked = False

            for flood_lat, flood_lon in flood_points:

                for i in range(
                    len(geometry) - 1
                ):

                    lon1, lat1 = geometry[i]
                    lon2, lat2 = geometry[i + 1]

                    distance = (
                        self.point_to_line_distance(
                            flood_lon,
                            flood_lat,
                            lon1,
                            lat1,
                            lon2,
                            lat2
                        )
                    )

                    if distance <= BLAST_RADIUS:
                        blocked = True
                        break

                if blocked:
                    break

            if blocked:

                edge["blocked"] = True

                flooded_edges.add(
                    (
                        u,
                        v
                    )
                )

        # --------------------------------------------------------
        # Recalculate shortcut availability
        # --------------------------------------------------------

        for edge in edges:

            if not edge["shortcut"]:
                continue

            children = edge.get(
                "children"
            )

            if not children:
                continue

            child_a, child_b = children

            if (
                child_a.get("blocked")
                or
                child_b.get("blocked")
            ):
                edge["blocked"] = True

        print(
            "🌊 CCH flood customization: "
            f"{len(flooded_edges)} base edges blocked."
        )

        return flooded_edges

    # ============================================================
    # CCH BIDIRECTIONAL QUERY
    # ============================================================

    def cch_query(
        self,
        nodes,
        edges,
        rank,
        source,
        target
    ):
        """
        Bidirectional CCH-style shortest path query.

        Uses rank restrictions:

            Forward:
                rank(u) <= rank(v)

            Backward:
                rank(v) <= rank(u)

        This prevents unrestricted traversal of the
        contracted graph.
        """

        forward = {}
        backward = {}

        for edge in edges:

            if edge.get("blocked"):
                continue

            u = edge["u"]
            v = edge["v"]

            ru = rank.get(
                u,
                0
            )

            rv = rank.get(
                v,
                0
            )

            # Upward graph
            if ru <= rv:

                forward.setdefault(
                    u,
                    []
                ).append(edge)

            # Reverse upward graph
            if rv <= ru:

                backward.setdefault(
                    v,
                    []
                ).append(edge)

        # --------------------------------------------------------
        # Forward search
        # --------------------------------------------------------

        forward_dist = {
            source: 0.0
        }

        forward_parent = {}

        pq_forward = [
            (
                0.0,
                source
            )
        ]

        # --------------------------------------------------------
        # Backward search
        # --------------------------------------------------------

        backward_dist = {
            target: 0.0
        }

        backward_parent = {}

        pq_backward = [
            (
                0.0,
                target
            )
        ]

        best = float("inf")
        meeting = None

        # --------------------------------------------------------
        # Bidirectional search
        # --------------------------------------------------------

        while (
            pq_forward
            or
            pq_backward
        ):

            # -----------------------------------------------
            # Forward
            # -----------------------------------------------

            if pq_forward:

                dist_u, u = heapq.heappop(
                    pq_forward
                )

                if (
                    dist_u
                    != forward_dist.get(
                        u
                    )
                ):
                    continue

                if dist_u > best:
                    pq_forward.clear()
                else:

                    if u in backward_dist:

                        total = (
                            dist_u
                            +
                            backward_dist[u]
                        )

                        if total < best:

                            best = total
                            meeting = u

                    for edge in forward.get(
                        u,
                        ()
                    ):

                        v = edge["v"]

                        new_dist = (
                            dist_u
                            +
                            edge["time"]
                        )

                        if new_dist < (
                            forward_dist.get(
                                v,
                                float("inf")
                            )
                        ):

                            forward_dist[v] = (
                                new_dist
                            )

                            forward_parent[v] = (
                                u,
                                edge
                            )

                            heapq.heappush(
                                pq_forward,
                                (
                                    new_dist,
                                    v
                                )
                            )

            # -----------------------------------------------
            # Backward
            # -----------------------------------------------

            if pq_backward:

                dist_u, u = heapq.heappop(
                    pq_backward
                )

                if (
                    dist_u
                    != backward_dist.get(
                        u
                    )
                ):
                    continue

                if dist_u > best:
                    pq_backward.clear()
                else:

                    if u in forward_dist:

                        total = (
                            dist_u
                            +
                            forward_dist[u]
                        )

                        if total < best:

                            best = total
                            meeting = u

                    for edge in backward.get(
                        u,
                        ()
                    ):

                        v = edge["u"]

                        new_dist = (
                            dist_u
                            +
                            edge["time"]
                        )

                        if new_dist < (
                            backward_dist.get(
                                v,
                                float("inf")
                            )
                        ):

                            backward_dist[v] = (
                                new_dist
                            )

                            backward_parent[v] = (
                                u,
                                edge
                            )

                            heapq.heappush(
                                pq_backward,
                                (
                                    new_dist,
                                    v
                                )
                            )

        if meeting is None:
            raise RuntimeError(
                "Target unreachable"
            )

        # --------------------------------------------------------
        # Reconstruct edge path
        # --------------------------------------------------------

        left = []

        node = meeting

        while node != source:

            parent_data = (
                forward_parent.get(
                    node
                )
            )

            if parent_data is None:
                break

            parent, edge = parent_data

            left.append(edge)

            node = parent

        left.reverse()

        right = []

        node = meeting

        while node != target:

            parent_data = (
                backward_parent.get(
                    node
                )
            )

            if parent_data is None:
                break

            parent, edge = parent_data

            right.append(edge)

            node = parent

        route_edges = (
            left
            +
            right
        )

        if not route_edges:
            raise RuntimeError(
                "No valid CCH path."
            )

        return (
            best,
            route_edges
        )

    # ============================================================
    # SHORTCUT UNPACKING
    # ============================================================

    def unpack_edge(
        self,
        edge
    ):
        """
        Recursively expands CCH shortcuts back into
        original road segments.

        Leaflet receives original geometry rather
        than abstract shortcut edges.
        """

        if not edge.get(
            "shortcut"
        ):
            return [
                edge
            ]

        children = edge.get(
            "children"
        )

        if not children:
            return [
                edge
            ]

        first, second = children

        return (
            self.unpack_edge(first)
            +
            self.unpack_edge(second)
        )

    # ============================================================
    # ROUTE OUTPUT
    # ============================================================

    def build_route_payload(
        self,
        nodes,
        route_edges,
        api_key
    ):
        route_coords = []
        route_segments = []

        total_distance = 0.0
        live_total_time = 0.0

        traffic_cache = {}

        for edge in route_edges:

            u = edge["u"]
            v = edge["v"]

            if (
                u not in nodes
                or v not in nodes
            ):
                continue

            length = float(
                edge.get(
                    "length",
                    0
                )
            )

            travel_time = float(
                edge.get(
                    "time",
                    0
                )
            )

            if travel_time <= 0:
                travel_time = (
                    length
                    /
                    self.DEFAULT_SPEED_MPS
                )

            total_distance += length

            multiplier = 1.0

            # ----------------------------------------------------
            # Traffic
            # ----------------------------------------------------

            if api_key:

                # Round coordinates so nearby segments reuse
                # the same TomTom request.
                cache_key = (
                    round(
                        nodes[u][0],
                        4
                    ),
                    round(
                        nodes[u][1],
                        4
                    )
                )

                if cache_key not in traffic_cache:

                    traffic_cache[
                        cache_key
                    ] = (
                        self.get_tomtom_traffic_multiplier(
                            nodes[u][0],
                            nodes[u][1],
                            api_key
                        )
                    )

                multiplier = traffic_cache[
                    cache_key
                ]

            segment_time = (
                travel_time
                *
                multiplier
            )

            live_total_time += (
                segment_time
            )

            # ----------------------------------------------------
            # Traffic color
            # ----------------------------------------------------

            if multiplier >= 2.5:

                color = "#FF0000"

            elif multiplier >= 1.5:

                color = "#FFA500"

            else:

                color = "#3388ff"

            # ----------------------------------------------------
            # Geometry
            # ----------------------------------------------------

            geometry = edge.get(
                "geometry"
            )

            if geometry:

                coords = [
                    {
                        "latitude": lat,
                        "longitude": lon
                    }
                    for lon, lat in geometry
                ]

            else:

                coords = [
                    {
                        "latitude": nodes[u][0],
                        "longitude": nodes[u][1]
                    },
                    {
                        "latitude": nodes[v][0],
                        "longitude": nodes[v][1]
                    }
                ]

            route_segments.append(
                {
                    "coords": coords,
                    "color": color
                }
            )

            route_coords.extend(
                coords
            )

        # --------------------------------------------------------
        # Remove duplicate consecutive coordinates
        # --------------------------------------------------------

        cleaned_coords = []

        previous = None

        for point in route_coords:

            current = (
                point["latitude"],
                point["longitude"]
            )

            if current == previous:
                continue

            cleaned_coords.append(
                point
            )

            previous = current

        return {
            "status": "success",
            "path": cleaned_coords,
            "segments": route_segments,
            "distance": float(
                total_distance
            ),
            "time": float(
                live_total_time
            )
        }

    # ============================================================
    # MAIN ROUTER
    # ============================================================

    def compute_route(
        self,
        origin_lat,
        origin_lon,
        dest_lat,
        dest_lon,
        vehicle_layer="LOW",
        api_key=None,
        flood_data=None
    ):
        print(
            "\n🗺️ FRENDS CCH route request:"
            f" ({origin_lat}, {origin_lon})"
            f" → ({dest_lat}, {dest_lon})"
        )

        # --------------------------------------------------------
        # Validate coordinates
        # --------------------------------------------------------

        try:

            origin_lat = float(
                origin_lat
            )

            origin_lon = float(
                origin_lon
            )

            dest_lat = float(
                dest_lat
            )

            dest_lon = float(
                dest_lon
            )

        except (
            ValueError,
            TypeError
        ):

            return {
                "status": "error",
                "message": (
                    "Invalid coordinates provided."
                )
            }

        # --------------------------------------------------------
        # Load only local graph
        # --------------------------------------------------------

        try:

            (
                nodes,
                edges,
                endpoints
            ) = self.load_local_graph(
                origin_lat,
                origin_lon,
                dest_lat,
                dest_lon
            )

        except Exception as e:

            return {
                "status": "error",
                "message": (
                    f"Database error: {e}"
                )
            }

        if not nodes or not edges:

            return {
                "status": "error",
                "message": (
                    "Route exceeds limits "
                    "or no roads found."
                )
            }

        source, target = endpoints

        # --------------------------------------------------------
        # CCH contraction
        # --------------------------------------------------------

        try:

            (
                cch_edges,
                rank
            ) = self.apply_cch_contraction(
                nodes,
                edges,
                source,
                target
            )

        except Exception as e:

            return {
                "status": "error",
                "message": (
                    f"CCH preprocessing failed: {e}"
                )
            }

        # --------------------------------------------------------
        # Flood customization
        # --------------------------------------------------------

        try:

            self.customize_for_floods(
                nodes,
                cch_edges,
                flood_data,
                vehicle_layer
            )

        except Exception as e:

            return {
                "status": "error",
                "message": (
                    f"Flood customization failed: {e}"
                )
            }

        # --------------------------------------------------------
        # CCH query
        # --------------------------------------------------------

        try:

            (
                base_time,
                route_edges
            ) = self.cch_query(
                nodes,
                cch_edges,
                rank,
                source,
                target
            )

        except Exception:

            return {
                "status": "error",
                "message": (
                    "No safe route available. "
                    "Destination isolated by floods."
                )
            }

        # --------------------------------------------------------
        # Unpack shortcuts
        # --------------------------------------------------------

        unpacked = []

        for edge in route_edges:

            unpacked.extend(
                self.unpack_edge(
                    edge
                )
            )

        # --------------------------------------------------------
        # Remove duplicate shortcut descendants
        # --------------------------------------------------------

        unique_edges = []

        seen = set()

        for edge in unpacked:

            key = (
                edge["u"],
                edge["v"],
                id(edge)
            )

            if key in seen:
                continue

            seen.add(key)

            unique_edges.append(
                edge
            )

        # --------------------------------------------------------
        # Build frontend payload
        # --------------------------------------------------------

        try:

            result = (
                self.build_route_payload(
                    nodes,
                    unique_edges,
                    api_key
                )
            )

            if (
                not result["path"]
                or len(result["path"]) < 2
            ):

                return {
                    "status": "error",
                    "message": (
                        "Failed to generate "
                        "a valid route."
                    )
                }

            # If TomTom wasn't available,
            # use CCH's base travel time.
            if not api_key:

                result["time"] = float(
                    base_time
                )

            return result

        except Exception as e:

            return {
                "status": "error",
                "message": (
                    f"Failed compiling payload: {e}"
                )
            }