import firebase_admin
from firebase_admin import credentials, db
from flask import Flask, jsonify, request
from flask_cors import CORS
import os
from dotenv import load_dotenv  # 🌟 NEW IMPORT FOR SECURITY
from routing_engine import FRENDSRoutingEngine
import requests  
import threading
import time

# 🌟 Load hidden variables from .env (Local laptop only)
load_dotenv() 

# Check if Render's secret vault exists, otherwise use the local laptop file
if os.path.exists('/etc/secrets/frends-v3-map-backend.json'):
    cred = credentials.Certificate('/etc/secrets/frends-v3-map-backend.json')
else:
    cred = credentials.Certificate('frends-v3-map-backend.json')

# 🌟 ACTIVATE FIREBASE HERE 🌟
if not firebase_admin._apps:
    try:
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://frends-v3-default-rtdb.asia-southeast1.firebasedatabase.app/' 
        })
        print("🔥 Firebase Admin initialized successfully!")
    except Exception as e:
        print(f"❌ CRITICAL: Failed to initialize Firebase: {e}")

app = Flask(__name__)

# Updated CORS for local testing
CORS(app, resources={r"/*": {"origins": "*"}})

# Initialize the routing engine
engine = FRENDSRoutingEngine()

# 🔑 SECURE TOMTOM API KEY (Fetched from Render Dashboard or local .env)
TOMTOM_API_KEY = os.environ.get("TOMTOM_API_KEY")


# --- HYBRID ROUTING GEOFENCE LOGIC ---

# Approximate Bounding Box for Metro Manila (min_lon, min_lat, max_lon, max_lat)
METRO_MANILA_BOUNDS = (120.90, 14.35, 121.15, 14.77)


def is_within_metro_manila(lat, lon):
    """Checks if a given coordinate is strictly inside Metro Manila."""
    min_lon, min_lat, max_lon, max_lat = METRO_MANILA_BOUNDS
    return (min_lat <= lat <= max_lat) and (min_lon <= lon <= max_lon)


# --- FIREBASE SETUP ---


def initialize_firebase():
    try:
        if os.path.exists("firebase-service-account.json"):
            cred = credentials.Certificate("frends-v3-map-backend.json")
            firebase_admin.initialize_app(
                cred,
                {
                    # UPDATE THIS with your actual Firebase URL
                    "databaseURL": "https://frends-v3-default-rtdb.asia-southeast1.firebasedatabase.app"
                },
            )
            print("🔥 Firebase connected successfully.")
            setup_firebase_listeners()
        else:
            print(
                "⚠️ Firebase JSON key missing. Running server locally without Firebase sync."
            )
    except Exception as e:
        print(f"⚠️ Firebase initialization error: {e}")


def setup_firebase_listeners():
    """Background listener for IoT Flood Data and Hazards"""

    def flood_stream_handler(event):
        if event.data and isinstance(event.data, dict):
            for node_id, node_data in event.data.items():
                if all(k in node_data for k in ("lat", "lon", "depth")):
                    engine.update_flood_node(
                        node_data["lat"], node_data["lon"], node_data["depth"]
                    )

    try:
        db.reference("iot_nodes").listen(flood_stream_handler)
        print("🛰️ Firebase listeners active.")
    except Exception as e:
        print(f"⚠️ Could not attach listeners: {e}")


# Run Firebase setup in a background thread so it doesn't block startup
threading.Thread(target=initialize_firebase, daemon=True).start()


# --- API ENDPOINTS ---


@app.route("/health", methods=["GET"])
def health_check():
    """Keeps server awake and confirms connection with the frontend."""
    return (
        jsonify(
            {
                "status": "ONLINE",
                "service": "FRENDS Dynamic Routing",
                "timestamp": time.time(),
            }
        ),
        200,
    )


@app.route("/api/route", methods=["POST"])
def get_dynamic_route():
    data = request.get_json()
    if not data:
        return (
            jsonify({"status": "ERROR", "message": "No payload provided."}),
            400,
        )

    try:
        origin_lat = float(data["origin_lat"])
        origin_lon = float(data["origin_lon"])
        dest_lat = float(data["dest_lat"])
        dest_lon = float(data["dest_lon"])
        vehicle_layer = data.get("vehicle_type", "LOW")

        # 1. HYBRID ROUTING CHECK
        if is_within_metro_manila(
            origin_lat, origin_lon
        ) and is_within_metro_manila(dest_lat, dest_lon):
            print("🛣️ Both points in Metro Manila: Using local OSMnx engine.")

            # 🌟 Fetch current flood markers directly from Firebase RTDB
            try:
                from firebase_admin import db 
                flood_data = db.reference("nodes").get() 
            except Exception as e:
                print(f"⚠️ Failed to fetch flood data for routing: {e}")
                flood_data = None

            # Compute route using OSMnx + TomTom Traffic API + 🌟 Flood Data
            route_result = engine.compute_route(
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                dest_lat=dest_lat,
                dest_lon=dest_lon,
                vehicle_layer=vehicle_layer,
                api_key=TOMTOM_API_KEY,
                flood_data=flood_data 
            )

            if route_result is None:
                return (
                    jsonify(
                        {
                            "status": "ALERT",
                            "message": "Local Engine: No alternative path available due to flooding constraints.",
                        }
                    ),
                    200,
                )

            # Check if the engine deliberately threw a firewall error
            if isinstance(route_result, dict) and route_result.get("status") == "error":
                return (
                    jsonify(
                        {
                            "status": "ALERT", 
                            "message": route_result.get("message", "Destination isolated by floods.")
                        }
                    ),
                    200,
                )

            if isinstance(route_result, dict):
                return (
                    jsonify(
                        {
                            "status": "SUCCESS",
                            "path": route_result.get("path", []),         # FIXED KEY
                            "segments": route_result.get("segments", []), 
                            "distance": route_result.get("distance", 0),  # FIXED KEY
                            "time": route_result.get("time", 0),          # FIXED KEY
                        }
                    ),
                    200,
                )
            else:
                # Fallback format handling
                return (
                    jsonify(
                        {
                            "status": "SUCCESS",
                            "path": route_result,  # FIXED KEY
                            "segments": [],
                            "distance": 0,         # FIXED KEY
                            "time": 0,             # FIXED KEY
                        }
                    ),
                    200,
                )

        else:
            print("🌐 Point(s) outside Metro Manila: Offloading to OSRM API.")
            
            # 2. OSRM ROUTING (For points outside Metro Manila)
            url = f"https://router.project-osrm.org/route/v1/driving/{origin_lon},{origin_lat};{dest_lon},{dest_lat}?overview=full&geometries=geojson"
            headers = {"User-Agent": "FRENDS-Research-Project/1.0"}

            try:
                res = requests.get(url, headers=headers, timeout=15)

                if res.status_code == 200:
                    osrm_data = res.json()
                    if osrm_data.get("code") == "Ok":
                        osrm_coords = osrm_data["routes"][0]["geometry"][
                            "coordinates"
                        ]
                        distance = osrm_data["routes"][0]["distance"]  # meters
                        duration = osrm_data["routes"][0]["duration"]  # seconds

                        route = [
                            {"latitude": c[1], "longitude": c[0]}
                            for c in osrm_coords
                        ]

                        return (
                            jsonify(
                                {
                                    "status": "SUCCESS",
                                    "path": route,        # FIXED KEY
                                    "segments": [],
                                    "distance": distance, # FIXED KEY
                                    "time": duration,     # FIXED KEY
                                }
                            ),
                            200,
                        )

                # Fallback straight line if external OSRM fails
                print(
                    "⚠️ OSRM public server failed. Using geometric fallback bridge."
                )
                route = [
                    {"latitude": origin_lat, "longitude": origin_lon},
                    {"latitude": dest_lat, "longitude": dest_lon},
                ]
                return (
                    jsonify(
                        {
                            "status": "SUCCESS",
                            "path": route,    # FIXED KEY
                            "segments": [],
                            "distance": 0,    # FIXED KEY
                            "time": 0,        # FIXED KEY
                        }
                    ),
                    200,
                )

            except Exception as e:
                print(f"OSRM Connection Exception: {e}")
                route = [
                    {"latitude": origin_lat, "longitude": origin_lon},
                    {"latitude": dest_lat, "longitude": dest_lon},
                ]
                return (
                    jsonify(
                        {
                            "status": "SUCCESS",
                            "path": route,    # FIXED KEY
                            "segments": [],
                            "distance": 0,    # FIXED KEY
                            "time": 0,        # FIXED KEY
                        }
                    ),
                    200,
                )

    except Exception as e:
        print(f"Routing failed: {e}")
        return (
            jsonify(
                {
                    "status": "ERROR",
                    "message": f"Data Parsing Error: {str(e)}",
                }
            ),
            400,
        )


@app.route("/api/hazard/report", methods=["POST"])
def report_hazard():
    """Endpoint for crowdsourced hazard pins."""
    data = request.get_json()
    try:
        engine.register_crowdsourced_hazard(
            lat=float(data["lat"]),
            lon=float(data["lon"]),
            hazard_type=data["category"],
        )
        return jsonify({"status": "SUCCESS"}), 200
    except Exception as e:
        return jsonify({"status": "ERROR", "message": str(e)}), 500


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port, debug=False)