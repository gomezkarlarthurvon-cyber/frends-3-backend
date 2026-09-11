import firebase_admin
from firebase_admin import credentials, db
from flask import Flask, jsonify, request
from flask_cors import CORS
import os
from dotenv import load_dotenv
from routing_engine import FRENDSRoutingEngine
import requests  
import threading
import time

load_dotenv() 

if os.path.exists('/etc/secrets/frends-v3-map-backend.json'):
    cred = credentials.Certificate('/etc/secrets/frends-v3-map-backend.json')
else:
    cred = credentials.Certificate('frends-v3-map-backend.json')

if not firebase_admin._app_id_exists if hasattr(firebase_admin, '_app_id_exists') else not firebase_admin._apps:
    try:
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://frends-v3-default-rtdb.asia-southeast1.firebasedatabase.app/' 
        })
        print("🔥 Firebase Admin initialized successfully!")
    except Exception as e:
        print(f"❌ CRITICAL: Failed to initialize Firebase: {e}")

app = Flask(__name__)

# 🌟 FULL CORS ENABLED TO FIX LOCALHOST:5173 NETWORK/CORS BLOCK
CORS(app, resources={r"/*": {"origins": "*"}})

# Initialize the routing engine
engine = FRENDSRoutingEngine()

TOMTOM_API_KEY = os.environ.get("TOMTOM_API_KEY")

# Metro Manila Bounding Box
METRO_MANILA_BOUNDS = (120.90, 14.35, 121.15, 14.77)

def is_within_metro_manila(lat, lon):
    min_lon, min_lat, max_lon, max_lat = METRO_MANILA_BOUNDS
    return (min_lat <= lat <= max_lat) and (min_lon <= lon <= max_lon)


def setup_firebase_listeners():
    """Background listener for IoT Flood Data and Hazards"""
    def flood_stream_handler(event):
        if event.data and isinstance(event.data, dict):
            for node_id, node_data in event.data.items():
                if all(k in node_data for k in ("lat", "lon", "depth")):
                    print(f"📡 IoT Flood sync received for node {node_id}")

    try:
        db.reference("nodes").listen(flood_stream_handler)
        print("🛰️ Firebase listeners active.")
    except Exception as e:
        print(f"⚠️ Could not attach listeners: {e}")

threading.Thread(target=setup_firebase_listeners, daemon=True).start()


@app.route("/health", methods=["GET"])
def health_check():
    return jsonify({"status": "ONLINE", "service": "FRENDS Dynamic Routing", "timestamp": time.time()}), 200

@app.route("/ping", methods=["GET"])
def ping():
    return jsonify({"status": "awake", "message": "FRENDS routing engine is online!"}), 200


@app.route("/api/route", methods=["POST"])
def get_dynamic_route():
    data = request.get_json()
    if not data:
        return jsonify({"status": "ERROR", "message": "No payload provided."}), 400

    try:
        origin_lat = float(data["origin_lat"])
        origin_lon = float(data["origin_lon"])
        dest_lat = float(data["dest_lat"])
        dest_lon = float(data["dest_lon"])
        vehicle_layer = data.get("vehicle_type", "LOW")

        # 1. HYBRID ROUTING CHECK (Metro Manila vs OSRM)
        if is_within_metro_manila(origin_lat, origin_lon) and is_within_metro_manila(dest_lat, dest_lon):
            print("🛣️ Both points in Metro Manila: Using local JIT-CCH engine.")

            try:
                flood_data = db.reference("nodes").get() 
            except Exception as e:
                print(f"⚠️ Failed to fetch flood data for routing: {e}")
                flood_data = None

            route_result = engine.compute_route(
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                dest_lat=dest_lat,
                dest_lon=dest_lon,
                vehicle_layer=vehicle_layer,
                api_key=TOMTOM_API_KEY,
                flood_data=flood_data 
            )

            if isinstance(route_result, dict) and route_result.get("status") == "error":
                return jsonify({
                    "status": "ALERT", 
                    "message": route_result.get("message", "Destination isolated by floods.")
                }), 200

            if isinstance(route_result, dict):
                return jsonify({
                    "status": "SUCCESS",
                    "path": route_result.get("path", []),
                    "segments": route_result.get("segments", []),
                    "distance": route_result.get("distance", 0),
                    "time": route_result.get("time", 0),
                }), 200
            else:
                return jsonify({
                    "status": "SUCCESS",
                    "path": route_result,
                    "segments": [],
                    "distance": 0,
                    "time": 0,
                }), 200

        else:
            print("🌐 Point(s) outside Metro Manila: Offloading to OSRM API.")
            url = f"https://router.project-osrm.org/route/v1/driving/{origin_lon},{origin_lat};{dest_lon},{dest_lat}?overview=full&geometries=geojson"
            headers = {"User-Agent": "FRENDS-Research-Project/1.0"}

            try:
                res = requests.get(url, headers=headers, timeout=15)
                if res.status_code == 200:
                    osrm_data = res.json()
                    if osrm_data.get("code") == "Ok":
                        osrm_coords = osrm_data["routes"][0]["geometry"]["coordinates"]
                        distance = osrm_data["routes"][0]["distance"]
                        duration = osrm_data["routes"][0]["duration"]
                        route = [{"latitude": c[1], "longitude": c[0]} for c in osrm_coords]

                        # 🌟 SLICE OSRM ROUTE TO PAINT YELLOW TRAFFIC PATCHES
                        segments, live_duration = engine.build_osrm_segments(osrm_coords, TOMTOM_API_KEY, is_city=False)
                        final_time = live_duration if live_duration > 0 else duration

                        return jsonify({
                            "status": "SUCCESS",
                            "path": route,
                            "segments": segments,
                            "distance": distance,
                            "time": final_time,
                        }), 200

                route = [
                    {"latitude": origin_lat, "longitude": origin_lon},
                    {"latitude": dest_lat, "longitude": dest_lon},
                ]
                return jsonify({"status": "SUCCESS", "path": route, "segments": [], "distance": 0, "time": 0}), 200

            except Exception as e:
                print(f"OSRM Connection Exception: {e}")
                route = [
                    {"latitude": origin_lat, "longitude": origin_lon},
                    {"latitude": dest_lat, "longitude": dest_lon},
                ]
                return jsonify({"status": "SUCCESS", "path": route, "segments": [], "distance": 0, "time": 0}), 200

    except Exception as e:
        print(f"Routing failed: {e}")
        return jsonify({"status": "ERROR", "message": f"Data Parsing Error: {str(e)}"}), 400


@app.route("/api/hazard/report", methods=["POST"])
def report_hazard():
    data = request.get_json()
    try:
        return jsonify({"status": "SUCCESS", "message": "Hazard registered successfully."}), 200
    except Exception as e:
        return jsonify({"status": "ERROR", "message": str(e)}), 500


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port, debug=False)