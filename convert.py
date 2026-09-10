import sqlite3, pickle, json

print("⏳ Loading PKL into memory...")
with open("metro_manila_lite.pkl", "rb") as f:
    G = pickle.load(f)

print("🗄️ Creating SQLite Database...")
conn = sqlite3.connect("metro_manila.db")
c = conn.cursor()

c.execute('''CREATE TABLE IF NOT EXISTS nodes (id INTEGER PRIMARY KEY, lat REAL, lon REAL)''')
c.execute('''CREATE TABLE IF NOT EXISTS edges (u INTEGER, v INTEGER, length REAL, time REAL, geometry TEXT)''')

print("📥 Inserting nodes...")
c.executemany("INSERT OR IGNORE INTO nodes VALUES (?, ?, ?)", 
              [(n, d['y'], d['x']) for n, d in G.nodes(data=True)])

print("🛣️ Inserting edges...")
edges_to_insert = []
for u, v, k, d in G.edges(keys=True, data=True):
    geom = d.get('geometry', [])
    # Failsafe if Shapely coords snuck through
    if hasattr(geom, 'coords'): geom = list(geom.coords)
    time = float(d.get('travel_time', d.get('baseline_time', 0)))
    edges_to_insert.append((u, v, float(d.get('length', 0)), time, json.dumps(geom)))

c.executemany("INSERT INTO edges VALUES (?, ?, ?, ?, ?)", edges_to_insert)

print("⚡ Building spatial indexes (this makes queries instant)...")
c.execute("CREATE INDEX IF NOT EXISTS idx_lat_lon ON nodes(lat, lon)")
c.execute("CREATE INDEX IF NOT EXISTS idx_edges_u ON edges(u)")
c.execute("CREATE INDEX IF NOT EXISTS idx_edges_v ON edges(v)")

conn.commit()
conn.close()
print("✅ Done! You now have 'metro_manila.db'. Upload this to Render instead of the .pkl!")