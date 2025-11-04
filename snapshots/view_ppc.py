import open3d as o3d
import numpy as np
from pathlib import Path

# === Directorio base ===
SNAP_DIR = Path("/home/enrique/ros2_ws/src/lidar_camera_fusion/snapshots")

# === Buscar la carpeta más reciente ===
subdirs = [d for d in SNAP_DIR.iterdir() if d.is_dir()]
if not subdirs:
    raise RuntimeError("❌ No hay carpetas de snapshot disponibles.")

# Filtrar solo las que tengan formato correcto (YYYY-MM-DD_HH-MM-SS)
valid_dirs = [
    d for d in subdirs
    if len(d.name) >= 19
    and d.name[4] == '-'
    and d.name[7] == '-'
    and '_' in d.name
    and d.name[13] == '-'
]

if not valid_dirs:
    raise RuntimeError("❌ No se encontraron carpetas válidas con formato de fecha.")

# Ordenar alfabéticamente → equivale a ordenar cronológicamente
latest_dir = sorted(valid_dirs, key=lambda d: d.name)[-1]
processed_dir = latest_dir / "processed"

print(f"📂 Última carpeta detectada por nombre: {latest_dir.name}")

# === Cargar nubes ===
def safe_load(path):
    if path.exists():
        data = np.load(path)
        if data.size > 0:
            return data
        else:
            return None
    else:
        return None
    
points_all = safe_load(next(latest_dir.glob("*_points.npy")))
points_veg_clean = safe_load(processed_dir / "smallveg_clean.npy")
points_veg_all = safe_load(processed_dir / "smallveg_orig.npy")
points_veg_right = safe_load(processed_dir / "veg_right.npy")
points_veg_left = safe_load(processed_dir / "veg_left.npy")
points_road_center = safe_load(processed_dir / "road_center.npy")

# === Crear lista de geometrías ===
geoms = []

def add_pcd(points, color):
    if points is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color(color)
        geoms.append(pcd)

# === Añadir nubes existentes ===
add_pcd(points_all, [0.5, 0.5, 0.5])     # gris
add_pcd(points_veg_all, [1.0, 0.5, 0.0]) # naranja
add_pcd(points_veg_clean, [0.0, 1.0, 0.0]) # verde
add_pcd(points_veg_right, [0.0, 0.5, 1.0]) # azul claro
add_pcd(points_veg_left, [0.5, 0.5, 1.0])  # violeta
add_pcd(points_road_center, [1.0, 0.0, 0.0]) # rojo

if not geoms:
    raise RuntimeError("❌ No hay nubes válidas para visualizar.")

# === Visualizar ===
print(f"🟢 Mostrando {len(geoms)} nubes en Open3D...")
o3d.visualization.draw_geometries(geoms)

