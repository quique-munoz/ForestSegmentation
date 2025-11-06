#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fusiona LiDAR + RFDETRSegPreview para obtener puntos 3D de la clase 'smallveg'
en coordenadas del LiDAR y generar métricas por lado, obteniendo el eje del camino.
"""
# ========= IMPORTS =========
import os, json, cv2, torch, numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from datetime import datetime
import pandas as pd
from rfdetr import RFDETRSegPreview
import supervision as sv
from PIL import Image
import warnings
from torch.jit import TracerWarning
import subprocess

# ========= WARNINGS =========
warnings.filterwarnings("ignore", category=TracerWarning)
warnings.filterwarnings("ignore", message=r"torch\.meshgrid:.*indexing")

# ========= CONFIG =========
SNAP_DIR = Path('/home/enrique/ros2_ws/src/forest_segmentation/snapshots')
CHECKPOINT_PATH = '/home/enrique/ros2_ws/src/forest_segmentation/runs/checkpoint_best_ema.pth'
CLASS_NAMES = ["background", "forest", "obstacle", "rough", "sky", "smallveg", "smooth"]

SMALLVEG_CLASS = 5
SMOOTH_CLASS = 6
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

CFG = {
    "segment_length": 0.1,
    "max_forward_range": 20.0,
    #"threshold_start": 10.0,
    "nb_neighbors": 25,
    "std_ratio": 1.0, 
    "z_min": -8.0,
    "z_max": 8.0,
    "device": "cuda"
}

# ========= UTILS =========
def get_latest_snapshot_folder():
    """
    Obtiene la carpeta de snapshot más reciente dentro de SNAP_ROOT,
    ordenando únicamente por el nombre (formato YYYY-MM-DD_HH-MM-SS).
    """
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

    print(f"📂 Última carpeta detectada por nombre: {latest_dir.name}")
    return latest_dir

def create_output_dir(snapshot_dir, latest_name):
    """Crea carpeta processed dentro de la carpeta del snapshot."""
    proc_dir = snapshot_dir / "processed"
    proc_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 Carpeta de salida: {proc_dir}")
    return proc_dir

def load_snapshot(snapshot_dir):
    """Carga los archivos del último snapshot dentro de la carpeta dada."""
    npys = sorted(snapshot_dir.glob('*_points.npy'))
    if not npys:
        raise RuntimeError("No hay archivos de nube de puntos en el snapshot.")
    latest = npys[-1].stem.replace('_points', '')
    print(f"🕐 Procesando snapshot: {latest}")

    img_path = snapshot_dir / f"{latest}_image.png"
    points_path = snapshot_dir / f"{latest}_points.npy"
    tf_path = snapshot_dir / f"{latest}_tf.json"
    caminfo_path = snapshot_dir / "camera_info.json"

    image = np.array(Image.open(img_path).convert("RGB"))
    points = np.load(points_path)
    with open(tf_path) as f: tf = json.load(f)
    with open(caminfo_path) as f: cam = json.load(f)
    K = np.array(cam['K']).reshape(3, 3)
    return latest, image, points, tf, K

def load_model(checkpoint, class_names, device):
    """Load the RF-DETR segmentation model."""
    print("📦 Loading model...")
    model = RFDETRSegPreview(pretrain_weights=checkpoint, class_names=class_names, device=CFG['device'])
    model.optimize_for_inference()
    print("✅ Model loaded.")
    return model

def clean_lidar_cloud(points, nb_neighbors, std_ratio=1.0, z_min=-2.0, z_max=5.0):
    """Clean LiDAR point cloud using height filtering and statistical outlier removal."""
    mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    points = points[mask]
    if len(points) > nb_neighbors:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        _, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        points = np.asarray(pcd.select_by_index(ind).points)
    return points

def analyze_side(veg_points, side_name, proc_dir):
    """Analyze vegetation points on one side of the road."""
    if len(veg_points) == 0:
        print(f"⚠️ No hay puntos de vegetación en el lado {side_name}.")
        return None
    #print(np.min(veg_points[:, 0]), np.min(veg_points[:, 1]), np.min(veg_points[:, 2]))
    x_min = np.min(veg_points[:, 0])
    #if x_min > CFG['threshold_start']:
    if x_min > CFG['max_forward_range']:
        print(f"⚠️ Vegetación del lado {side_name} empieza a {x_min:.2f} m (> {CFG['max_forward_range']} m). Se omite análisis.")
        return None
    
    # define X range
    #x_start, x_end = x_min, x_min + CFG['max_forward_range']
    x_start, x_end = x_min, CFG['max_forward_range']
    print(f"📏 [{side_name}] Analizando rango X: {x_start:.2f} → {x_end:.2f} m")

    # filter points in range
    veg_near = veg_points[(veg_points[:, 0] >= x_start) & (veg_points[:, 0] <= x_end)]
    segments = np.arange(x_start, x_end, CFG['segment_length'])
    heights = []

    for i in range(len(segments) - 1):
        x0, x1 = segments[i], segments[i + 1]
        seg = veg_near[(veg_near[:, 0] >= x0) & (veg_near[:, 0] < x1)]

        if len(seg) > 0:
            z_min, z_max = np.min(seg[:, 2]), np.max(seg[:, 2])
            heights.append([x0, z_max - z_min, z_min, z_max])
        else:
            heights.append([x0, np.nan, np.nan, np.nan])

    heights = np.array(heights, dtype=float)

    df = pd.DataFrame(heights, columns=["x_start", "height", "z_min", "z_max"])
    return df

def run_map_script(snapshot_dir):
    """Execute the map_interactive.py script to visualize results."""
    map_script = Path(__file__).parent / "map_interactive.py"

    if not map_script.exists():
        print("⚠️ No se encontró map_interactive.py, se omite actualización del mapa.")
        return

    try:
        subprocess.run(["python3", str(map_script), str(snapshot_dir)], check=True)
        print("✅ Mapa generado correctamente.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error al ejecutar map_interactive.py: {e}")

# ========= MAIN FUNCTIONS =========
def run_inference(model, image, proc_dir):
    """Run inference on the image and save overlay."""
    detections = model.predict(image, threshold=0.3)
    if detections.mask is None:
        raise RuntimeError("❌ No se han generado máscaras de segmentación.")
    
    H, W, _ = image.shape
    pred_resized = np.zeros((H, W), dtype=np.uint8)
    for cid, mask in zip(detections.class_id, detections.mask):
        pred_resized[mask > 0.5] = cid

    palette = sv.ColorPalette.DEFAULT
    mask_annotator = sv.MaskAnnotator(color=palette)
    overlay = mask_annotator.annotate(scene=image.copy(), detections=detections)
    cv2.imwrite(str(proc_dir / "overlay.png"), overlay)
    return pred_resized, overlay

def project_lidar_to_camera(pred_resized, points, tf, K, proc_dir):
    """Project LiDAR points to camera frame and label them."""
    H, W = pred_resized.shape
    tx, ty, tz = tf['translation'].values()
    qx, qy, qz, qw = tf['rotation'].values()
    Rm = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = [tx, ty, tz]
    # Transform points to camera frame
    points_h = np.hstack((points, np.ones((points.shape[0],1))))
    points_cam = (T @ points_h.T).T[:, :3]
    # Keep only points in front of camera
    mask_front = points_cam[:, 2] > 0
    points_cam = points_cam[mask_front]
    # Project to image plane
    uv = (K @ points_cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    # Discard points outside image
    valid = (uv[:,0] >= 0) & (uv[:,0] < W) & (uv[:,1] >= 0) & (uv[:,1] < H)
    uv = uv[valid].astype(int)
    points_valid = points_cam[valid]
    # Assign labels
    labels = pred_resized[uv[:,1], uv[:,0]]
    T_cam_to_lidar = np.linalg.inv(T)
    # Backproject points of each class
    def backproject(cls_id, name):
        cls_cam = points_valid[labels == cls_id]
        if len(cls_cam) == 0:
            print(f"⚠️ No se encontraron puntos de clase {name}.")
            return np.empty((0,3))
        cls_h = np.hstack((cls_cam, np.ones((cls_cam.shape[0],1))))
        cls_lidar = (T_cam_to_lidar @ cls_h.T).T[:, :3]
        return cls_lidar

    veg_points_orig = backproject(SMALLVEG_CLASS, "smallveg")
    np.save(proc_dir / "smallveg_orig.npy", veg_points_orig)
    veg_points_lidar = clean_lidar_cloud(veg_points_orig, nb_neighbors=CFG["nb_neighbors"], std_ratio=CFG["std_ratio"], z_min=CFG["z_min"], z_max=CFG["z_max"])
    np.save(proc_dir / "smallveg_clean.npy", veg_points_lidar)

    road_points_lidar = backproject(SMOOTH_CLASS, "smooth")
    np.save(proc_dir / "smooth.npy", road_points_lidar)

    return veg_points_lidar, road_points_lidar

def analyze_vegetation(veg_points_lidar, road_points_lidar, proc_dir):
    """Analyze vegetation points and fit road centerline."""
    if len(veg_points_lidar) == 0:
        print("⚠️ No hay puntos de vegetación para analizar.")
        return

    xlsx_path = proc_dir / "veg_heights.xlsx"
    # === CASO OFF-ROAD: no hay puntos de camino ===
    if len(road_points_lidar) < 30:
        print("🌾 Entorno off-road: se analizará la vegetación como un único bloque (sin dividir izquierda/derecha).")
        x_min, x_max = np.min(veg_points_lidar[:, 0]), np.max(veg_points_lidar[:, 0])
        segments = np.arange(x_min, x_max, CFG['segment_length'])
        heights = []

        for i in range(len(segments) - 1):
            x0, x1 = segments[i], segments[i + 1]
            seg = veg_points_lidar[(veg_points_lidar[:, 0] >= x0) & (veg_points_lidar[:, 0] < x1)]
            if len(seg) > 0:
                z_min, z_max = np.min(seg[:, 2]), np.max(seg[:, 2])
                heights.append([x0, z_max - z_min, z_min, z_max])
            else:
                heights.append([x0, np.nan, np.nan, np.nan])

        df_all = pd.DataFrame(heights, columns=["x_start", "height", "z_min", "z_max"])
        df_all.to_excel(xlsx_path, sheet_name="all", index=False)
        print(f"📊 Métricas off-road guardadas en {xlsx_path}")
        return
        
    
    coeffs = np.polyfit(road_points_lidar[:,0], road_points_lidar[:,1], deg=3)
    road_center = np.poly1d(coeffs)
    print("🛣️  Eje del camino ajustado.")
    x_min, x_max = np.min(road_points_lidar[:,0]), np.max(road_points_lidar[:,0])
    
    x_line = np.linspace(x_min, x_max, 200) # 200 points along X
    y_line = road_center(x_line)
    z_line = np.full_like(x_line, np.mean(road_points_lidar[:,2]))

    road_center_points = np.stack([x_line, y_line, z_line], axis=1)
    np.save(proc_dir / "road_center.npy", road_center_points)
    
    xv, yv, zv = veg_points_lidar[:,0], veg_points_lidar[:,1], veg_points_lidar[:,2]
    y_center = road_center(xv)
    side = np.sign(yv - y_center)
    veg_left  = veg_points_lidar[side > 0]
    veg_right = veg_points_lidar[side <= 0]

    np.save(proc_dir / "veg_left.npy", veg_left)
    np.save(proc_dir / "veg_right.npy", veg_right)

    df_left = analyze_side(veg_left, "left", proc_dir)
    df_right = analyze_side(veg_right, "right", proc_dir)

    if df_left is None and df_right is None:
        print("⚠️ No se generó ninguna hoja (left/right), no se guardará el Excel.")
        return

    with pd.ExcelWriter(xlsx_path) as writer:
        if df_left is not None:
            df_left.to_excel(writer, sheet_name="left", index=False)
        if df_right is not None:
            df_right.to_excel(writer, sheet_name="right", index=False)

    print(f"📊 Métricas de vegetación guardadas en {xlsx_path}")

# ========= MAIN PIPELINE =========
def main():
    snapshot_dir = get_latest_snapshot_folder()
    latest, image, points, tf, K = load_snapshot(snapshot_dir)
    proc_dir = create_output_dir(snapshot_dir, latest)

    model = load_model(CHECKPOINT_PATH, CLASS_NAMES, DEVICE)
    pred_resized, overlay = run_inference(model, image, proc_dir)
    veg_points_lidar, road_points_lidar = project_lidar_to_camera(pred_resized, points, tf, K, proc_dir)
    analyze_vegetation(veg_points_lidar, road_points_lidar, proc_dir)

    run_map_script(snapshot_dir)
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n🟥 Ejecución interrumpida manualmente (Ctrl+C). Cerrando de forma segura...")
        torch.cuda.empty_cache()
        os._exit(0)
