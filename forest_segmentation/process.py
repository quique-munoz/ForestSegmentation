#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Procesa TODAS las carpetas de snapshot (de más antigua a más nueva),
fusiona LiDAR + RFDETRSegPreview para obtener puntos 3D 'smallveg' en coords LiDAR,
genera métricas/archivos por snapshot y actualiza el mapa LIMPIANDO JSON previos.
Si un snapshot no tiene camera_info.json, reutiliza la última K conocida.
"""

# ========= IMPORTS =========
import os, json, cv2, torch, numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import pandas as pd
from rfdetr import RFDETRSegPreview
import supervision as sv
from PIL import Image
import warnings
from torch.jit import TracerWarning
import subprocess
import matplotlib
import webbrowser
matplotlib.use("Agg")  # backend no interactivo

# ========= WARNINGS =========
warnings.filterwarnings("ignore", category=TracerWarning)
warnings.filterwarnings("ignore", message=r"torch\.meshgrid:.*indexing")

# ========= CONFIG =========
SNAP_DIR = Path('/home/enrique/ros2_ws/src/forest_segmentation/snapshots')
MAP_DIR = SNAP_DIR / "maps"
MAP_PATH = MAP_DIR / "veg_map.html"
POINTS_JSON = MAP_DIR / "veg_points.json"
CHECKPOINT_PATH = '/home/enrique/ros2_ws/src/forest_segmentation/runs/checkpoint_best_ema.pth'
CLASS_NAMES = ["background", "forest", "obstacle", "rough", "sky", "smallveg", "smooth"]

SMALLVEG_CLASS = 5
SMOOTH_CLASS = 6
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

CFG = {
    "segment_length": 0.1,
    "max_forward_range": 15.0,
    "nb_neighbors": 25,
    "std_ratio": 1.0,
    "z_min": -8.0,
    "z_max": 8.0,
    "device": "cuda"
}

# ========= UTILS =========
def list_snapshot_folders_sorted():
    """Carpetas YYYY-MM-DD_HH-MM-SS (antiguo → nuevo)."""
    if not SNAP_DIR.exists():
        raise RuntimeError(f"❌ No existe SNAP_DIR: {SNAP_DIR}")

    subdirs = [d for d in SNAP_DIR.iterdir() if d.is_dir()]
    valid = []
    for d in subdirs:
        n = d.name
        if (len(n) >= 19 and n[4] == '-' and n[7] == '-' and '_' in n and n[13] == '-'):
            valid.append(d)
    if not valid:
        raise RuntimeError("❌ No se encontraron carpetas válidas YYYY-MM-DD_HH-MM-SS.")
    return sorted(valid, key=lambda x: x.name)

def create_output_dir(snapshot_dir):
    """Crea processed/ en el snapshot."""
    proc_dir = snapshot_dir / "processed"
    proc_dir.mkdir(parents=True, exist_ok=True)
    print(f"📁 Carpeta de salida: {proc_dir}")
    return proc_dir

def load_snapshot(snapshot_dir):
    """
    Devuelve: latest (prefijo str), image (H,W,3 RGB), points (N,3), tf (dict),
    K (3,3) o None si no hay camera_info.json.
    """
    npys = sorted(snapshot_dir.glob('*_points.npy'))
    if not npys:
        raise FileNotFoundError("No hay archivos *_points.npy en el snapshot.")
    latest = npys[-1].stem.replace('_points', '')
    print(f"🕐 Procesando snapshot: {latest}")

    img_path = snapshot_dir / f"{latest}_image.png"
    points_path = snapshot_dir / f"{latest}_points.npy"
    tf_path = snapshot_dir / f"{latest}_tf.json"
    caminfo_path = snapshot_dir / "camera_info.json"

    if not img_path.exists() or not points_path.exists() or not tf_path.exists():
        raise FileNotFoundError(f"Faltan ficheros en {snapshot_dir} (requiere *_image.png, *_points.npy, *_tf.json)")

    image = np.array(Image.open(img_path).convert("RGB"))
    points = np.load(points_path)
    with open(tf_path) as f:
        tf = json.load(f)

    K = None
    if caminfo_path.exists():
        try:
            with open(caminfo_path) as f:
                cam = json.load(f)
            K = np.array(cam['K']).reshape(3, 3)
        except Exception as e:
            print(f"⚠️ camera_info.json inválido en {snapshot_dir.name}: {e}. Se intentará reutilizar K anterior.")
            K = None

    return latest, image, points, tf, K

def load_model(checkpoint, class_names, device):
    """Carga el modelo RF-DETR una sola vez."""
    print("📦 Cargando modelo RF-DETR...")
    model = RFDETRSegPreview(pretrain_weights=checkpoint, class_names=class_names, device=CFG['device'])
    model.optimize_for_inference()
    print("✅ Modelo cargado.")
    return model

def clean_lidar_cloud(points, nb_neighbors, std_ratio=1.0, z_min=-2.0, z_max=5.0):
    """Limpia la nube (filtro por altura + outliers estadísticos)."""
    mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    points = points[mask]
    if len(points) > nb_neighbors:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        _, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        points = np.asarray(pcd.select_by_index(ind).points)
    return points

def analyze_side(veg_points, side_name, proc_dir):
    """Analiza vegetación por lado en segmentos de X."""
    if len(veg_points) == 0:
        print(f"⚠️ No hay puntos de vegetación en el lado {side_name}.")
        return None

    x_min = np.min(veg_points[:, 0])
    if x_min > CFG['max_forward_range']:
        print(f"⚠️ Vegetación {side_name} empieza a {x_min:.2f} m (> {CFG['max_forward_range']} m). Omite análisis.")
        return None

    x_start, x_end = x_min, CFG['max_forward_range']
    print(f"📏 [{side_name}] Rango X: {x_start:.2f} → {x_end:.2f} m")

    veg_near = veg_points[(veg_points[:, 0] >= x_start) & (veg_points[:, 0] <= x_end)]
    segments = np.arange(x_start, x_end, CFG['segment_length'])
    heights = []

    for i in range(len(segments) - 1):
        x0, x1 = segments[i], segments[i + 1]
        seg = veg_near[(veg_near[:, 0] >= x0) & (veg_near[:, 0] < x1)]
        if len(seg) > 0:
            z_min, z_max = np.min(seg[:, 2]), np.max(seg[:, 2])
            y_mean = np.mean(seg[:, 1])
            heights.append([x0, z_max - z_min, z_min, z_max, y_mean])
        else:
            heights.append([x0, np.nan, np.nan, np.nan, np.nan])

    df = pd.DataFrame(heights, columns=["x_start", "height", "z_min", "z_max", "y_mean"])
    return df

def run_map_script(snapshot_dir, reset_done_flag):
    """
    Llama map_interactive.py con el snapshot actual.
    La primera vez elimina JSON previos (reset de mapa).
    """
    map_script = Path(__file__).parent / "map_interactive.py"
    if not map_script.exists():
        print("⚠️ No se encontró map_interactive.py, se omite actualización del mapa.")
        return True  # no bloquea pipeline

    if not reset_done_flag[0]:
        reset_map_jsons()
        reset_done_flag[0] = True

    try:
        subprocess.run(["python3", str(map_script), str(snapshot_dir)], check=True)
        print("✅ Mapa actualizado.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Error al ejecutar map_interactive.py: {e}")
    return True

def reset_map_jsons():
    """Borra JSON previos de mapa para empezar limpio."""
    base_dirs = [Path(__file__).parent, SNAP_DIR]
    patterns = ["map*.json", "tiles*.json", "geo*.json"]
    removed = []
    for d in base_dirs:
        for pat in patterns:
            for f in d.glob(pat):
                try:
                    f.unlink()
                    removed.append(str(f))
                except Exception:
                    pass
    if removed:
        print("🧹 Limpieza de JSON previos del mapa:\n  - " + "\n  - ".join(removed))
    else:
        print("ℹ️ No había JSON previos de mapa que borrar.")

def remove_veg_points(json_path: Path):
    try:
        json_path.unlink(missing_ok=True)  
        print(f"🗑️  Eliminado: {json_path}")
    except Exception as e:
        print(f"⚠️ No se pudo eliminar {json_path}: {e}")

# ========= INFERENCE / PROJECTION =========
def run_inference(model, image, proc_dir):
    """Inferencia + overlay + VARI (solo smallveg)."""
    detections = model.predict(image, threshold=0.3)
    if detections.mask is None:
        raise RuntimeError("❌ No se han generado máscaras de segmentación.")

    H, W, _ = image.shape
    pred_resized = np.zeros((H, W), dtype=np.uint8)

    scores = np.array(detections.confidence)
    order = np.argsort(scores)  # pintamos más confiables al final
    for i in order:
        cid = detections.class_id[i]
        mask = detections.mask[i]
        pred_resized[mask > 0.5] = cid

    palette = sv.ColorPalette.DEFAULT
    color_mask = np.zeros_like(image, dtype=np.uint8)
    for class_id in np.unique(pred_resized):
        color = palette.by_idx(int(class_id)).as_bgr()
        color_mask[pred_resized == class_id] = color

    overlay = cv2.addWeighted(image, 0.5, color_mask, 0.5, 0)
    cv2.imwrite(str(proc_dir / "overlay.png"), overlay)

    # ===== VARI smallveg =====
    print("🌿 Calculando índice VARI (solo smallveg)...")
    mask_smallveg = (pred_resized == SMALLVEG_CLASS)
    arr = image.astype(float)  # RGB
    Rc, Gc, Bc = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]

    vari = np.full_like(Gc, np.nan, dtype=float)
    m = mask_smallveg
    denom = (Gc + Rc - Bc + 1e-6)
    vari[m] = (Gc[m] - Rc[m]) / denom[m]
    vari = np.clip(vari, -1, 1)

    import matplotlib.pyplot as plt
    vmin, vmax = np.nanpercentile(vari, 10), np.nanpercentile(vari, 90)
    fig, ax = plt.subplots(figsize=(8, 4), dpi=150)
    cim = ax.imshow(vari, vmin=vmin, vmax=vmax, cmap="RdYlGn")
    fig.colorbar(cim, ax=ax, label="VARI (smallveg)")
    ax.set_xticks([]); ax.set_yticks([])
    fig.savefig(proc_dir / "vari_smallveg.png", bbox_inches="tight")
    plt.close(fig)

    print("✅ VARI guardado.")
    return pred_resized, overlay

def project_lidar_to_camera(pred_resized, points, tf, K, proc_dir):
    """Proyecta LiDAR→cámara, etiqueta y vuelve a LiDAR por clases."""
    H, W = pred_resized.shape

    # TF (lidar->cam)
    tx, ty, tz = tf['translation'].values()
    qx, qy, qz, qw = tf['rotation'].values()
    Rm = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = [tx, ty, tz]

    # Transformar puntos a cámara
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))
    points_cam = (T @ points_h.T).T[:, :3]

    # Solo delante de la cámara
    mask_front = points_cam[:, 2] > 0
    points_cam = points_cam[mask_front]

    # Proyectar
    uv = (K @ points_cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]

    # Dentro de imagen
    valid = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    uv = uv[valid].astype(int)
    points_valid = points_cam[valid]

    labels = pred_resized[uv[:, 1], uv[:, 0]]

    T_cam_to_lidar = np.linalg.inv(T)

    def backproject(cls_id, name):
        cls_cam = points_valid[labels == cls_id]
        if len(cls_cam) == 0:
            print(f"⚠️ No se encontraron puntos de clase {name}.")
            return np.empty((0, 3))
        cls_h = np.hstack((cls_cam, np.ones((cls_cam.shape[0], 1))))
        cls_lidar = (T_cam_to_lidar @ cls_h.T).T[:, :3]
        return cls_lidar

    veg_points_orig = backproject(SMALLVEG_CLASS, "smallveg")
    np.save(proc_dir / "smallveg_orig.npy", veg_points_orig)
    veg_points_lidar = clean_lidar_cloud(
        veg_points_orig,
        nb_neighbors=CFG["nb_neighbors"],
        std_ratio=CFG["std_ratio"],
        z_min=CFG["z_min"], z_max=CFG["z_max"]
    )
    np.save(proc_dir / "smallveg_clean.npy", veg_points_lidar)

    road_points_lidar = backproject(SMOOTH_CLASS, "smooth")
    np.save(proc_dir / "smooth.npy", road_points_lidar)

    return veg_points_lidar, road_points_lidar

def analyze_vegetation(veg_points_lidar, road_points_lidar, proc_dir):
    """Analiza vegetación y eje de camino si existe."""
    if len(veg_points_lidar) == 0:
        print("⚠️ No hay puntos de vegetación para analizar.")
        return

    xlsx_path = proc_dir / "veg_heights.xlsx"

    # OFF-ROAD (sin puntos de camino)
    if len(road_points_lidar) < 30:
        print("🌾 Entorno off-road: vegetación como único bloque.")
        x_min, x_max = np.min(veg_points_lidar[:, 0]), np.max(veg_points_lidar[:, 0])
        segments = np.arange(x_min, x_max, CFG['segment_length'])
        heights = []
        for i in range(len(segments) - 1):
            x0, x1 = segments[i], segments[i + 1]
            seg = veg_points_lidar[(veg_points_lidar[:, 0] >= x0) & (veg_points_lidar[:, 0] < x1)]
            if len(seg) > 0:
                z_min, z_max = np.min(seg[:, 2]), np.max(seg[:, 2])
                y_mean = np.mean(seg[:, 1])
                heights.append([x0, z_max - z_min, z_min, z_max, y_mean])
            else:
                heights.append([x0, np.nan, np.nan, np.nan, np.nan])

        df_all = pd.DataFrame(heights, columns=["x_start", "height", "z_min", "z_max", "y_mean"])
        df_all.to_excel(xlsx_path, sheet_name="all", index=False)
        print(f"📊 Métricas off-road guardadas en {xlsx_path}")
        return

    # Con camino: ajusta eje
    coeffs = np.polyfit(road_points_lidar[:, 0], road_points_lidar[:, 1], deg=3)
    road_center = np.poly1d(coeffs)
    print("🛣️  Eje del camino ajustado.")
    x_min, x_max = np.min(road_points_lidar[:, 0]), np.max(road_points_lidar[:, 0])

    x_line = np.linspace(x_min, x_max, 200)
    y_line = road_center(x_line)
    z_line = np.full_like(x_line, np.mean(road_points_lidar[:, 2]))
    road_center_points = np.stack([x_line, y_line, z_line], axis=1)
    np.save(proc_dir / "road_center.npy", road_center_points)

    xv, yv, zv = veg_points_lidar[:, 0], veg_points_lidar[:, 1], veg_points_lidar[:, 2]
    y_center = road_center(xv)
    side = np.sign(yv - y_center)
    veg_left  = veg_points_lidar[side > 0]
    veg_right = veg_points_lidar[side <= 0]

    np.save(proc_dir / "veg_left.npy", veg_left)
    np.save(proc_dir / "veg_right.npy", veg_right)

    df_left = analyze_side(veg_left, "left", proc_dir)
    df_right = analyze_side(veg_right, "right", proc_dir)

    if df_left is None and df_right is None:
        print("⚠️ No se generó ninguna hoja (left/right); no se guarda Excel.")
        return

    with pd.ExcelWriter(xlsx_path) as writer:
        if df_left is not None:
            df_left.to_excel(writer, sheet_name="left", index=False)
        if df_right is not None:
            df_right.to_excel(writer, sheet_name="right", index=False)

    print(f"📊 Métricas de vegetación guardadas en {xlsx_path}")

# ========= MAIN PIPELINE =========
def process_snapshot_folder(model, snapshot_dir, reset_done_flag, state):
    """
    state: dict mutable con 'last_cam_K' para reutilizar intrínsecos.
    """
    try:
        latest, image, points, tf, K = load_snapshot(snapshot_dir)
    except Exception as e:
        print(f"⚠️ Omitiendo {snapshot_dir.name}: {e}")
        return False

    # Resolver K (usar la última conocida si falta en este snapshot)
    if K is None:
        if state['last_cam_K'] is None:
            print(f"❌ {snapshot_dir.name}: no hay camera_info.json y no existe K previa. Se omite.")
            return False
        K = state['last_cam_K'].copy()
        print(f"ℹ️ {snapshot_dir.name}: reutilizando intrínsecos de cámara del snapshot anterior.")
    else:
        # Actualizar última K conocida
        state['last_cam_K'] = K.copy()

    proc_dir = create_output_dir(snapshot_dir)

    # Guardar trazabilidad de la K usada en este snapshot
    caminfo_used = {'K': K.reshape(-1).tolist()}
    with open(proc_dir / "camera_info_used.json", 'w') as f:
        json.dump(caminfo_used, f, indent=2)

    try:
        pred_resized, overlay = run_inference(model, image, proc_dir)
        veg_points_lidar, road_points_lidar = project_lidar_to_camera(pred_resized, points, tf, K, proc_dir)
        analyze_vegetation(veg_points_lidar, road_points_lidar, proc_dir)
        run_map_script(snapshot_dir, reset_done_flag)
    except Exception as e:
        print(f"❌ Error procesando {snapshot_dir.name}: {e}")
        return False
    return True

def main():
    remove_veg_points(POINTS_JSON)
    MAP_PATH
    # 1) listar carpetas (antiguas → nuevas)
    folders = list_snapshot_folders_sorted()
    print("🗂️ Snapshots a procesar (antiguo → nuevo):")
    for d in folders:
        print("  -", d.name)

    # 2) cargar modelo una sola vez
    model = load_model(CHECKPOINT_PATH, CLASS_NAMES, DEVICE)

    # 3) estado para reutilizar intrínsecos de cámara
    state = {'last_cam_K': None}

    # 4) reset de mapa sólo una vez (antes del primer snapshot procesado)
    reset_done_flag = [False]  # mutable

    # 5) procesar en orden
    ok_count = 0
    for snap in folders:
        if process_snapshot_folder(model, snap, reset_done_flag, state):
            ok_count += 1

    print(f"\n✅ Finalizado. Snapshots procesados correctamente: {ok_count}/{len(folders)}")
    webbrowser.open(f"file://{MAP_PATH.resolve()}")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n🟥 Interrumpido (Ctrl+C). Vaciando CUDA y saliendo...")
        torch.cuda.empty_cache()
        os._exit(0)
