#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mapa con un único marcador "combinado" por snapshot:
- Si existe hoja 'all' -> usar ese valor.
- Si no, combinar left/right (por defecto, estrategia 'max').
El popup muestra los valores individuales y, si hay L/R, la media ponderada.
"""

import sys
import json
import webbrowser
from pathlib import Path

import folium
import numpy as np
import pandas as pd
from folium import plugins

# ================== Args & rutas ==================
if len(sys.argv) < 2:
    raise RuntimeError("Debes pasar la carpeta como argumento.\nEjemplo: python3 map_interactive.py /ruta/al/snapshot")

snapshot_dir = Path(sys.argv[1]).resolve()
if not snapshot_dir.exists() or not snapshot_dir.is_dir():
    raise RuntimeError(f"La carpeta proporcionada no existe o no es un directorio: {snapshot_dir}")

SNAP_DIR = snapshot_dir.parent
MAP_DIR = SNAP_DIR / "maps"
MAP_DIR.mkdir(parents=True, exist_ok=True)
MAP_PATH = MAP_DIR / "veg_map.html"
POINTS_JSON = MAP_DIR / "veg_points.json"

# ================== Config ==================
"""Strategy to combine left/right heights when 'all' is not available."""
# max: take the maximum of both sides
# weighted_mean: weighted mean based on number of segments
# mean: simple mean of both sides
COMBINE_STRATEGY = "max"  # "max" | "weighted_mean" | "mean"

# ================== Utilidades ==================
def robust_mean_and_count(df, side_name="unknown", iqr_factor=0.6, mad_threshold=2.5, min_height=0.05):
    """
    Calcula la media robusta y el número de segmentos válidos de altura de vegetación.
    - Filtra outliers mediante IQR y MAD.
    - Excluye segmentos sin vegetación (altura <= min_height).
    """
    if df is None or "height" not in df.columns:
        return np.nan, 0

    # Filtrar alturas no nulas o negativas
    h = df["height"].dropna().values
    h = h[h > min_height]  # <-- ignorar hierba "inexistente"
    if h.size == 0:
        print(f"⚠️ [{side_name}] Sin alturas válidas (todas <= {min_height} m o NaN).")
        return np.nan, 0

    # --- Filtrado IQR ---
    q1, q3 = np.percentile(h, [25, 75])
    iqr = q3 - q1
    lower, upper = q1 - iqr_factor * iqr, q3 + iqr_factor * iqr
    mask_iqr = (h >= lower) & (h <= upper)
    h_iqr = h[mask_iqr]

    if h_iqr.size == 0:
        print(f"⚠️ [{side_name}] Todos los valores eliminados por IQR.")
        return np.nan, 0

    # --- Filtrado MAD ---
    median = np.median(h_iqr)
    mad = np.median(np.abs(h_iqr - median))
    if mad < 1e-6:  # prácticamente sin dispersión
        return float(np.mean(h_iqr)), int(h_iqr.size)

    z_mad = 0.6745 * (h_iqr - median) / mad
    mask_mad = np.abs(z_mad) < mad_threshold
    h_final = h_iqr[mask_mad]

    if h_final.size == 0:
        print(f"⚠️ [{side_name}] Todos los valores eliminados por MAD, usando IQR.")
        return float(np.mean(h_iqr)), int(h_iqr.size)

    # --- Resultado final ---
    mean_h = float(np.mean(h_final))
    count_h = int(h_final.size)
    print(f"📊 [{side_name}] {count_h} segmentos válidos → media {mean_h:.2f} m")
    return mean_h, count_h


def height_to_color_icon(h):
    """Color/Icon severity based on height."""
    if not np.isfinite(h):
        return "gray", "question"
    if h < 1.0:
        return "green", "check"
    elif h < 2.0:
        return "orange", "triangle-exclamation"
    else:
        return "red", "fire"

def combine_heights(h_left, n_left, h_right, n_right, h_all, strategy="max"):
    """
    If available, use the 'all' height.
    If not, combine L/R according to strategy.
    """
    if np.isfinite(h_all):
        return h_all

    vals = [v for v in [h_left, h_right] if np.isfinite(v)]
    if len(vals) == 0:
        return np.nan

    if strategy == "max":
        return max(vals)
    elif strategy == "mean":
        return float(np.mean(vals))
    elif strategy == "weighted_mean":
        total_n = (n_left if np.isfinite(h_left) else 0) + (n_right if np.isfinite(h_right) else 0)
        if total_n > 0:
            num = (h_left * n_left if np.isfinite(h_left) else 0.0) + (h_right * n_right if np.isfinite(h_right) else 0.0)
            return float(num / total_n)
        # si no hay n, caer a media simple
        return float(np.mean(vals))
    else:
        # fallback seguro
        return max(vals)

def fmt(v):
    return f"{v:.2f} m" if (v is not None and np.isfinite(v)) else "—"

# ================== GPS ==================
gps_files = sorted(snapshot_dir.glob("*_gps.json"))
if not gps_files:
    raise RuntimeError(f"No se encontró ningún archivo *_gps.json en {snapshot_dir}")
latest_gps = gps_files[-1]
with open(latest_gps, "r") as f:
    gps = json.load(f)
lat, lon = float(gps["latitude"]), float(gps["longitude"])
print(f"🛰️ GPS: {latest_gps.name} | 🌍 lat={lat:.6f}, lon={lon:.6f}")

# ================== Excel ==================
proc_dir = snapshot_dir / "processed"
xlsx_path = proc_dir / "veg_heights.xlsx"
if not xlsx_path.exists():
    print("⚠️ No hay archivo veg_heights.xlsx → No se detectó vegetación, se omite este snapshot.")
    # Mostrar mapa anterior si existe; si no, crear vacío
    if MAP_PATH.exists():
        webbrowser.open(f"file://{MAP_PATH.resolve()}")
    else:
        m = folium.Map(location=[lat, lon], zoom_start=18, max_zoom=22, tiles="OpenStreetMap")
        m.save(MAP_PATH)
        webbrowser.open(f"file://{MAP_PATH.resolve()}")
    sys.exit(0)

print(f"📗 Usando métricas de vegetación: {xlsx_path}")

xls = pd.ExcelFile(xlsx_path)
sheets = xls.sheet_names
has_all = "all" in sheets
has_left = "left" in sheets
has_right = "right" in sheets

df_all   = pd.read_excel(xlsx_path, sheet_name="all")   if has_all   else None
df_left  = pd.read_excel(xlsx_path, sheet_name="left")  if (not has_all and has_left)  else None
df_right = pd.read_excel(xlsx_path, sheet_name="right") if (not has_all and has_right) else None

# ================== Cálculo de alturas ==================
h_all,   n_all   = robust_mean_and_count(df_all,   "all")   if has_all   else (np.nan, 0)
h_left,  n_left  = robust_mean_and_count(df_left,  "left")  if df_left  is not None else (np.nan, 0)
h_right, n_right = robust_mean_and_count(df_right, "right") if df_right is not None else (np.nan, 0)
# Combinado
h_combined = combine_heights(h_left, n_left, h_right, n_right, h_all, COMBINE_STRATEGY)
color_comb, icon_comb = height_to_color_icon(h_combined)

# ================== Histórico ==================
if POINTS_JSON.exists():
    with open(POINTS_JSON, "r") as f:
        try:
            points_data = json.load(f)
        except Exception:
            points_data = {}
else:
    points_data = {}

points_data.setdefault("combined", [])
points_data.setdefault("vehicle", [])

timestamp = snapshot_dir.name

# Añadir combinado (si hay dato)
if np.isfinite(h_combined):
    entry = {
        "lat": lat, "lon": lon,
        "height": float(h_combined),
        "color": color_comb, "icon": icon_comb,
        "timestamp": timestamp,
        "strategy": "all" if np.isfinite(h_all) else COMBINE_STRATEGY
    }
    # Detalle para el popup
    if np.isfinite(h_all):
        entry.update({"all": float(h_all), "n_all": int(n_all)})
    else:
        if np.isfinite(h_left):
            entry.update({"left": float(h_left), "n_left": int(n_left)})
        if np.isfinite(h_right):
            entry.update({"right": float(h_right), "n_right": int(n_right)})
        # weighted mean informativa si hay ambos lados con cobertura
        if np.isfinite(h_left) and np.isfinite(h_right) and (n_left + n_right) > 0:
            entry["weighted_mean"] = float((h_left*n_left + h_right*n_right) / (n_left + n_right))

    points_data["combined"].append(entry)
else:
    print("ℹ️ Sin valores de vegetación válidos → no se añade marcador combinado.")

# Trayectoria del vehículo
points_data["vehicle"].append({"lat": lat, "lon": lon, "timestamp": timestamp})

with open(POINTS_JSON, "w") as f:
    json.dump(points_data, f, indent=2)
print(f"📁 Histórico actualizado en: {POINTS_JSON}")

# ================== Render del mapa ==================
m = folium.Map(location=[lat, lon], zoom_start=18, max_zoom=22, tiles="OpenStreetMap")
plugins.MeasureControl(primary_length_unit='meters').add_to(m)

fg_veg = folium.FeatureGroup(name="Vegetación detectada")
m.add_child(fg_veg)
fg_path = folium.FeatureGroup(name="Ruta del vehículo").add_to(m)
g_red = plugins.FeatureGroupSubGroup(fg_veg, "Vegetación alta")
m.add_child(g_red)
g_orange = plugins.FeatureGroupSubGroup(fg_veg, "Vegetación media")
m.add_child(g_orange)
g_green = plugins.FeatureGroupSubGroup(fg_veg, "Vegetación baja")
m.add_child(g_green)

# Un único marcador por snapshot (historico "combined")
for p in points_data["combined"]:
    # Popup con detalle
    lines = [
        f"<b>GPS:</b> {p.get('lat')}, {p.get('lon')}",
        f"<b>Altura vegetación ({p.get('strategy','')})</b>: {fmt(p.get('height'))}",
        "<hr style='margin:4px 0'/>"
    ]
    if p.get("strategy") == "all":
        lines.append(f"<b>All</b>: {fmt(p.get('all'))} (n={p.get('n_all',0)})")
    else:
        lines.append(f"<b>Left</b>: {fmt(p.get('left'))} (n={p.get('n_left',0)})")
        lines.append(f"<b>Right</b>: {fmt(p.get('right'))} (n={p.get('n_right',0)})")
        if "weighted_mean" in p:
            lines.append(f"<b>Weighted mean (L/R)</b>: {fmt(p.get('weighted_mean'))}")

    popup_html = "<br/>".join(lines)
    if p["color"] == "red":
        folium.Marker(
            [p["lat"], p["lon"]],
            icon=folium.Icon(color=p["color"], icon=p["icon"], prefix="fa"),
            popup=folium.Popup(popup_html, max_width=340)
        ).add_to(g_red)
    elif p["color"] == "orange":
        folium.Marker(
            [p["lat"], p["lon"]],
            icon=folium.Icon(color=p["color"], icon=p["icon"], prefix="fa"),
            popup=folium.Popup(popup_html, max_width=340)
        ).add_to(g_orange)
    elif p["color"] == "green":
        folium.Marker(
            [p["lat"], p["lon"]],
            icon=folium.Icon(color=p["color"], icon=p["icon"], prefix="fa"),
            popup=folium.Popup(popup_html, max_width=340)
        ).add_to(g_green)

# Polyline de la trayectoria (si hay >1 puntos)
veh = points_data.get("vehicle", [])
if len(veh) > 1:
    trail = [(v["lat"], v["lon"]) for v in veh]
    #folium.PolyLine(trail, color="blue", weight=2.5, opacity=0.8, tooltip="Trayectoria del vehículo").add_to(m)
    plugins.AntPath(trail, color="blue", delay=10000, dash_array=[20,30], pulse_color="lightblue", tooltip="Trayectoria del vehículo").add_to(fg_path)


# ================== Capa de calor basada en los puntos de vegetación ==================
MAX_HEIGHT = 2.0

heat_data = []
for p in points_data["combined"]:
    if np.isfinite(p.get("height", np.nan)):
        h_val = np.clip(p["height"] / MAX_HEIGHT, 0, 1)  # Normalizado 0–1
        heat_data.append([p["lat"], p["lon"], h_val])

if heat_data:
    print(f"🔥 Añadiendo mapa de calor con {len(heat_data)} puntos (basado en alturas).")
    plugins.HeatMap(
        heat_data,
        name="Mapa de calor de vegetación",
        control_scale=True,
        overlay=True,
        min_opacity=0.4,
        max_opacity=0.9,
        radius=50,          # tamaño del área de influencia (ajusta según densidad GPS)
        blur=25,
        gradient={
            0.0: 'green',    # vegetación baja
            0.5: 'yellow',   # media
            1.0: 'red'       # alta
        },

    ).add_to(m)
    
else:
    print("⚠️ No hay datos válidos para generar la capa de calor.")

folium.LayerControl(collapsed=False).add_to(m)
plugins.GroupedLayerControl(
    groups={'Altura vegetación': [g_red, g_orange, g_green]},
    exclusive_groups=False,
    collapsed=False,
).add_to(m)

# Guardar y abrir
m.save(MAP_PATH)
print(f"💾 Mapa actualizado en: {MAP_PATH}")
# webbrowser.open(f"file://{MAP_PATH.resolve()}")