#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SnapshotSaverAction (flexible, por modalidades)
------------------------------------------------
Permite que el *cliente* elija por goal qué modalidades guardar:
- Imagen (sensor_msgs/Image)
- GPS (sensor_msgs/NavSatFix)
- LiDAR (sensor_msgs/PointCloud2) + TF opcional camera<-lidar
- CameraInfo (sensor_msgs/CameraInfo)

Uso con Action "TakeSnapshot":
Request:
  bool require_image
  bool require_gps
  bool require_lidar
  bool require_caminfo
  float32 sync_slop_sec   # ventana temporal ±slop para casar mensajes
  float32 timeout_sec     # máximo de espera del goal

Result:
  bool success
  string error
  string output_dir
  string basename
  string image_path
  string points_path
  string gps_path
  string tf_path

Notas clave de esta versión:
- Solo guarda **mensajes posteriores** al inicio del goal (corte de tiempo).
- Limpia **cachés** al iniciar el goal para evitar usar mensajes previos.
- Garantiza **un único guardado** por goal aunque entren varias callbacks.
- Si se solicita CameraInfo y no hay aún, no bloquea (guarda el resto igual).
"""

import os
import json
import threading
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, NavSatFix
import cv2
import numpy as np
import tf2_ros

from forest_segmentation_interfaces.action import TakeSnapshot


class SnapshotSaverAction(Node):
    def __init__(self):
        super().__init__('snapshot_saver_action')
        self.bridge = CvBridge()

        # ===================== Parámetros =====================
        self.declare_parameter('image_topic',   '/gmsl_camera/port_0/cam_0/image_raw')
        self.declare_parameter('caminfo_topic', '/gmsl_camera/port_0/cam_0/camera_info')
        self.declare_parameter('lidar_topic',   '/LiDAR_1/points_raw')
        self.declare_parameter('gps_topic',     '/fix')
        self.declare_parameter('camera_frame',  'camera_left')
        self.declare_parameter('lidar_frame',   'lidar')
        self.declare_parameter('base_snap_dir', '')  # si vacío, se resuelve estilo proyecto

        image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        caminfo_topic = self.get_parameter('caminfo_topic').get_parameter_value().string_value
        lidar_topic   = self.get_parameter('lidar_topic').get_parameter_value().string_value
        gps_topic     = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame  = self.get_parameter('lidar_frame').get_parameter_value().string_value

        base_snap_dir = self.get_parameter('base_snap_dir').get_parameter_value().string_value
        if not base_snap_dir:
            # Estilo forest_segmentation/src/forest_segmentation/snapshots
            try:
                target_pkg = 'forest_segmentation'
                pkg_share_path = get_package_share_directory(target_pkg)
                src_pkg_path = os.path.abspath(os.path.join(pkg_share_path, '../../../../src', target_pkg))
                base_snap_dir = os.path.join(src_pkg_path, 'snapshots')
            except (PackageNotFoundError, Exception):
                base_snap_dir = os.path.expanduser('~/snapshots')
        self.base_snap_dir = base_snap_dir
        os.makedirs(self.base_snap_dir, exist_ok=True)
        self.get_logger().info(f'Raíz de snapshots: {self.base_snap_dir}')

        # ===================== Estado por goal =====================
        self.cb_group = ReentrantCallbackGroup()
        self._goal_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._active_goal = None
        self._goal_event = threading.Event()
        self._goal_in_progress = False

        # Flags por goal (defaults)
        self._require_image = True
        self._require_gps = True
        self._require_lidar = True
        self._require_caminfo = False
        self._slop_sec = 0.05
        self._timeout_sec = 3.0

        # Control temporal y “save-once”
        self._goal_start_sec = None
        self._save_triggered = False

        # ===================== Caches por tópico =====================
        self.cache_img = deque(maxlen=60)    # (t, Image)
        self.cache_gps = deque(maxlen=300)   # (t, NavSatFix)
        self.cache_lidar = deque(maxlen=60)  # (t, PointCloud2)

        # ===================== Suscripciones =====================
        self.create_subscription(Image,      image_topic,   self._image_cb,   10, callback_group=self.cb_group)
        self.create_subscription(CameraInfo, caminfo_topic, self._caminfo_cb, 10, callback_group=self.cb_group)
        self.create_subscription(PointCloud2, lidar_topic,  self._lidar_cb,   10, callback_group=self.cb_group)
        self.create_subscription(NavSatFix,  gps_topic,     self._gps_cb,     10, callback_group=self.cb_group)

        self.caminfo = None

        # ===================== TF =====================
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===================== Action server =====================
        self._server = ActionServer(
            self,
            TakeSnapshot,
            'take_snapshot',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group
        )

        self.get_logger().info('SnapshotSaverAction listo (action: /take_snapshot)')

    # ===================== Utilidades =====================
    @staticmethod
    def _file_prefix_from_stamp(stamp_sec: float) -> str:
        return datetime.fromtimestamp(stamp_sec).strftime('%Y%m%d_%H%M%S_%f')

    @staticmethod
    def _folder_name_now() -> str:
        return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    @staticmethod
    def _to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _closest_in_slop(self, cache: deque, t_anchor: float, slop: float):
        """
        Devuelve el msg más cercano a t_anchor si |Δt|<=slop, usando SOLO mensajes
        con t >= _goal_start_sec para evitar arrastrar datos previos al goal.
        """
        best = None
        best_dt = None
        min_t = self._goal_start_sec
        with self._cache_lock:
            for t, msg in reversed(cache):  # de más nuevo a más viejo
                # ignora todo lo anterior al goal
                if min_t is not None and t < min_t:
                    continue
                dt = abs(t - t_anchor)
                if dt <= slop and (best_dt is None or dt < best_dt):
                    best, best_dt = (t, msg), dt
                elif t < t_anchor - slop:
                    break
        return best[1] if best else None

    # ===================== Callbacks de topics =====================
    def _image_cb(self, msg: Image):
        t = self._to_sec(msg.header.stamp)
        with self._cache_lock:
            self.cache_img.append((t, msg))
        # Si hay goal en curso y se requiere imagen, intentamos snapshot anclando a imagen
        if self._goal_in_progress and self._require_image:
            self._try_snapshot(anchor='image', t_anchor=t)

    def _caminfo_cb(self, msg: CameraInfo):
        self.caminfo = msg

    def _lidar_cb(self, msg: PointCloud2):
        t = self._to_sec(msg.header.stamp)
        with self._cache_lock:
            self.cache_lidar.append((t, msg))
        # Si no se requiere imagen pero sí lidar, podemos anclar a lidar
        if self._goal_in_progress and (not self._require_image) and self._require_lidar:
            self._try_snapshot(anchor='lidar', t_anchor=t)

    def _gps_cb(self, msg: NavSatFix):
        t = self._to_sec(msg.header.stamp)
        with self._cache_lock:
            self.cache_gps.append((t, msg))
        # Si solo se requiere GPS, anclamos a GPS
        if self._goal_in_progress and (not self._require_image) and (not self._require_lidar) and self._require_gps:
            self._try_snapshot(anchor='gps', t_anchor=t)

    # ===================== Lógica de snapshot =====================
    def _try_snapshot(self, anchor: str, t_anchor: float):
        # Reúne las modalidades solicitadas dentro de ±slop y guarda
        img_msg = None
        gps_msg = None
        lidar_msg = None

        # Imagen (si se quiere, re-ancora el tiempo a la imagen real)
        if self._require_image:
            img_msg = self._closest_in_slop(self.cache_img, t_anchor, self._slop_sec)
            if img_msg is None:
                return
            t_anchor = self._to_sec(img_msg.header.stamp)

        # GPS
        if self._require_gps:
            gps_msg = self._closest_in_slop(self.cache_gps, t_anchor, self._slop_sec)
            if gps_msg is None:
                return

        # LiDAR
        if self._require_lidar:
            lidar_msg = self._closest_in_slop(self.cache_lidar, t_anchor, self._slop_sec)
            if lidar_msg is None:
                return

        # Evita condiciones de carrera: solo el primer hilo/proceso de callback guardará
        with self._goal_lock:
            if not self._goal_in_progress or self._save_triggered:
                return
            self._save_triggered = True

        # Si llegamos aquí, hay material suficiente para guardar
        self._do_save(img_msg, gps_msg, lidar_msg, t_anchor)

    def _do_save(self, img_msg, gps_msg, lidar_msg, t_anchor: float):
        # Carpeta del snapshot
        run_stamp = self._folder_name_now()
        snapshot_dir = os.path.join(self.base_snap_dir, run_stamp)
        os.makedirs(snapshot_dir, exist_ok=True)

        # Prefijo temporal coherente (ancla)
        time_str = self._file_prefix_from_stamp(t_anchor)

        # Rutas (vacías si no se pidió)
        img_path = os.path.join(snapshot_dir, f'{time_str}_image.png') if self._require_image else ''
        npy_path = os.path.join(snapshot_dir, f'{time_str}_points.npy') if self._require_lidar else ''
        gps_path = os.path.join(snapshot_dir, f'{time_str}_gps.json') if self._require_gps else ''
        tf_path  = os.path.join(snapshot_dir, f'{time_str}_tf.json') if self._require_lidar else ''

        # Imagen
        if self._require_image and img_msg is not None:
            try:
                img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
                cv2.imwrite(img_path, img)
            except Exception as e:
                self.get_logger().warn(f'No se pudo convertir/guardar la imagen: {e}')
                img_path = ''

        # GPS
        if self._require_gps and gps_msg is not None:
            try:
                with open(gps_path, 'w') as f:
                    json.dump({
                        'latitude': gps_msg.latitude,
                        'longitude': gps_msg.longitude,
                        'altitude': gps_msg.altitude
                    }, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f'No se pudo guardar GPS: {e}')
                gps_path = ''

        # LiDAR + TF
        if self._require_lidar and lidar_msg is not None:
            # PointCloud2 -> npy (asumiendo campos XYZ float32 contiguos)
            try:
                if len(lidar_msg.data) > 0 and lidar_msg.point_step > 0:
                    arr = np.frombuffer(lidar_msg.data, dtype=np.uint8)
                    # Interpretación simple: agrupar por point_step y tomar 3 floats iniciales
                    stride_f32 = int(lidar_msg.point_step // 4)
                    pts = arr.view(np.float32).reshape(-1, stride_f32)[:, :3]
                    pts = pts[np.isfinite(pts).all(axis=1)]
                    if pts.shape[0] >= 1:
                        np.save(npy_path, pts)
                    else:
                        self.get_logger().warn('Nube con 0 puntos válidos; no se guarda .npy')
                        npy_path = ''
                else:
                    self.get_logger().warn('PointCloud2 vacío; no se guarda .npy')
                    npy_path = ''
            except Exception as e:
                self.get_logger().error(f'Error procesando nube: {e}')
                npy_path = ''

            # TF camera<-lidar (opcional), al tiempo del snapshot (coherente)
            try:
                tf_time = Time(nanoseconds=int(t_anchor * 1e9))
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, self.lidar_frame, tf_time, timeout=Duration(seconds=0.5)
                )
                with open(tf_path, 'w') as f:
                    json.dump({
                        'translation': {
                            'x': tf.transform.translation.x,
                            'y': tf.transform.translation.y,
                            'z': tf.transform.translation.z
                        },
                        'rotation': {
                            'x': tf.transform.rotation.x,
                            'y': tf.transform.rotation.y,
                            'z': tf.transform.rotation.z,
                            'w': tf.transform.rotation.w
                        },
                        'target_frame': self.camera_frame,
                        'source_frame': self.lidar_frame,
                        'stamp_sec': t_anchor
                    }, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f'No se pudo obtener TF {self.camera_frame}<-{self.lidar_frame}: {e}')
                tf_path = ''

        # CameraInfo (no bloqueante)
        if self._require_caminfo and self.caminfo is not None:
            try:
                caminfo_path = os.path.join(snapshot_dir, 'camera_info.json')
                with open(caminfo_path, 'w') as f:
                    json.dump({
                        'K': list(self.caminfo.k),
                        'D': list(self.caminfo.d),
                        'width': int(self.caminfo.width),
                        'height': int(self.caminfo.height),
                        'frame_id': self.caminfo.header.frame_id
                    }, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f'No se pudo guardar CameraInfo: {e}')

        # Guardar resultado y completar goal
        self._last_result = (time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path)
        with self._goal_lock:
            self._goal_in_progress = False
        self._goal_event.set()

    # ===================== Action plumbing =====================
    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        with self._goal_lock:
            if self._active_goal is not None and goal_handle == self._active_goal:
                self._goal_event.set()
                self._goal_in_progress = False
                self._save_triggered = False
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        # Preparar estado del goal
        with self._goal_lock:
            # Si hay un goal activo, lo abortamos para priorizar el nuevo
            if self._active_goal is not None and self._active_goal.is_active:
                self._active_goal.abort()
            self._active_goal = goal_handle
            self._goal_event.clear()
            self._last_result = None

            # Flags desde el request
            self._require_image   = bool(getattr(goal_handle.request, 'require_image', True))
            self._require_gps     = bool(getattr(goal_handle.request, 'require_gps', True))
            self._require_lidar   = bool(getattr(goal_handle.request, 'require_lidar', True))
            self._require_caminfo = bool(getattr(goal_handle.request, 'require_caminfo', False))

            self._slop_sec    = float(getattr(goal_handle.request, 'sync_slop_sec', 0.05) or 0.05)
            self._timeout_sec = float(getattr(goal_handle.request, 'timeout_sec', 3.0) or 3.0)

            # Al menos una modalidad
            if not (self._require_image or self._require_gps or self._require_lidar):
                goal_handle.abort()
                return TakeSnapshot.Result(
                    success=False, error='No modalities requested',
                    output_dir='', basename='', image_path='', points_path='', gps_path='', tf_path=''
                )

            # Marca tiempo de inicio del goal (ROS clock del nodo) y resetea save-once
            self._goal_start_sec = self.get_clock().now().nanoseconds / 1e9
            self._save_triggered = False

            # Limpia caches de modalidades solicitadas (para no arrastrar mensajes previos)
            with self._cache_lock:
                if self._require_image:
                    self.cache_img.clear()
                if self._require_gps:
                    self.cache_gps.clear()
                if self._require_lidar:
                    self.cache_lidar.clear()

            self._goal_in_progress = True

        goal_handle.publish_feedback(TakeSnapshot.Feedback(state='syncing'))

        # Espera a que _do_save señale el evento
        if not self._goal_event.wait(timeout=self._timeout_sec):
            with self._goal_lock:
                self._goal_in_progress = False
                self._save_triggered = False
            goal_handle.abort()
            return TakeSnapshot.Result(
                success=False, error='Timeout waiting for requested modalities',
                output_dir='', basename='', image_path='', points_path='', gps_path='', tf_path=''
            )

        # Construir resultado
        time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path = self._last_result
        goal_handle.publish_feedback(TakeSnapshot.Feedback(state='saving'))

        result = TakeSnapshot.Result(
            success=True,
            error='',
            output_dir=snapshot_dir,
            basename=time_str,
            image_path=img_path,
            points_path=npy_path,
            gps_path=gps_path,
            tf_path=tf_path
        )
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = SnapshotSaverAction()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
