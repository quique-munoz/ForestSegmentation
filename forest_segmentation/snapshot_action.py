#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, NavSatFix
import message_filters
import tf2_ros
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from forest_segmentation_interfaces.action import TakeSnapshot


class SnapshotSaverAction(Node):
    def __init__(self):
        super().__init__('snapshot_saver_action')
        self.bridge = CvBridge()

        # ------------------ Parámetros ------------------
        self.declare_parameter('image_topic',   '/gmsl_camera/port_0/cam_0/image_raw')
        self.declare_parameter('caminfo_topic', '/gmsl_camera/port_0/cam_0/camera_info')
        self.declare_parameter('lidar_topic',   '/LiDAR_1/points_raw')
        self.declare_parameter('gps_topic',     '/piksi/navsatfix_best_fix')
        self.declare_parameter('camera_frame',  'camera_left')
        self.declare_parameter('lidar_frame',   'lidar')
        self.declare_parameter('base_snap_dir', '')          # si vacío, se intenta resolver dentro del paquete
        self.declare_parameter('sync_slop_sec_default', 0.05)  # slop por defecto (se puede sobrescribir por goal)
        self.declare_parameter('queue_size', 10)

        image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        caminfo_topic = self.get_parameter('caminfo_topic').get_parameter_value().string_value
        lidar_topic   = self.get_parameter('lidar_topic').get_parameter_value().string_value
        gps_topic     = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame  = self.get_parameter('lidar_frame').get_parameter_value().string_value
        slop_default = float(self.get_parameter('sync_slop_sec_default').get_parameter_value().double_value)
        queue_size    = int(self.get_parameter('queue_size').get_parameter_value().integer_value)

        base_snap_dir = self.get_parameter('base_snap_dir').get_parameter_value().string_value
        if not base_snap_dir:
            # Resolver carpeta snapshots dentro del paquete si existe, si no, ~/
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

        # ------------------ Infraestructura ------------------
        self.cb_group = ReentrantCallbackGroup()

        # Suscriptores y sincronizador (parametrizados)
        self.image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=10)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, lidar_topic, qos_profile=10)
        self.gps_sub   = message_filters.Subscriber(self, NavSatFix, gps_topic, qos_profile=10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub, self.gps_sub],
            queue_size=queue_size,
            slop=slop_default,
            allow_headerless=False
        )
        self.ts.slop = Duration(seconds=slop_default)
        self.ts.registerCallback(self._synced_callback)

        # CameraInfo (no bloquea; se guarda si el goal lo pide y está disponible)
        self.caminfo = None
        self.create_subscription(CameraInfo, caminfo_topic, self._caminfo_cb, 10, callback_group=self.cb_group)

        # TF (cache por defecto)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estado por goal
        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._goal_event = threading.Event()
        self._last_result = None
        self._save_caminfo = False
        self._goal_in_progress = False

        # Action server
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

    # ------------------ Utilidades ------------------
    @staticmethod
    def _file_prefix_from_stamp(stamp_sec: float) -> str:
        return datetime.fromtimestamp(stamp_sec).strftime("%Y%m%d_%H%M%S_%f")

    @staticmethod
    def _folder_name_now() -> str:
        return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def _caminfo_cb(self, msg: CameraInfo):
        if self.caminfo is None:
            self.get_logger().info('CameraInfo recibido por primera vez.')
        self.caminfo = msg

    # ------------------ Callback sincronizada ------------------
    def _synced_callback(self, img_msg: Image, lidar_msg: PointCloud2, gps_msg: NavSatFix):
        # Solo si hay goal activo y aún no se ha guardado para ese goal
        with self._goal_lock:
            if (self._active_goal is None or not self._active_goal.is_active or not self._goal_in_progress):
                return

        # Imagen
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir imagen: {e}")
            return

        # PointCloud2 -> numpy (XYZ float32 asumiendo layout compacto)
        if len(lidar_msg.data) == 0 or lidar_msg.point_step == 0:
            self.get_logger().warn("Nube vacía, ignorando frame.")
            return
        try:
            arr = np.frombuffer(lidar_msg.data, dtype=np.uint8)
            stride_f32 = int(lidar_msg.point_step // 4)
            pts = arr.view(np.float32).reshape(-1, stride_f32)[:, :3]
            pts = pts[np.isfinite(pts).all(axis=1)]
            if pts.shape[0] < 100:
                self.get_logger().warn("Nube con pocos puntos, ignorada.")
                return
        except Exception as e:
            self.get_logger().error(f"Error procesando nube: {e}")
            return

        # GPS
        gps_data = {
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude,
            'altitude': gps_msg.altitude
        }

        # TF estática (TIME_ZERO)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.lidar_frame, Time()  # TIME_ZERO
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener TF ({self.camera_frame}<-{self.lidar_frame}): {e}")
            return

        # ------ Guardado ------
        run_stamp = self._folder_name_now()
        snapshot_dir = os.path.join(self.base_snap_dir, run_stamp)
        os.makedirs(snapshot_dir, exist_ok=True)

        tsec = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        time_str = self._file_prefix_from_stamp(tsec)

        img_path = os.path.join(snapshot_dir, f'{time_str}_image.png')
        npy_path = os.path.join(snapshot_dir, f'{time_str}_points.npy')
        gps_path = os.path.join(snapshot_dir, f'{time_str}_gps.json')
        tf_path  = os.path.join(snapshot_dir, f'{time_str}_tf.json')
        caminfo_path = os.path.join(snapshot_dir, 'camera_info.json')

        cv2.imwrite(img_path, img)
        np.save(npy_path, pts)
        with open(gps_path, 'w') as f:
            json.dump(gps_data, f, indent=2)

        tf_dict = {
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
            'source_frame': self.lidar_frame
        }
        with open(tf_path, 'w') as f:
            json.dump(tf_dict, f, indent=2)

        if self._save_caminfo:
            if self.caminfo is not None:
                with open(caminfo_path, 'w') as f:
                    json.dump({
                        'K': list(self.caminfo.k),
                        'D': list(self.caminfo.d),
                        'width': int(self.caminfo.width),
                        'height': int(self.caminfo.height),
                        'frame_id': self.caminfo.header.frame_id
                    }, f, indent=2)
            else:
                self.get_logger().warn('Goal pedía CameraInfo, pero aún no se recibió; snapshot sin camera_info.json.')

        self._last_result = (time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path)
        with self._goal_lock:
            self._goal_in_progress = False
        self._goal_event.set()

    # ------------------ Action plumbing ------------------
    def _goal_cb(self, goal_request):
        return rclpy.action.GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        with self._goal_lock:
            if self._active_goal is not None and goal_handle == self._active_goal:
                self._goal_event.set()
                self._goal_in_progress = False
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        # Preparar estado del goal
        with self._goal_lock:
            if self._active_goal is not None and self._active_goal.is_active:
                self._active_goal.abort()
            self._active_goal = goal_handle
            self._goal_event.clear()
            self._last_result = None
            self._save_caminfo = bool(getattr(goal_handle.request, 'require_caminfo', False))
            self._goal_in_progress = True

        # slop por goal (float, NO Duration)
        slop = float(getattr(goal_handle.request, 'sync_slop_sec', 0.05) or 0.05)
        self.ts.slop = Duration(seconds=slop)

        goal_handle.publish_feedback(TakeSnapshot.Feedback(state="syncing"))

        timeout_sec = float(getattr(goal_handle.request, 'timeout_sec', 3.0) or 3.0)
        if not self._goal_event.wait(timeout=timeout_sec):
            with self._goal_lock:
                self._goal_in_progress = False
            goal_handle.abort()
            return TakeSnapshot.Result(
                success=False, error="Timeout waiting for synchronized data",
                output_dir='', basename='', image_path='', points_path='', gps_path='', tf_path=''
            )

        time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path = self._last_result
        goal_handle.publish_feedback(TakeSnapshot.Feedback(state="saving"))
        result = TakeSnapshot.Result(
            success=True, error='',
            output_dir=snapshot_dir, basename=time_str,
            image_path=img_path, points_path=npy_path, gps_path=gps_path, tf_path=tf_path
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
