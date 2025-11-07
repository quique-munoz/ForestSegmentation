#!/usr/bin/env python3
import os
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, NavSatFix
import message_filters
import tf2_ros
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from forest_segmentation_interfaces.action import TakeSnapshot


class SnapshotSaverAction(Node):
    def __init__(self):
        super().__init__('snapshot_saver_action')
        self.bridge = CvBridge()

        # Raíz donde guardar (no se crean subcarpetas aún)
        target_pkg = 'forest_segmentation'
        pkg_share_path = get_package_share_directory(target_pkg)
        src_pkg_path = os.path.abspath(os.path.join(pkg_share_path, '../../../../src', target_pkg))
        self.base_snap_dir = os.path.join(src_pkg_path, 'snapshots')
        os.makedirs(self.base_snap_dir, exist_ok=True)
        self.get_logger().info(f'Raíz de snapshots: {self.base_snap_dir}')

        # Callback group reentrante (para no bloquear con el ActionServer)
        self.cb_group = ReentrantCallbackGroup()

        # Suscriptores + sincronizador (slop float en constructor)
        self.image_sub = message_filters.Subscriber(self, Image, '/gmsl_camera/port_0/cam_0/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/LiDAR_1/points_raw')
        self.gps_sub   = message_filters.Subscriber(self, NavSatFix, '/piksi/navsatfix_best_fix')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub, self.gps_sub],
            queue_size=10,
            slop=0.05  # float aquí; más tarde lo pondremos como Duration por-goal
        )
        self.ts.registerCallback(self._synced_callback)

        # CameraInfo: solo en memoria; se guardará dentro del snapshot si el goal lo pide
        self.caminfo = None
        self.create_subscription(
            CameraInfo,
            '/gmsl_camera/port_0/cam_0/camera_info',
            self._caminfo_cb,
            1,
            callback_group=self.cb_group
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_frame = 'camera_left'
        self.lidar_frame = 'lidar'

        # Estado por goal
        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._goal_event = threading.Event()
        self._last_result = None       # (time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path)
        self._save_caminfo = False     # si ese goal quiere que intentemos guardar CameraInfo
        self._goal_in_progress = False # asegura UNA sola captura por goal

        # Servidor de acciones (en grupo reentrante)
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

    # ===== Utilidades =====
    @staticmethod
    def _file_prefix_from_stamp(stamp_sec: float) -> str:
        """YYYYMMDD_HHMMSS_microseg (para prefijo de ficheros)."""
        return datetime.fromtimestamp(stamp_sec).strftime("%Y%m%d_%H%M%S_%f")

    @staticmethod
    def _folder_name_now() -> str:
        """YYYY-MM-DD_HH-MM-SS (para carpeta del snapshot, creada al llegar el goal)."""
        return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def _caminfo_cb(self, msg: CameraInfo):
        # No bloqueamos nunca; si está cuando llegue el snapshot y el goal lo pide, se guarda
        if self.caminfo is None:
            self.get_logger().info('CameraInfo recibido por primera vez.')
        self.caminfo = msg

    def _synced_callback(self, img_msg: Image, lidar_msg: PointCloud2, gps_msg: NavSatFix):
        # Solo si hay un goal activo y aún no se ha guardado para ese goal
        with self._goal_lock:
            if (self._active_goal is None or not self._active_goal.is_active or not self._goal_in_progress):
                return

        # Imagen
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir imagen: {e}")
            return

        # Nube -> numpy (ajusta si tu PointCloud2 no es XYZ flotante "compacto")
        if len(lidar_msg.data) == 0:
            self.get_logger().warn("Nube vacía, ignorando frame.")
            return
        try:
            points = np.frombuffer(lidar_msg.data, dtype=np.float32)
            points = points.reshape(-1, int(lidar_msg.point_step / 4))[:, :3]
            points = points[np.isfinite(points).all(axis=1)]
            if points.shape[0] < 100:
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

        # TF estática (ignorar timestamp → TIME_ZERO)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame,     # target
                self.lidar_frame,      # source
                Time(),                # TIME_ZERO: ignora el stamp
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener TF ({self.camera_frame}<-{self.lidar_frame}): {e}")
            return

        # ======= Crear carpeta (al llegar el goal) y guardar =======
        run_stamp = self._folder_name_now()
        snapshot_dir = os.path.join(self.base_snap_dir, run_stamp)
        os.makedirs(snapshot_dir, exist_ok=True)

        # Prefijo de ficheros usando el stamp de la imagen
        tsec = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        time_str = self._file_prefix_from_stamp(tsec)

        img_path = os.path.join(snapshot_dir, f'{time_str}_image.png')
        npy_path = os.path.join(snapshot_dir, f'{time_str}_points.npy')
        gps_path = os.path.join(snapshot_dir, f'{time_str}_gps.json')
        tf_path  = os.path.join(snapshot_dir, f'{time_str}_tf.json')
        caminfo_path = os.path.join(snapshot_dir, 'camera_info.json')  # como en tu nodo original

        cv2.imwrite(img_path, img)
        np.save(npy_path, points)
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
            }
        }
        with open(tf_path, 'w') as f:
            json.dump(tf_dict, f, indent=2)

        # Guardar CameraInfo SOLO si el goal lo pidió y hay caminfo disponible ahora
        if self._save_caminfo:
            if self.caminfo is not None:
                with open(caminfo_path, 'w') as f:
                    json.dump({
                        'K': list(self.caminfo.k),
                        'D': list(self.caminfo.d),
                        'width': self.caminfo.width,
                        'height': self.caminfo.height,
                        'frame_id': self.caminfo.header.frame_id
                    }, f, indent=2)
            else:
                self.get_logger().warn('Goal pedía CameraInfo, pero aún no se recibió; snapshot sin camera_info.json.')

        # Resultado para el execute_callback (una sola vez por goal)
        self._last_result = (time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path)
        with self._goal_lock:
            self._goal_in_progress = False  # evita guardados adicionales en este goal
        self._goal_event.set()

    # ===== Action plumbing =====
    def _goal_cb(self, goal_request):
        return rclpy.action.GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        with self._goal_lock:
            if self._active_goal is not None and goal_handle == self._active_goal:
                self._goal_event.set()
                self._goal_in_progress = False
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        # Preparar estado del goal (habilitar una sola captura)
        with self._goal_lock:
            if self._active_goal is not None and self._active_goal.is_active:
                self._active_goal.abort()
            self._active_goal = goal_handle
            self._goal_event.clear()
            self._last_result = None
            self._save_caminfo = bool(goal_handle.request.require_caminfo)
            self._goal_in_progress = True

        # Ajustar slop por goal (ponerlo como Duration para evitar TypeError en message_filters)
        slop = goal_handle.request.sync_slop_sec or 0.05
        self.ts.slop = Duration(seconds=float(slop))

        goal_handle.publish_feedback(TakeSnapshot.Feedback(state="syncing"))

        # Espera a que _synced_callback complete el guardado (sin bloquear el executor)
        timeout_sec = 3.0
        if not self._goal_event.wait(timeout=timeout_sec):
            with self._goal_lock:
                self._goal_in_progress = False
            goal_handle.abort()
            return TakeSnapshot.Result(success=False, error="Timeout waiting for synchronized data")

        # Construir resultado y finalizar
        time_str, snapshot_dir, img_path, npy_path, gps_path, tf_path = self._last_result
        goal_handle.publish_feedback(TakeSnapshot.Feedback(state="saving"))

        result = TakeSnapshot.Result(
            success=True,
            error="",
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
        # MultiThreadedExecutor para que action y subscriptores avancen en paralelo
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
