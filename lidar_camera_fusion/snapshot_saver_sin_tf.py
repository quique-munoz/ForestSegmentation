#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, NavSatFix
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
import message_filters
import cv2
import numpy as np
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class SnapshotSaver(Node):
    def __init__(self):
        super().__init__('snapshot_saver')
        self.bridge = CvBridge()
        self.snapshot_done = False

        # --- Guardar dentro del paquete ---
        target_pkg = 'lidar_camera_fusion'
        pkg_share_path = get_package_share_directory(target_pkg)
        src_pkg_path = os.path.abspath(os.path.join(pkg_share_path, '../../../../src', target_pkg))        

        # === Crear carpeta raíz "snapshots" ===
        base_snap_dir = os.path.join(src_pkg_path, 'snapshots')
        os.makedirs(base_snap_dir, exist_ok=True)

        # === Crear subcarpeta con timestamp ===
        run_stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.output_dir = os.path.join(base_snap_dir, run_stamp)
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'Guardando archivos en: {self.output_dir}')

        # --- Suscripciones ---
        image_sub = message_filters.Subscriber(self, Image, '/gmsl_camera/port_0/cam_0/image_raw')
        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/LiDAR_1/points_raw')
        gps_sub   = message_filters.Subscriber(self, NavSatFix, '/piksi/navsatfix_best_fix')

        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.caminfo = None
        self.create_subscription(CameraInfo, '/gmsl_camera/port_0/cam_0/camera_info', self.caminfo_cb, 1)

        # --- Sincronizador ---
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub, gps_sub], 10, 0.05)
        ts.registerCallback(self.synced_callback)

        self.get_logger().info(f'SnapshotSaver inicializado. Guardará 1 snapshot en: {self.output_dir}')

    def caminfo_cb(self, msg):
        if self.caminfo is None:
            self.caminfo = msg
            camfile = os.path.join(self.output_dir, 'camera_info.json')
            with open(camfile, 'w') as f:
                json.dump({
                    'K': list(msg.k),
                    'D': list(msg.d),
                    'width': msg.width,
                    'height': msg.height,
                    'frame_id': msg.header.frame_id
                }, f, indent=2)
            self.get_logger().info(f'CameraInfo guardado: {camfile}')

    def synced_callback(self, img_msg, lidar_msg, gps_msg):
        if self.snapshot_done:
            return  # ya se guardó una vez

        # --- Imagen ---
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir imagen: {e}")
            return

        # --- Nube ---
        if len(lidar_msg.data) == 0:
            self.get_logger().warn("Nube vacía, ignorando frame.")
            return

        try:
            points = np.frombuffer(lidar_msg.data, dtype=np.float32)
            points = points.reshape(-1, int(lidar_msg.point_step / 4))[:, :3]
            mask = np.isfinite(points).all(axis=1)
            points = points[mask]
            if points.shape[0] < 100:
                self.get_logger().warn("Nube con pocos puntos, ignorada.")
                return
        except Exception as e:
            self.get_logger().error(f"Error procesando nube: {e}")
            return

        # --- GPS ---
        gps_data = {
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude,
            'altitude': gps_msg.altitude
        }

        # --- TF (lidar -> cámara) ---
        if not self.caminfo:
            self.get_logger().warn("No se recibió CameraInfo aún.")
            return

        #try:
        #    tf = self.tf_buffer.lookup_transform('camera_left', 'lidar', rclpy.time.Time())
        #except Exception as e:
        #    self.get_logger().warn(f"No se pudo obtener TF (camera_left<-lidar): {e}")
        #    return

        # --- Guardar todos los datos ---
        timestamp = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        time_str = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S_%f")

        img_path = os.path.join(self.output_dir, f'{time_str}_image.png')
        npy_path = os.path.join(self.output_dir, f'{time_str}_points.npy')
        gps_path = os.path.join(self.output_dir, f'{time_str}_gps.json')
        #tf_path  = os.path.join(self.output_dir, f'{time_str}_tf.json')

        cv2.imwrite(img_path, img)
        np.save(npy_path, points)
        with open(gps_path, 'w') as f:
            json.dump(gps_data, f, indent=2)

        #tf_dict = {
        #    'translation': {
        #        'x': tf.transform.translation.x,
        #        'y': tf.transform.translation.y,
        #        'z': tf.transform.translation.z
        #    },
        #    'rotation': {
        #        'x': tf.transform.rotation.x,
        #        'y': tf.transform.rotation.y,
        #        'z': tf.transform.rotation.z,
        #        'w': tf.transform.rotation.w
        #    }
        #}
        #with open(tf_path, 'w') as f:
        #    json.dump(tf_dict, f, indent=2)

        self.get_logger().info(f'Snapshot completo guardado en {time_str}')
        self.snapshot_done = True

        # --- Apagar nodo tras guardar ---
        self.get_logger().info("Snapshot finalizado, cerrando nodo...")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SnapshotSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
