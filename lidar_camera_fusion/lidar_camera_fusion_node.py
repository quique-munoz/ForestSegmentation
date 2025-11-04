#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from scipy.spatial.transform import Rotation as R

class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        self.bridge = CvBridge()
        self.K = None
        self.image = None

        # === Suscripciones ===
        self.create_subscription(PointCloud2, '/LiDAR_1/points_raw', self.lidar_callback, 10)
        self.create_subscription(Image, '/gmsl_camera/port_0/cam_0/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/gmsl_camera/port_0/cam_0/camera_info', self.caminfo_callback, 10)

        # === TF ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("✅ Nodo LidarCameraFusion iniciado correctamente.")

    def caminfo_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def lidar_callback(self, msg):
        if self.image is None or self.K is None:
            return

        # Leer nube de puntos
        points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(points) == 0:
            return

        try:
            # Buscar transformación lidar -> cámara
            trans = self.tf_buffer.lookup_transform(
                'camera_left',  # Frame de destino (ajusta si tu cámara tiene otro nombre)
                msg.header.frame_id,  # 'lidar'
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener TF: {e}")
            return

        # Extraer rotación y traslación
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w

        # Matriz homogénea 4x4
        r = R.from_quat([qx, qy, qz, qw])
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = [tx, ty, tz]

        # Transformar nube al frame de la cámara
        points_h = np.hstack((points, np.ones((points.shape[0], 1))))
        points_cam = (T @ points_h.T).T[:, :3]

        # Filtrar puntos delante de la cámara
        mask_front = points_cam[:, 2] > 0
        points_cam = points_cam[mask_front]

        # Proyección 3D → 2D
        uv = (self.K @ points_cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]

        # Filtrar los puntos dentro del tamaño de imagen
        H, W = self.image.shape[:2]
        valid_mask = (
            (uv[:, 0] >= 0) & (uv[:, 0] < W) &
            (uv[:, 1] >= 0) & (uv[:, 1] < H)
        )
        uv = uv[valid_mask].astype(int)

        # Visualizar puntos sobre la imagen
        img_vis = self.image.copy()
        for (u, v) in uv:
            cv2.circle(img_vis, (u, v), 2, (0, 255, 0), -1)

        cv2.imshow("LiDAR Projection", img_vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LidarCameraFusion()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
