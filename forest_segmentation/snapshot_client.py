#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from forest_segmentation_interfaces.action import TakeSnapshot


class SnapshotClient(Node):
    def __init__(self):
        super().__init__('snapshot_client')

        # Parámetros (puedes sobreescribir con --ros-args -p ...)
        self.declare_parameter('period_sec', 15.0)     # periodo entre snapshots
        self.declare_parameter('sync_slop_sec', 0.1) # tolerancia de sync

        self.period = float(self.get_parameter('period_sec').value)
        self.slop = float(self.get_parameter('sync_slop_sec').value)

        self.cli = ActionClient(self, TakeSnapshot, 'take_snapshot')

        self.first_goal = True           # solo la primera vez pedimos CameraInfo
        self.goal_in_flight = False
        self.count = 0

        # Espera (no bloqueante) a que el servidor esté disponible y arranca el timer
        self._wait_for_server_and_start()

    def _wait_for_server_and_start(self):
        self.get_logger().info('Esperando action server /take_snapshot...')
        # Lanza un timer corto que reintenta hasta que el servidor esté
        self.wait_timer = self.create_timer(0.2, self._try_start)

    def _try_start(self):
        if self.cli.wait_for_server(timeout_sec=0.0):
            self.get_logger().info('Servidor /take_snapshot disponible. Iniciando envíos periódicos.')
            self.destroy_timer(self.wait_timer)
            self.timer = self.create_timer(self.period, self._timer_cb)

    def _timer_cb(self):
        # No dispares si hay un goal en curso
        if self.goal_in_flight:
            self.get_logger().warn('Goal anterior aún en curso; salto este periodo.')
            return

        # Preparar goal
        goal = TakeSnapshot.Goal()
        goal.sync_slop_sec = float(self.slop)
        goal.require_caminfo = bool(self.first_goal)

        self.get_logger().info(
            f'Enviando snapshot: slop={goal.sync_slop_sec:.3f}, '
            f'require_caminfo={goal.require_caminfo}'
        )

        self.goal_in_flight = True

        send_future = self.cli.send_goal_async(goal, feedback_callback=self._fb_cb)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado por el servidor.')
            self.goal_in_flight = False
            return

        self.get_logger().info('Goal aceptado; esperando resultado...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'✅ Snapshot OK: dir={result.output_dir}')
            self.get_logger().info(f'   imagen: {result.image_path}')
            self.get_logger().info(f'   nube:   {result.points_path}')
            self.get_logger().info(f'   gps:    {result.gps_path}')
            self.get_logger().info(f'   tf:     {result.tf_path}')
        else:
            self.get_logger().error(f'❌ Snapshot fallo: {result.error}')

        # Actualiza estado para el próximo tick
        self.goal_in_flight = False
        if self.first_goal:
            self.first_goal = False
        self.count += 1

    def _fb_cb(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.state}')


def main():
    rclpy.init()
    node = SnapshotClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
