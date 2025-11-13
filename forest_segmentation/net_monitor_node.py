#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 Ping Monitor (mínimo)
---------------------------
Comprueba conectividad a Internet haciendo ping a una lista de hosts.
Publica cada 'ping_interval_sec' en:

/net/status        -> std_msgs/String (JSON)
                    {
                      "stamp": <epoch_sec>,
                      "up": true/false,
                      "host": "8.8.8.8",
                      "tx": 3, "rx": 3,
                      "loss_pct": 0.0,
                      "rtt_min_ms": 23.1,
                      "rtt_avg_ms": 29.3,
                      "rtt_max_ms": 34.1,
                      "rtt_mdev_ms": 3.8
                    }

/net/diagnostics   -> diagnostic_msgs/DiagnosticArray
                     level=0 OK si up, 2 ERROR si down.

Parámetros:
- ping_hosts_csv (str): lista separada por comas. Default: "8.8.8.8,1.1.1.1,google.com"
- ping_count (int): paquetes por sonda. Default: 3
- ping_deadline_sec (int): timeout de ping. Default: 3
- ping_interval_sec (float): periodo entre sondas. Default: 5.0

Dependencia: iputils-ping (viene por defecto en Ubuntu).


ros2 run forest_segmentation net_monitor_node --ros-args   -p ping_hosts_csv:=8.8.8.8   -p ping_count:=1   -p ping_deadline_sec:=1   -p ping_interval_sec:=1.0
"""

import json, re, shlex, subprocess, time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def run_ping(host: str, count: int, deadline: int) -> Tuple[Dict[str, float], Dict[str, int]]:
    """Ejecuta ping y devuelve métricas (latencias y pérdidas) + tx/rx."""
    try:
        cmd = f"ping -n -c {count} -w {deadline} {host}"
        out = subprocess.run(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             timeout=deadline + 1.5, check=False, text=True).stdout
    except Exception:
        return ({"min": float("nan"), "avg": float("nan"), "max": float("nan"), "mdev": float("nan")},
                {"tx": 0, "rx": 0, "loss": 100.0})

    # Paquetes tx/rx/pérdida
    tx = rx = 0
    loss = 100.0
    m_loss = re.search(r"(\d+)\s+packets transmitted,\s+(\d+)\s+received.*?(\d+)% packet loss", out)
    if m_loss:
        tx, rx, loss = int(m_loss.group(1)), int(m_loss.group(2)), float(m_loss.group(3))

    # Latencias (dos formatos posibles)
    lat = {"min": float("nan"), "avg": float("nan"), "max": float("nan"), "mdev": float("nan")}
    m_rtt = re.search(r"rtt\s+min/avg/max/mdev\s*=\s*([\d\.]+)/([\d\.]+)/([\d\.]+)/([\d\.]+)\s*ms", out)
    if not m_rtt:
        m_rtt = re.search(r"round-trip\s+min/avg/max/(?:stddev|mdev)\s*=\s*([\d\.]+)/([\d\.]+)/([\d\.]+)/([\d\.]+)\s*ms", out)
    if m_rtt:
        lat["min"], lat["avg"], lat["max"], lat["mdev"] = map(float, m_rtt.groups())

    return (lat, {"tx": tx, "rx": rx, "loss": loss})


class PingMonitorNode(Node):
    def __init__(self):
        super().__init__("ping_monitor")

        # Parámetros
        self.declare_parameter("ping_hosts_csv", "8.8.8.8,1.1.1.1,google.com")
        self.declare_parameter("ping_count", 3)
        self.declare_parameter("ping_deadline_sec", 3)
        self.declare_parameter("ping_interval_sec", 5.0)

        self.hosts = [h.strip() for h in self.get_parameter("ping_hosts_csv").value.split(",") if h.strip()]
        self.count = int(self.get_parameter("ping_count").value)
        self.deadline = int(self.get_parameter("ping_deadline_sec").value)
        self.interval = float(self.get_parameter("ping_interval_sec").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_status = self.create_publisher(String, "/net/status", qos)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/net/diagnostics", qos)

        self.timer = self.create_timer(max(0.1, self.interval), self._tick)
        self.get_logger().info(f"ping_monitor listo. Hosts: {self.hosts} (cada {self.interval}s)")

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        best: Optional[Dict] = None
        # Probar hosts en orden; el primero que responda, publica y termina
        for host in self.hosts:
            lat, pkt = run_ping(host, self.count, self.deadline)
            up = pkt["rx"] > 0 and pkt["loss"] < 100.0
            result = {
                "stamp": now,
                "up": up,
                "host": host,
                "tx": pkt["tx"],
                "rx": pkt["rx"],
                "loss_pct": pkt["loss"],
                "rtt_min_ms": lat["min"],
                "rtt_avg_ms": lat["avg"],
                "rtt_max_ms": lat["max"],
                "rtt_mdev_ms": lat["mdev"],
            }
            if up:
                best = result
                break
            # si no hay respuesta, guarda para publicar el último intento (down)
            best = result

        # Publicar /net/status (JSON)
        msg = String()
        msg.data = json.dumps(best, ensure_ascii=False)
        self.pub_status.publish(msg)

        # Publicar /net/diagnostics
        kv = [KeyValue(key=k, value=str(v)) for k, v in best.items()]
        stat = DiagnosticStatus(name="ping_monitor",
                                message="OK" if best["up"] else "NO_LINK",
                                hardware_id=best["host"],
                                values=kv)
        stat.level = bytes([0 if best["up"] else 2])  # 0 OK, 2 ERROR
        arr = DiagnosticArray(status=[stat])
        # arr.header.stamp no es crítico, pero podemos dejarlo vacío o convertir now si quieres
        self.pub_diag.publish(arr)


def main():
    rclpy.init()
    node = PingMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
