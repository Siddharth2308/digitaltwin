"""ROS2 node: drive the RViz twin from LIVE machine position feedback.

Subscribes to the machine's position (per machine_state.yaml), scales each axis from its
physical workspace onto the URDF joint limits, and publishes /joint_states at a fixed rate.
Use this INSTEAD of joint_state_publisher_gui and INSTEAD of rviz_player -- all three drive
/joint_states, so run only one at a time.

    ros2 run kerala_bot_sim state_listener --ros-args -p config:=/abs/path/machine_state.yaml
"""
from __future__ import annotations

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .mapping import scale, in_range, units_factor


class StateListener(Node):
    def __init__(self):
        super().__init__("state_listener")
        self.declare_parameter("config", "")
        cfg_path = self.get_parameter("config").value
        if not cfg_path:
            raise RuntimeError("'config' parameter (path to machine_state.yaml) is required")
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        self.mode = cfg["input"]["mode"]
        self.unit_f = units_factor(cfg["input"].get("units", "m"))
        self.clamp = bool(cfg.get("clamp_to_limits", True))

        # active axes = enabled AND wired to a real URDF joint
        self.axes = {}
        for name, a in cfg["axes"].items():
            if a.get("enabled") and a.get("joint"):
                self.axes[name] = a
                # start at machine_min so the pose is defined before data arrives
                a["_value_m"] = float(a["machine_min"]) * self.unit_f
        if not self.axes:
            raise RuntimeError("no enabled axes with a joint; nothing to publish")

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self._warned = set()
        self._subscribe(cfg)

        rate = float(cfg.get("publish_rate_hz", 30.0))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"state_listener: mode={self.mode}, axes={list(self.axes)} -> "
            f"joints {[a['joint'] for a in self.axes.values()]}")

    # ---- subscriptions ------------------------------------------------------
    def _subscribe(self, cfg):
        if self.mode in ("point", "point_stamped"):
            topic = cfg["input"]["topic"]
            if self.mode == "point":
                from geometry_msgs.msg import Point
                self.create_subscription(Point, topic, self._on_point, 10)
            else:
                from geometry_msgs.msg import PointStamped
                self.create_subscription(PointStamped, topic, self._on_point_stamped, 10)
            self.get_logger().info(f"subscribed {topic} ({self.mode})")
        elif self.mode == "float_axes":
            from std_msgs.msg import Float64
            for name, a in self.axes.items():
                topic = a["topic"]
                self.create_subscription(
                    Float64, topic, lambda m, n=name: self._on_float(n, m), 10)
                self.get_logger().info(f"subscribed {topic} -> axis {name}")
        else:
            raise RuntimeError(f"unknown input mode '{self.mode}'")

    def _set(self, name, raw_value):
        if name not in self.axes:
            return
        a = self.axes[name]
        val_m = float(raw_value) * self.unit_f
        mn = float(a["machine_min"]) * self.unit_f
        mx = float(a["machine_max"]) * self.unit_f
        if not in_range(val_m, mn, mx) and name not in self._warned:
            self.get_logger().warn(
                f"axis {name} value {val_m:.4f} m outside machine range "
                f"[{mn:.4f},{mx:.4f}] -- clamping (further warnings suppressed)")
            self._warned.add(name)
        a["_value_m"] = val_m

    def _on_point(self, msg):
        self._set("x", msg.x); self._set("y", msg.y); self._set("z", msg.z)

    def _on_point_stamped(self, msg):
        self._set("x", msg.point.x); self._set("y", msg.point.y); self._set("z", msg.point.z)

    def _on_float(self, name, msg):
        self._set(name, msg.data)

    # ---- publish ------------------------------------------------------------
    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for a in self.axes.values():
            mn = float(a["machine_min"]) * self.unit_f
            mx = float(a["machine_max"]) * self.unit_f
            j = scale(a["_value_m"], mn, mx,
                      float(a["joint_min"]), float(a["joint_max"]), clamp=self.clamp)
            msg.name.append(a["joint"])
            msg.position.append(j)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
