"""ROS2 node: replay a logged simulation into RViz as /joint_states over time.

Reads the sim log (t, x_cmd, y_cmd, x_act, y_act) and an axis_map (bed->joint mapping),
then publishes sensor_msgs/JointState stepping through the log in (optionally scaled) real
time. Run this INSTEAD of joint_state_publisher_gui (they both drive /joint_states).

    ros2 run kerala_bot_sim rviz_player --ros-args \
        -p log:=/abs/path/sim_log.csv -p axis_map:=/abs/path/axis_map.yaml -p rate:=1.0
"""
from __future__ import annotations

import csv

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .mapping import scale


def _load_log(path):
    rows = {"t": [], "x_cmd": [], "y_cmd": [], "x_act": [], "y_act": []}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for k in rows:
                rows[k].append(float(row[k]))
    return rows


def _map(pos, m):
    return scale(pos, m["bed_min"], m["bed_max"], m["joint_min"], m["joint_max"], clamp=False)


class RvizPlayer(Node):
    def __init__(self):
        super().__init__("rviz_player")
        self.declare_parameter("log", "")
        self.declare_parameter("axis_map", "")
        self.declare_parameter("rate", 1.0)      # playback speed multiplier
        self.declare_parameter("loop", True)

        log_path = self.get_parameter("log").value
        map_path = self.get_parameter("axis_map").value
        self.rate = float(self.get_parameter("rate").value)
        self.loop = bool(self.get_parameter("loop").value)

        if not log_path or not map_path:
            raise RuntimeError("both 'log' and 'axis_map' parameters are required")

        self.rows = _load_log(log_path)
        with open(map_path) as f:
            self.amap = yaml.safe_load(f)

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.i = 0
        self.n = len(self.rows["t"])
        self.joint_x = self.amap["x"]["joint"]
        self.joint_y = self.amap["y"]["joint"]
        self.src_x = self.amap["x"].get("source", "x_act")
        self.src_y = self.amap["y"].get("source", "y_act")

        # publish at the log's own sample spacing, scaled by 'rate'
        dt = (self.rows["t"][1] - self.rows["t"][0]) if self.n > 1 else 0.01
        self.timer = self.create_timer(max(dt / self.rate, 1e-3), self.tick)
        self.get_logger().info(
            f"replaying {self.n} samples: {self.joint_x} <- {self.src_x}, "
            f"{self.joint_y} <- {self.src_y} (rate x{self.rate})")

    def tick(self):
        if self.i >= self.n:
            if not self.loop:
                self.get_logger().info("playback done")
                self.timer.cancel()
                return
            self.i = 0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_x, self.joint_y]
        msg.position = [
            _map(self.rows[self.src_x][self.i], self.amap["x"]),
            _map(self.rows[self.src_y][self.i], self.amap["y"]),
        ]
        self.pub.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = RvizPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
