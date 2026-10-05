#!/usr/bin/env python3
# test_tracking.py
#
# Tracking-only test for PoseTrackingWrapper.
#
#   1. Publish ready=False once  (consumes the wrapper's skip_initial_message)
#   2. Publish ready=True 0.5s later  -> wrapper switches to cartesian ctrl
#   3. Stream PoseStamped targets at 20 Hz forever
#   4. On Ctrl-C, publish ready=False to hand back to the arm controller
#
# Usage:
#   python3 test_tracking.py [left|right] [square|circle|line] [period_sec]
#
# Examples:
#   python3 test_tracking.py left square 5
#   python3 test_tracking.py right circle 6
#   python3 test_tracking.py left line 4

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
import tf2_ros
import time
import math
import sys


def euler_to_quaternion(roll, pitch, yaw):
    qx = (math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
          - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    qy = (math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
          + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2))
    qz = (math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
          - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2))
    qw = (math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
          + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    return [qx, qy, qz, qw]


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


class TrackingOnlyTester(Node):
    def __init__(self, side='left', shape='square', period=5.0):
        super().__init__('tracking_only_tester')

        self.side = side.lower()
        self.shape = shape.lower()
        self.period = period

        # ---------- per-side config ----------
        if self.side == 'right':
            pose_topic  = '/right_cartesian_motion_controller/target_frame'
            ready_topic = '/right_arm/ready'
            warn_topic  = '/right_arm/warn'
            self.ee_frame = 'tcp_r'
            self.target_frame_id = 'right_hand_frame'
            cx, cy, cz = 0.55, -0.30, 1.15
            radius = 0.10
        else:
            pose_topic  = '/left_cartesian_motion_controller/target_frame'
            ready_topic = '/left_arm/ready'
            warn_topic  = '/left_arm/warn'
            self.ee_frame = 'tcp_l'
            self.target_frame_id = 'left_hand_frame'
            cx, cy, cz = 0.60, 0.30, 1.15
            radius = 0.10

        self.center = (cx, cy, cz)
        self.radius = radius

        # ---------- QoS ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------- publishers ----------
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos)
        self.pose_pub  = self.create_publisher(PoseStamped, pose_topic, qos)

        # ---------- warn subscriber (visibility only) ----------
        self.warn_sub = self.create_subscription(
            Bool, warn_topic, self.warn_cb, qos)

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- state ----------
        self.start_pose = None
        self.start_time = None
        self.transition_sec = 3.0
        self._sent_false = False
        self._sent_true = False

        # ---------- timers ----------
        # Step 1: publish ready=False once, 0.5 s after startup
        self.create_timer(0.5, self._send_initial_false_once)
        # Step 2: publish ready=True once, 1.5 s after startup
        self.create_timer(1.5, self._send_initial_true_once)
        # Pose stream at 20 Hz
        self.pose_timer = self.create_timer(0.05, self.publish_target)

        self.get_logger().info(
            f'[{self.side.upper()}] tracking-only test started  '
            f'(shape={self.shape}, period={self.period}s)')

    # ---------- warn subscriber ----------
    def warn_cb(self, msg: Bool):
        self.get_logger().warn(
            f'>>> WARN RX on /{self.side}_arm/warn: data={msg.data}')

    # ---------- ready sequence ----------
    def _send_initial_false_once(self):
        if self._sent_false:
            return
        self._sent_false = True
        self.pub_ready(False)

    def _send_initial_true_once(self):
        if self._sent_true:
            return
        self._sent_true = True
        self.pub_ready(True)

    def pub_ready(self, state: bool):
        msg = Bool()
        msg.data = state
        self.ready_pub.publish(msg)
        self.get_logger().info(f'Published ready={state}')

    # ---------- pose stream ----------
    def publish_target(self):
        # Don't stream targets until we've sent ready=True.
        if not self._sent_true:
            return

        if self.start_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', self.ee_frame, rclpy.time.Time())
                self.start_pose = {
                    'x': tf.transform.translation.x,
                    'y': tf.transform.translation.y,
                    'z': tf.transform.translation.z,
                    'orientation': tf.transform.rotation,
                }
                self.start_time = time.time()
                self.get_logger().info(
                    f'Start pose captured: '
                    f'({self.start_pose["x"]:.3f}, '
                    f'{self.start_pose["y"]:.3f}, '
                    f'{self.start_pose["z"]:.3f})  '
                    f'-- streaming targets now')
            except Exception as e:
                self.get_logger().warn(
                    f'Waiting for TF [{self.ee_frame}]: {e}',
                    throttle_duration_sec=2.0)
                return

        elapsed = time.time() - self.start_time

        # ----- shape generator -----
        if self.shape == 'circle':
            w = 2.0 * math.pi / self.period
            tx = self.center[0] + self.radius * math.cos(w * elapsed)
            ty = self.center[1] + self.radius * math.sin(w * elapsed)
            tz = self.center[2]
        elif self.shape == 'line':
            s = math.sin(2.0 * math.pi * elapsed / self.period)
            tx = self.center[0] + self.radius * s
            ty = self.center[1]
            tz = self.center[2]
        else:  # square
            corners = [
                (self.center[0] + self.radius, self.center[1] - self.radius, self.center[2]),
                (self.center[0] - self.radius, self.center[1] - self.radius, self.center[2]),
                (self.center[0] - self.radius, self.center[1] + self.radius, self.center[2]),
                (self.center[0] + self.radius, self.center[1] + self.radius, self.center[2]),
            ]
            seg_t = self.period / len(corners)
            seg_i = int(elapsed // seg_t) % len(corners)
            seg_p = (elapsed % seg_t) / seg_t
            p1 = corners[seg_i]
            p2 = corners[(seg_i + 1) % len(corners)]
            tx = p1[0] + (p2[0] - p1[0]) * seg_p
            ty = p1[1] + (p2[1] - p1[1]) * seg_p
            tz = p1[2] + (p2[2] - p1[2]) * seg_p

        # ----- smooth glide from start_pose to the shape -----
        if elapsed < self.transition_sec:
            a = elapsed / self.transition_sec
            x = self.start_pose['x'] + (tx - self.start_pose['x']) * a
            y = self.start_pose['y'] + (ty - self.start_pose['y']) * a
            z = self.start_pose['z'] + (tz - self.start_pose['z']) * a
        else:
            x, y, z = tx, ty, tz

        # ----- build message -----
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # small orientation wiggle around the captured start orientation
        n_roll  = 0.05 * math.sin(elapsed * 2.1)
        n_pitch = 0.05 * math.cos(elapsed * 1.5)
        n_yaw   = 0.05 * math.sin(elapsed * 3.3)
        q_noise = euler_to_quaternion(n_roll, n_pitch, n_yaw)
        o = self.start_pose['orientation']
        q = quaternion_multiply([o.x, o.y, o.z, o.w], q_noise)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_pub.publish(msg)

        # broadcast target TF for RViz
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.target_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    # ---------- shutdown ----------
    def destroy_node(self):
        try:
            self.get_logger().info('Shutting down -- publishing ready=False')
            self.pub_ready(False)
            time.sleep(0.2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    side = 'left'
    shape = 'square'
    period = 5.0

    # CLI: test_tracking.py [left|right] [square|circle|line] [period_sec]
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(argv) > 0 and argv[0].lower() in ('left', 'right'):
        side = argv[0].lower()
    if len(argv) > 1 and argv[1].lower() in ('square', 'circle', 'line'):
        shape = argv[1].lower()
    if len(argv) > 2:
        period = float(argv[2])

    node = TrackingOnlyTester(side, shape, period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()