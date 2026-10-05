#!/usr/bin/env python3
# test_pose_tracking.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
import time
import math
import sys


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return [qx, qy, qz, qw]


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]


class PoseTrackingTester(Node):
    def __init__(self, side='left',
                 ready_on_sec=8.0,
                 ready_off_sec=5.0,
                 start_ready=False,
                 auto_cycle=True):
        super().__init__('pose_tracking_tester')

        self.side = side.lower()

        # -------- per-side configuration --------
        if self.side == 'right':
            pose_topic  = '/right_cartesian_motion_controller/target_frame'
            ready_topic = '/right_arm/ready'
            warn_topic  = '/right_arm/warn'
            self.ee_frame = 'tcp_r'
            self.target_frame_id = 'right_target_frame'
            self.square_waypoints = [
                (0.6, -0.25, 1.0),
                (0.5, -0.35, 1.0),
                (0.5, -0.35, 1.3),
                (0.6, -0.25, 1.3),
            ]
        else:
            pose_topic  = '/left_cartesian_motion_controller/target_frame'
            ready_topic = '/left_arm/ready'
            warn_topic  = '/left_arm/warn'
            self.ee_frame = 'tcp_l'
            self.target_frame_id = 'left_target_frame'
            self.square_waypoints = [
                (0.7, 0.25, 1.0),
                (0.5, 0.35, 1.0),
                (0.5, 0.35, 1.3),
                (0.7, 0.25, 1.3),
            ]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -------- publishers --------
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos)
        self.pose_pub  = self.create_publisher(PoseStamped, pose_topic, qos)

        # -------- subscriber: watch the wrapper's warn pulses --------
        self.warn_sub = self.create_subscription(
            Bool, warn_topic, self.warn_cb, qos)

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # -------- test state --------
        self.auto_cycle   = auto_cycle
        self.ready_on_sec = ready_on_sec
        self.ready_off_sec = ready_off_sec
        self.ready_state  = bool(start_ready)
        self.last_toggle_time = time.time()

        self.start_pose = None
        self.start_time = None
        self.time_per_segment = 5.0

        # -------- timers --------
        # 20 Hz pose publisher
        self.pose_timer  = self.create_timer(0.05, self.publish_target)
        # 1 Hz state machine — toggles ready
        self.state_timer = self.create_timer(1.0, self.state_tick)

        # Publish the initial ready state ONCE so the wrapper's
        # skip_initial_message logic consumes it.
        self.get_logger().info(
            f'[{self.side.upper()}] publishing initial ready={self.ready_state}')
        self.pub_ready(self.ready_state)

        self.get_logger().info(
            f'Initialized. auto_cycle={self.auto_cycle} '
            f'ready_on_sec={self.ready_on_sec} '
            f'ready_off_sec={self.ready_off_sec}')

    # ---------- warn subscriber ----------
    def warn_cb(self, msg: Bool):
        self.get_logger().warn(
            f'>>> [WARN RX on /{self.side}_arm/warn] data={msg.data}')

    # ---------- state machine ----------
    def state_tick(self):
        if not self.auto_cycle:
            return
        now = time.time()
        elapsed = now - self.last_toggle_time

        if self.ready_state and elapsed >= self.ready_on_sec:
            self.get_logger().info(
                f'=== READY EDGE: true -> false  ('
                f'{elapsed:.2f}s in tracking) ===')
            self.ready_state = False
            self.pub_ready(False)
            self.last_toggle_time = now
        elif (not self.ready_state) and elapsed >= self.ready_off_sec:
            self.get_logger().info(
                f'=== READY EDGE: false -> true  ('
                f'{elapsed:.2f}s in arm mode) ===')
            self.ready_state = True
            self.pub_ready(True)
            self.last_toggle_time = now

    def pub_ready(self, state: bool):
        msg = Bool()
        msg.data = state
        self.ready_pub.publish(msg)
        self.get_logger().info(f'Published ready={state} on '
                               f'/{self.side}_arm/ready')

    # ---------- pose streaming ----------
    def publish_target(self):
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
                    f'Initial pose captured for {self.side}.')
            except Exception as e:
                self.get_logger().warn(
                    f'Waiting for TF [{self.ee_frame}]: {e}',
                    throttle_duration_sec=2.0)
                return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        elapsed = time.time() - self.start_time

        transition_time = 5.0
        if elapsed < transition_time:
            alpha = elapsed / transition_time
            x = self.start_pose['x'] + (self.square_waypoints[0][0] - self.start_pose['x']) * alpha
            y = self.start_pose['y'] + (self.square_waypoints[0][1] - self.start_pose['y']) * alpha
            z = self.start_pose['z'] + (self.square_waypoints[0][2] - self.start_pose['z']) * alpha
        else:
            loop_elapsed = elapsed - transition_time
            total = len(self.square_waypoints) * self.time_per_segment
            cur = loop_elapsed % total
            seg_idx = int(cur // self.time_per_segment)
            seg_p = (cur % self.time_per_segment) / self.time_per_segment
            p1 = self.square_waypoints[seg_idx]
            p2 = self.square_waypoints[(seg_idx + 1) % len(self.square_waypoints)]
            x = p1[0] + (p2[0] - p1[0]) * seg_p
            y = p1[1] + (p2[1] - p1[1]) * seg_p
            z = p1[2] + (p2[2] - p1[2]) * seg_p

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        noise_roll  = 0.05 * math.sin(elapsed * 2.1)
        noise_pitch = 0.05 * math.cos(elapsed * 1.5)
        noise_yaw   = 0.05 * math.sin(elapsed * 3.3)
        q_noise = euler_to_quaternion(noise_roll, noise_pitch, noise_yaw)
        orig = self.start_pose['orientation']
        q_new = quaternion_multiply([orig.x, orig.y, orig.z, orig.w], q_noise)
        msg.pose.orientation.x = q_new[0]
        msg.pose.orientation.y = q_new[1]
        msg.pose.orientation.z = q_new[2]
        msg.pose.orientation.w = q_new[3]

        self.pose_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.target_frame_id
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # Always leave the wrapper in a known state on exit
        try:
            self.pub_ready(False)
            time.sleep(0.2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    side = 'left'
    ready_on_sec = 8.0
    ready_off_sec = 5.0
    start_ready = False
    auto_cycle = True

    # Simple CLI: side [ready_on_sec ready_off_sec start_ready auto_cycle]
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(argv) > 0 and argv[0].lower() in ('left', 'right'):
        side = argv[0].lower()
    if len(argv) > 1:
        ready_on_sec = float(argv[1])
    if len(argv) > 2:
        ready_off_sec = float(argv[2])
    if len(argv) > 3:
        start_ready = argv[3].lower() in ('1', 'true', 'yes')
    if len(argv) > 4:
        auto_cycle = argv[4].lower() in ('1', 'true', 'yes')

    node = PoseTrackingTester(side, ready_on_sec, ready_off_sec,
                              start_ready, auto_cycle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.pub_ready(False)
            time.sleep(0.2)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()