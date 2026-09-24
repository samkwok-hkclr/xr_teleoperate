import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
import time
import math
import random
import sys

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to Quaternion"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_multiply(q1, q2):
    """Multiply two quaternions"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

class InfiniteSquarePublisher(Node):
    def __init__(self, side='left'):
        super().__init__('infinite_square_publisher')
        
        self.side = side.lower()
        
        # Dynamically set topic, frame, and waypoints based on argument
        if self.side == 'right':
            topic = '/right_cartesian_motion_controller/target_frame'
            ready_topic = '/right_arm/ready'
            self.ee_frame = 'tcp_r'
            self.target_frame_id = 'right_target_frame'
            self.square_waypoints = [
                (0.6, -0.25, 1.0),
                (0.5, -0.35, 1.0),
                (0.5, -0.35, 1.3),
                (0.6, -0.25, 1.3)
            ]
        else:
            topic = '/left_cartesian_motion_controller/target_frame'
            ready_topic = '/left_arm/ready'
            self.ee_frame = 'tcp_l'
            self.target_frame_id = 'left_target_frame'
            self.square_waypoints = [
                (0.7, 0.25, 1.0),
                (0.5, 0.35, 1.0),
                (0.5, 0.35, 1.3),
                (0.7, 0.25, 1.3),
                # (0.3, -0.06, 0.38),
                # (0.3, 0.06, 0.42),
                # (0.3, 0.06, 0.45),
                # (0.3, -0.06, 0.38),
            ]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos_profile)
        self.pub = self.create_publisher(PoseStamped, topic, qos_profile)
        
        # TF Buffer, Listener, and Broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.start_pose = None
        self.start_time = None
        self.time_per_segment = 5.0  # Seconds per side of the square
        
        # 250 Hz timer
        self.timer = self.create_timer(0.02, self.publish_target)
        self.get_logger().info(f'Initialized for [{self.side.upper()}] arm on topic: {topic} (broadcasting TF: {self.target_frame_id})')

    def publish_target(self):
        if self.start_pose is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    self.ee_frame,
                    rclpy.time.Time()
                )
                self.start_pose = {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z,
                    'orientation': transform.transform.rotation
                }
                self.start_time = time.time()
                self.get_logger().info(f'Initial pose captured for {self.side}. Starting infinite square loop...')
            except Exception as e:
                self.get_logger().warn(f'Waiting for TF [{self.ee_frame}]: {e}', throttle_duration_sec=2.0)
                return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        elapsed = time.time() - self.start_time

        # Smoothly glide from current position to the first corner over 5 seconds
        transition_time = 5.0
        if elapsed < transition_time:
            alpha = elapsed / transition_time
            x = self.start_pose['x'] + (self.square_waypoints[0][0] - self.start_pose['x']) * alpha
            y = self.start_pose['y'] + (self.square_waypoints[0][1] - self.start_pose['y']) * alpha
            z = self.start_pose['z'] + (self.square_waypoints[0][2] - self.start_pose['z']) * alpha
        else:
            # Infinitely loop through the 4 corners using modulo arithmetic
            loop_elapsed = elapsed - transition_time
            total_cycle_time = len(self.square_waypoints) * self.time_per_segment
            current_cycle_time = loop_elapsed % total_cycle_time
            
            seg_idx = int(current_cycle_time // self.time_per_segment)
            seg_progress = (current_cycle_time % self.time_per_segment) / self.time_per_segment
            
            p1 = self.square_waypoints[seg_idx]
            p2 = self.square_waypoints[(seg_idx + 1) % len(self.square_waypoints)]
            
            x = p1[0] + (p2[0] - p1[0]) * seg_progress
            y = p1[1] + (p2[1] - p1[1]) * seg_progress
            z = p1[2] + (p2[2] - p1[2]) * seg_progress
            
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # --- ORIENTATION NOISE ---
        noise_roll = 0.05 * math.sin(elapsed * 2.1)
        noise_pitch = 0.05 * math.cos(elapsed * 1.5)
        noise_yaw = 0.05 * math.sin(elapsed * 3.3)

        q_noise = euler_to_quaternion(noise_roll, noise_pitch, noise_yaw)
        orig = self.start_pose['orientation']
        q_orig = [orig.x, orig.y, orig.z, orig.w]
        q_new = quaternion_multiply(q_orig, q_noise)

        msg.pose.orientation.x = q_new[0]
        msg.pose.orientation.y = q_new[1]
        msg.pose.orientation.z = q_new[2]
        msg.pose.orientation.w = q_new[3]
        
        # Publish the target pose message
        self.pub.publish(msg)

        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)

        # Broadcast the calculated target pose to the TF tree
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.target_frame_id
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def pub_ready(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self.ready_pub.publish(msg)
        self.get_logger().warn(f"Published ready: {state}")

    def destroy_node(self):
        self.pub_ready(False)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    side = 'left'
    if len(sys.argv) > 1:
        arg_val = sys.argv[1].lower()
        if arg_val in ['left', 'right']:
            side = arg_val

    node = InfiniteSquarePublisher(side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.pub_ready(False)
            time.sleep(0.2)  # give DDS a moment to deliver
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()