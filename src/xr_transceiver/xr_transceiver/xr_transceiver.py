
import sys
import numpy as np
from functools import partial

from enum import Enum
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from xr_teleop_msgs.msg import Button, ButtonStrength, ButtonThumbstick

from .vuer_wrapper import VuerWrapper

T_ROBOT_OPENXR = np.array([[ 0, 0,-1, 0],
                           [-1, 0, 0, 0],
                           [ 0, 1, 0, 0],
                           [ 0, 0, 0, 1]])

T_OPENXR_ROBOT = np.array([[ 0,-1, 0, 0],
                           [ 0, 0, 1, 0],
                           [-1, 0, 0, 0],
                           [ 0, 0, 0, 1]])

CONST_HEAD_POSE = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 1.5],
                            [0, 0, 1, -0.2],
                            [0, 0, 0, 1]])

CONST_RIGHT_ARM_POSE = np.array([[1, 0, 0, 0.15],
                                 [0, 1, 0, 1.13],
                                 [0, 0, 1, -0.3],
                                 [0, 0, 0, 1]])

CONST_LEFT_ARM_POSE = np.array([[1, 0, 0, -0.15],
                                [0, 1, 0, 1.13],
                                [0, 0, 1, -0.3],
                                [0, 0, 0, 1]])

def safe_mat_update(prev_mat, mat):
    # Return previous matrix and False flag if the new matrix is non-singular (determinant ≠ 0).
    det = np.linalg.det(mat)
    if not np.isfinite(det) or np.isclose(det, 0.0, atol=1e-6):
        return prev_mat, False
    return mat, True

def get_Brobot_world_head_yaw_rot(Brobot_world_head_rot):
    # Extract R_Brobot_world_head_yaw from Brobot_world_head_rot under (basis) Robot Convention.
    # Brobot_world_head_x_axis = Brobot_world_head_rot[:, 0].copy()
    Brobot_world_head_x_axis = Brobot_world_head_rot[:, 0].astype(float).copy()
    Brobot_world_head_x_axis[2] = 0.0
    x_norm = np.linalg.norm(Brobot_world_head_x_axis)
    if not np.isfinite(x_norm) or np.isclose(x_norm, 0.0, atol=1e-6):
        return np.eye(3)

    Brobot_world_head_x_axis /= x_norm
    Brobot_world_head_z_axis = np.array([0.0, 0.0, 1.0])
    Brobot_world_head_y_axis = np.cross(Brobot_world_head_z_axis, Brobot_world_head_x_axis)
    Brobot_world_head_y_axis /= np.linalg.norm(Brobot_world_head_y_axis)
    return np.column_stack([Brobot_world_head_x_axis, Brobot_world_head_y_axis, Brobot_world_head_z_axis])

def transform_Brobot_world_arm_to_head_then_waist(Brobot_world_arm, Brobot_world_head, arm_reference_mode):
    Brobot_head_arm = Brobot_world_arm.copy()
    # Transfer from WORLD to HEAD coordinate:
    # - "head_position": translation adjustment only.
    # - "head_yaw": left-multiply R_Brobot_world_head_yaw^T, ignoring pitch/roll.
    if arm_reference_mode == "head_yaw":
        R_Brobot_world_head_yaw = get_Brobot_world_head_yaw_rot(Brobot_world_head[:3, :3])
        Brobot_head_arm[:3, :3] = R_Brobot_world_head_yaw.T @ Brobot_world_arm[:3, :3]
        Brobot_head_arm[:3, 3] = R_Brobot_world_head_yaw.T @ (Brobot_world_arm[:3, 3] - Brobot_world_head[:3, 3])
    else:
        Brobot_head_arm[:3, 3] = Brobot_world_arm[:3, 3] - Brobot_world_head[:3, 3]

    # Translate the origin of Brobot_head_arm from HEAD to WAIST.
    Brobot_waist_arm = Brobot_head_arm.copy()
    Brobot_waist_arm[0, 3] += 0.15
    Brobot_waist_arm[2, 3] += 0.45
    return Brobot_waist_arm

class ARM(Enum):
    LEFT = 1
    RIGHT = 2

class XRTransceiver(LifecycleNode): # Inherit from LifecycleNode
    def __init__(self):
        super().__init__('xr_transceiver')
        self.declare_parameter('pose_frequency', 50.0)
        self.declare_parameter('button_frequency', 20.0)

        # Initialize properties to None/Empty so they exist
        self.head_pose_pub_ = None
        self.pose_pub_ = dict()
        self.ctrl_trigger_pub_ = dict()
        self.ctrl_squeeze_pub_ = dict()
        self.ctrl_thumbstick_pub_ = dict()
        self.ctrl_a_btn_pub_ = dict()
        self.ctrl_b_btn_pub_ = dict()

        self.warn_sub_ = dict()
        
        self.pose_timer_ = None
        self.button_timer_ = None
        self.vuer_wrapper = None

        self.arm_reference_mode = "head_position" # "head_yaw" "head_position"
        
        self.get_logger().info("Lifecycle Node Created. Waiting for 'configure' transition.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring...")
        
        self.head_pose_pub_ = self.create_lifecycle_publisher(PoseStamped, '/xr/raw_head_pose', qos_profile=qos_profile_sensor_data)
        
        for arm in ARM:
            side = arm.name.lower()
            self.pose_pub_[arm] = self.create_lifecycle_publisher(PoseStamped, f'/xr/raw_{side}_arm', qos_profile=qos_profile_sensor_data)
            self.ctrl_trigger_pub_[arm] = self.create_lifecycle_publisher(ButtonStrength, f'/xr/{side}_ctrl_trigger', qos_profile=qos_profile_sensor_data)
            self.ctrl_squeeze_pub_[arm] = self.create_lifecycle_publisher(ButtonStrength, f'/xr/{side}_ctrl_squeeze', qos_profile=qos_profile_sensor_data)
            self.ctrl_thumbstick_pub_[arm] = self.create_lifecycle_publisher(ButtonThumbstick, f'/xr/{side}_ctrl_thumbstick', qos_profile=qos_profile_sensor_data)
            self.ctrl_a_btn_pub_[arm] = self.create_lifecycle_publisher(Button, f'/xr/{side}_a_button', qos_profile=qos_profile_sensor_data)
            self.ctrl_b_btn_pub_[arm] = self.create_lifecycle_publisher(Button, f'/xr/{side}_b_button', qos_profile=qos_profile_sensor_data)

            self.warn_sub_[arm] = self.create_subscription(Bool, f'/{side}_arm/warn', partial(self.warn_cb, arm=arm), 1)

        # Initialize the hardware/wrapper
        self.vuer_wrapper = VuerWrapper()

        self.get_logger().info("Configuration done...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Activating the base class activates all registered lifecycle publishers
        super().on_activate(state)
        
        # Start timers only when the node is active
        pose_frequency = self.get_parameter("pose_frequency").value
        btn_frequency = self.get_parameter("button_frequency").value
        self.pose_timer_ = self.create_timer(1.0 / pose_frequency, self.pose_timer_cb)
        self.button_timer_ = self.create_timer(1.0 / btn_frequency, self.btn_timer_cb)

        self.get_logger().info("Activation done...")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        # Deactivating the base class deactivates all lifecycle publishers
        super().on_deactivate(state)
        
        # Stop the timers so we stop processing data
        if self.pose_timer_:
            self.destroy_timer(self.pose_timer_)
            self.pose_timer_ = None
        if self.button_timer_:
            self.destroy_timer(self.button_timer_)
            self.button_timer_ = None

        self.get_logger().info("deactivation done...")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        
        # Destroy publishers and subscriptions
        self.destroy_publisher(self.head_pose_pub_)
        for arm in ARM:
            self.destroy_publisher(self.pose_pub_[arm])
            self.destroy_publisher(self.ctrl_trigger_pub_[arm])
            self.destroy_publisher(self.ctrl_squeeze_pub_[arm])
            self.destroy_publisher(self.ctrl_thumbstick_pub_[arm])
            self.destroy_publisher(self.ctrl_a_btn_pub_[arm])
            self.destroy_publisher(self.ctrl_b_btn_pub_[arm])
            
            self.destroy_subscription(self.warn_sub_[arm])
            
        self.pose_pub_.clear()
        self.ctrl_trigger_pub_.clear()
        self.ctrl_squeeze_pub_.clear()
        self.ctrl_thumbstick_pub_.clear()
        self.ctrl_a_btn_pub_.clear()
        self.ctrl_b_btn_pub_.clear()

        if self.vuer_wrapper:
            self.vuer_wrapper.close() 
            self.vuer_wrapper = None

        self.get_logger().info("Cleaning done...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        self.get_logger().info("Shutdown done...")
        return TransitionCallbackReturn.SUCCESS

    def is_active(self) -> bool:
        return self.current_state[1] == State.PRIMARY_STATE_ACTIVE

    def is_connected(self) -> bool:
        return self.vuer_wrapper.client_connected

    def is_data_ready(self) -> bool:
        return self.vuer_wrapper.motion_data_ready

    def is_ready_to_publish(self) -> bool:
        return self.is_connected() and self.is_data_ready()

    def warn_cb(self, msg: Bool, arm: ARM):
        if not self.is_active:
            return
        
        if arm == ARM.LEFT:
            with self.vuer_wrapper.left_warn_shared.get_lock():
                self.vuer_wrapper.left_warn_shared.value = msg.data
        elif arm == ARM.RIGHT:
            with self.vuer_wrapper.right_warn_shared.get_lock():
                self.vuer_wrapper.right_warn_shared.value = msg.data

        if msg and msg.data:
            self.get_logger().debug(f"{arm.name.lower()} warning!")

    def pose_timer_cb(self):
        if not self.is_ready_to_publish():
            return

        Bxr_world_head, _ = safe_mat_update(CONST_HEAD_POSE, self.vuer_wrapper.head_pose)

        left_Bxr_world_arm, _ = safe_mat_update(CONST_LEFT_ARM_POSE, self.vuer_wrapper.left_arm_pose)
        right_Bxr_world_arm, _ = safe_mat_update(CONST_RIGHT_ARM_POSE, self.vuer_wrapper.right_arm_pose)

        Brobot_world_head = T_ROBOT_OPENXR @ Bxr_world_head @ T_OPENXR_ROBOT
        left_Brobot_world_arm  = T_ROBOT_OPENXR @ left_Bxr_world_arm @ T_OPENXR_ROBOT
        right_Brobot_world_arm = T_ROBOT_OPENXR @ right_Bxr_world_arm @ T_OPENXR_ROBOT

        left_Brobot_waist_arm = transform_Brobot_world_arm_to_head_then_waist(left_Brobot_world_arm, Brobot_world_head, self.arm_reference_mode)
        right_Brobot_waist_arm = transform_Brobot_world_arm_to_head_then_waist(right_Brobot_world_arm, Brobot_world_head, self.arm_reference_mode)

        self.get_logger().debug(f"head: {Brobot_world_head}", throttle_duration_sec=1.0)
        self.get_logger().debug(f"left: {left_Brobot_waist_arm}", throttle_duration_sec=1.0)
        self.get_logger().debug(f"right: {right_Brobot_waist_arm}", throttle_duration_sec=1.0)

        head_pose = self._get_pose(Brobot_world_head , "raw_head_frame")
        left_pose = self._get_pose(left_Brobot_waist_arm, "raw_head_frame")
        right_pose = self._get_pose(right_Brobot_waist_arm, "raw_head_frame")
        
        self.head_pose_pub_.publish(head_pose)
        self.pose_pub_[ARM.LEFT].publish(left_pose)
        self.pose_pub_[ARM.RIGHT].publish(right_pose)

    def btn_timer_cb(self):
        if not self.is_ready_to_publish():
            return

        now = self.get_clock().now().to_msg()
        wrapper = self.vuer_wrapper

        for arm in ARM:
            side = arm.name.lower()
            # Dynamically fetch wrapper properties using getattr
            self.ctrl_trigger_pub_[arm].publish(
                self._get_btn_str(getattr(
                    wrapper, f"{side}_ctrl_trigger"), 
                    getattr(wrapper, f"{side}_ctrl_triggerValue"),
                    now
                )
            )
            self.ctrl_squeeze_pub_[arm].publish(
                self._get_btn_str(getattr(
                    wrapper, f"{side}_ctrl_squeeze"),
                    getattr(wrapper, f"{side}_ctrl_squeezeValue"),
                    now
                )
            )
            self.ctrl_thumbstick_pub_[arm].publish(
                self._get_btn_thumbstick(
                    getattr(wrapper, f"{side}_ctrl_thumbstick"), 
                    getattr(wrapper, f"{side}_ctrl_thumbstickValue"),
                    now
                )
            )
            self.ctrl_a_btn_pub_[arm].publish(
                self._get_btn(
                    getattr(wrapper, f"{side}_ctrl_aButton"),
                    now
                )
            )
            self.ctrl_b_btn_pub_[arm].publish(
                self._get_btn(
                    getattr(wrapper, f"{side}_ctrl_bButton"),
                    now
                )
            )

    def _get_pose(self, pose_matrix: np.ndarray, frame_id: str) -> PoseStamped:
        mat = np.array(pose_matrix).reshape(4, 4)
        x, y, z = float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])
        
        rot_mat = mat[0:3, 0:3]
        quat = R.from_matrix(rot_mat).as_quat()
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

    def _get_btn(self, pressed, stamp) -> Button:
        msg = Button()
        msg.header.stamp = stamp
        msg.pressed = bool(pressed)
        return msg
    
    def _get_btn_str(self, pressed, strength: float, stamp) -> ButtonStrength:
        msg = ButtonStrength()
        msg.header.stamp = stamp
        msg.pressed = bool(pressed)
        msg.strength = strength
        return msg

    def _get_btn_thumbstick(self, pressed, thumbstick: np.ndarray, stamp) -> ButtonThumbstick:
        msg = ButtonThumbstick()
        msg.header.stamp = stamp
        msg.pressed = bool(pressed)
        if len(thumbstick) < 2:
            self.get_logger().error("Thumbstick array length is less than 2")
            return msg
        msg.x = thumbstick[0]
        msg.y = thumbstick[1]
        return msg

def main(args=None):
    rclpy.init(args=args)
    try:
        node = XRTransceiver()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

