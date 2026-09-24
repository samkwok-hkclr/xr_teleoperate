import os
import asyncio
import numpy as np
from pathlib import Path
from threading import Event
from multiprocessing import Value, Array, Queue, Process, shared_memory

from vuer import Vuer, VuerSession
from vuer.events import HapticActuatorPulse
from vuer.schemas import ImageBackground, Hands, MotionControllers, WebRTCVideoPlane, WebRTCStereoVideoPlane

class VuerWrapper:
    def __init__(self):
        self.head_pose_shared = Array('d', 16, lock=True)
        self.left_arm_pose_shared = Array('d', 16, lock=True)
        self.right_arm_pose_shared = Array('d', 16, lock=True)
        self.motion_data_ready_shared = Value('b', False, lock=True)

        self.left_ctrl_trigger_shared = Value('b', False, lock=True)
        self.left_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
        self.left_ctrl_squeeze_shared = Value('b', False, lock=True)
        self.left_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
        self.left_ctrl_thumbstick_shared = Value('b', False, lock=True)
        self.left_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
        self.left_ctrl_aButton_shared = Value('b', False, lock=True)
        self.left_ctrl_bButton_shared = Value('b', False, lock=True)
        self.left_warn_shared = Value('b', False, lock=True)

        self.right_ctrl_trigger_shared = Value('b', False, lock=True)
        self.right_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
        self.right_ctrl_squeeze_shared = Value('b', False, lock=True)
        self.right_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
        self.right_ctrl_thumbstick_shared = Value('b', False, lock=True)
        self.right_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
        self.right_ctrl_aButton_shared = Value('b', False, lock=True)
        self.right_ctrl_bButton_shared = Value('b', False, lock=True)
        self.right_warn_shared = Value('b', False, lock=True)

        self.client_connected_shared = Value('b', False, lock=True)

        self.display_fps = 60

        cert_file = "/root/xr_teleop/cert/cert.pem"
        key_file = "/root/xr_teleop/cert/key.pem"

        # env_cert = os.getenv("XR_TELEOP_CERT")
        # env_key = os.getenv("XR_TELEOP_KEY")
        # if cert_file is None or key_file is None:
        #     # 1.Try environment variables
        #     if env_cert and env_key:
        #         cert_file = cert_file or env_cert
        #         key_file = key_file or env_key
        #     else:
        #         # 2.Try ~/.config/xr_teleoperate/
        #         user_conf_dir = Path.home() / ".config" / "xr_teleoperate"
        #         cert_path_user = user_conf_dir / "cert.pem"
        #         key_path_user = user_conf_dir / "key.pem"

        #         if cert_path_user.exists() and key_path_user.exists():
        #             cert_file = cert_file or str(cert_path_user)
        #             key_file = key_file or str(key_path_user)
        #         else:
        #             # 3.Fallback to package root (current logic)
        #             current_module_dir = Path(__file__).resolve().parent.parent.parent
        #             cert_file = cert_file or str(current_module_dir / "cert.pem")
        #             key_file = key_file or str(current_module_dir / "key.pem")

        self.vuer = Vuer(host="0.0.0.0", cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=3)
        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)

        self.display_mode = "pass-through"
        
        if self.display_mode == "pass-through":
            fn = self.main_pass_through
        else:
            raise ValueError(f"[TeleVuer] Unknown display_mode: {self.display_mode}")
        
        self.vuer.spawn(start=False)(fn)
        self.stop_writer_event = Event()

        self.process = Process(target=self._vuer_run)
        self.process.daemon = True
        self.process.start()

    def _vuer_run(self):
        try:
            self.vuer.run()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Vuer encountered an error: {e}")
        finally:
            if hasattr(self, "stop_writer_event"):
                self.stop_writer_event.set()

    # def _xr_render_loop(self):
    #     while not self.stop_writer_event.is_set():
    #         pass
            # if not self.new_frame_event.wait(timeout=0.1):
            #     continue
            # self.new_frame_event.clear()
            # if self.latest_frame is None:
            #     continue
            # latest_frame = self.latest_frame
            # latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
            # self.img2display[:] = latest_frame

    def close(self):
        if hasattr(self, 'process') and self.process.is_alive():
            print("Requesting Vuer process termination...")
            self.process.terminate()
            
            # Wait 1 second for graceful shutdown
            self.process.join(timeout=1.0)
            
            # If it is STILL alive, it is hanging. Force kill it.
            if self.process.is_alive():
                print("Vuer process hung. Force killing...")
                self.process.kill() # SIGKILL cannot be ignored
                self.process.join()
                
        print("Vuer process completely ended.")
        # if self.display_mode in ("immersive", "ego") and not self.webrtc:
        #     self.stop_writer_event.set()
        #     self.new_frame_event.set()
        #     self.writer_thread.join(timeout=0.5)
        #     try:
        #         self.img2display_shm.close()
        #         self.img2display_shm.unlink()
        #     except:
        #         pass

    async def on_cam_move(self, event, session, fps=30):
        try:
            with self.head_pose_shared.get_lock():
                self.head_pose_shared[:] = event.value["camera"]["matrix"]
        except:
            pass
        
    async def on_controller_move(self, event, session, fps=30):
        try:
            # ControllerData
            with self.left_arm_pose_shared.get_lock():
                self.left_arm_pose_shared[:] = event.value["left"]
            with self.right_arm_pose_shared.get_lock():
                self.right_arm_pose_shared[:] = event.value["right"]
            # ControllerState
            left_controller = event.value["leftState"]
            right_controller = event.value["rightState"]

            def extract_controllers(controllerState, prefix):
                # trigger
                with getattr(self, f"{prefix}_ctrl_trigger_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_trigger_shared").value = bool(controllerState.get("trigger", False))
                with getattr(self, f"{prefix}_ctrl_triggerValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_triggerValue_shared").value = float(controllerState.get("triggerValue", 0.0))
                # squeeze
                with getattr(self, f"{prefix}_ctrl_squeeze_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeeze_shared").value = bool(controllerState.get("squeeze", False))
                with getattr(self, f"{prefix}_ctrl_squeezeValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeezeValue_shared").value = float(controllerState.get("squeezeValue", 0.0))
                # thumbstick
                with getattr(self, f"{prefix}_ctrl_thumbstick_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstick_shared").value = bool(controllerState.get("thumbstick", False))
                with getattr(self, f"{prefix}_ctrl_thumbstickValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstickValue_shared")[:] = controllerState.get("thumbstickValue", [0.0, 0.0])
                # buttons
                with getattr(self, f"{prefix}_ctrl_aButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_aButton_shared").value = bool(controllerState.get("aButton", False))
                with getattr(self, f"{prefix}_ctrl_bButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_bButton_shared").value = bool(controllerState.get("bButton", False))

            extract_controllers(left_controller, "left")
            extract_controllers(right_controller, "right")
            if self.left_warn:
                await session.send(
                    HapticActuatorPulse(
                        left={"strength": 0.1, "duration": 10},
                    )
                )
            if self.right_warn:
                await session.send(
                    HapticActuatorPulse(
                        right={"strength": 0.1, "duration": 10},
                    )
                )
            with self.motion_data_ready_shared.get_lock():
                self.motion_data_ready_shared.value = True
        except:
            pass

    async def main_pass_through(self, session: VuerSession):
        session.upsert(
            MotionControllers(
                stream=True, 
                key="motionControllers",
                left=True,
                right=True,
            ),
            to="bgChildren",
        )

        with self.client_connected_shared.get_lock():
            self.client_connected_shared.value = True

        try:
            while True:
                await asyncio.sleep(1.0 / self.display_fps)
        except asyncio.CancelledError:
            pass
        finally:
            with self.client_connected_shared.get_lock():
                self.client_connected_shared.value = False
            
            self._reset_motion_data()
            print("Client disconnected. Reset motion data and connection flag.")
        
    def _reset_motion_data(self):
        """Resets all poses, button states, and motion ready flags to default values."""
        identity_flat = np.eye(4).flatten(order="F")
        
        with self.head_pose_shared.get_lock():
            self.head_pose_shared[:] = identity_flat
        with self.left_arm_pose_shared.get_lock():
            self.left_arm_pose_shared[:] = identity_flat
        with self.right_arm_pose_shared.get_lock():
            self.right_arm_pose_shared[:] = identity_flat
        with self.motion_data_ready_shared.get_lock():
            self.motion_data_ready_shared.value = False

        # Reset controller values for both sides
        for prefix in ["left", "right"]:
            with getattr(self, f"{prefix}_ctrl_trigger_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_trigger_shared").value = False
            with getattr(self, f"{prefix}_ctrl_triggerValue_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_triggerValue_shared").value = 0.0
            with getattr(self, f"{prefix}_ctrl_squeeze_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_squeeze_shared").value = False
            with getattr(self, f"{prefix}_ctrl_squeezeValue_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_squeezeValue_shared").value = 0.0
            with getattr(self, f"{prefix}_ctrl_thumbstick_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_thumbstick_shared").value = False
            with getattr(self, f"{prefix}_ctrl_thumbstickValue_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_thumbstickValue_shared")[:] = [0.0, 0.0]
            with getattr(self, f"{prefix}_ctrl_aButton_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_aButton_shared").value = False
            with getattr(self, f"{prefix}_ctrl_bButton_shared").get_lock():
                getattr(self, f"{prefix}_ctrl_bButton_shared").value = False

    # ==================== common data ====================
    @property
    def head_pose(self):
        """np.ndarray, shape (4, 4), head SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.head_pose_shared.get_lock():
            return np.array(self.head_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def left_arm_pose(self):
        """np.ndarray, shape (4, 4), left arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.left_arm_pose_shared.get_lock():
            return np.array(self.left_arm_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def right_arm_pose(self):
        """np.ndarray, shape (4, 4), right arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.right_arm_pose_shared.get_lock():
            return np.array(self.right_arm_pose_shared[:]).reshape(4, 4, order="F")

    # ==================== Controller Data ====================
    @property
    def left_ctrl_trigger(self):
        """bool, left controller trigger pressed or not."""
        with self.left_ctrl_trigger_shared.get_lock():
            return self.left_ctrl_trigger_shared.value

    @property
    def left_ctrl_triggerValue(self):
        """float, left controller trigger analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_triggerValue_shared.get_lock():
            return self.left_ctrl_triggerValue_shared.value

    @property
    def left_ctrl_squeeze(self):
        """bool, left controller squeeze pressed or not."""
        with self.left_ctrl_squeeze_shared.get_lock():
            return self.left_ctrl_squeeze_shared.value

    @property
    def left_ctrl_squeezeValue(self):
        """float, left controller squeeze analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_squeezeValue_shared.get_lock():
            return self.left_ctrl_squeezeValue_shared.value

    @property
    def left_ctrl_thumbstick(self):
        """bool, whether left thumbstick is touched or clicked."""
        with self.left_ctrl_thumbstick_shared.get_lock():
            return self.left_ctrl_thumbstick_shared.value

    @property
    def left_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), left thumbstick 2D axis values (x, y)."""
        with self.left_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.left_ctrl_thumbstickValue_shared[:])

    @property
    def left_ctrl_aButton(self):
        """bool, left controller 'A' button pressed."""
        with self.left_ctrl_aButton_shared.get_lock():
            return self.left_ctrl_aButton_shared.value

    @property
    def left_ctrl_bButton(self):
        """bool, left controller 'B' button pressed."""
        with self.left_ctrl_bButton_shared.get_lock():
            return self.left_ctrl_bButton_shared.value

    @property
    def right_ctrl_trigger(self):
        """bool, right controller trigger pressed or not."""
        with self.right_ctrl_trigger_shared.get_lock():
            return self.right_ctrl_trigger_shared.value

    @property
    def right_ctrl_triggerValue(self):
        """float, right controller trigger analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_triggerValue_shared.get_lock():
            return self.right_ctrl_triggerValue_shared.value

    @property
    def right_ctrl_squeeze(self):
        """bool, right controller squeeze pressed or not."""
        with self.right_ctrl_squeeze_shared.get_lock():
            return self.right_ctrl_squeeze_shared.value

    @property
    def right_ctrl_squeezeValue(self):
        """float, right controller squeeze analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_squeezeValue_shared.get_lock():
            return self.right_ctrl_squeezeValue_shared.value

    @property
    def right_ctrl_thumbstick(self):
        """bool, whether right thumbstick is touched or clicked."""
        with self.right_ctrl_thumbstick_shared.get_lock():
            return self.right_ctrl_thumbstick_shared.value

    @property
    def right_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), right thumbstick 2D axis values (x, y)."""
        with self.right_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.right_ctrl_thumbstickValue_shared[:])

    @property
    def right_ctrl_aButton(self):
        """bool, right controller 'A' button pressed."""
        with self.right_ctrl_aButton_shared.get_lock():
            return self.right_ctrl_aButton_shared.value

    @property
    def right_ctrl_bButton(self):
        """bool, right controller 'B' button pressed."""
        with self.right_ctrl_bButton_shared.get_lock():
            return self.right_ctrl_bButton_shared.value

    @property
    def motion_data_ready(self):
        """bool, whether at least one hand or controller motion data event has been received."""
        with self.motion_data_ready_shared.get_lock():
            return self.motion_data_ready_shared.value

    @property
    def left_warn(self):
        """bool."""
        with self.left_warn_shared.get_lock():
            return self.left_warn_shared.value
        
    @property
    def right_warn(self):
        """bool."""
        with self.right_warn_shared.get_lock():
            return self.right_warn_shared.value
            
    @property
    def client_connected(self):
        """bool, whether at least one client is connected."""
        with self.client_connected_shared.get_lock():
            return self.client_connected_shared.value

