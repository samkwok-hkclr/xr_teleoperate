import os
import re
import shutil
import tempfile
import time
import asyncio
import numpy as np
from scipy.spatial.transform import Rotation as R

from pathlib import Path
from threading import Event
from multiprocessing import Value, Array, Queue, Process, shared_memory

from vuer import Vuer, VuerSession
from vuer.events import HapticActuatorPulse
from vuer.schemas import (
    SceneCamera, Grid, MotionControllers, Urdf,
    OrbitControls, CoordsMarker, group, DefaultScene,
)
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

def in_container() -> bool:
    # explicit override wins
    env = os.getenv("XR_TELEOP_ENV", "").lower()
    if env in ("host", "container"):
        return env == "container"

    if Path("/.dockerenv").exists():
        return True
    try:
        if any(m in Path("/proc/1/cgroup").read_text()
               for m in ("docker", "kubepods", "containerd", "lxc", "podman")):
            return True
    except OSError:
        pass
    try:
        if Path("/proc/1/comm").read_text().strip() not in ("systemd", "init"):
            return True
    except OSError:
        pass
    return False

def resolve_cert_paths():
    # 1. explicit env override (XR_TELEOP_CERT / XR_TELEOP_KEY)
    env_cert = os.getenv("XR_TELEOP_CERT")
    env_key  = os.getenv("XR_TELEOP_KEY")
    if env_cert and env_key:
        return Path(env_cert), Path(env_key)

    home = os.getenv("HOME")
    ws_name = os.getenv("WS_NAME")

    cert_file: Path | None = None
    key_file: Path | None = None

    # 2. known layout per environment
    if in_container():
        base = Path(home) / ws_name / "cert" 
    else:
        base = Path(home) / ws_name / "cert"

    cert_file = base / "cert.pem"
    key_file  = base / "key.pem"

    # 3. user config dir fallback
    if not (cert_file.exists() and key_file.exists()):
        user_dir = Path.home() / ".config" / "xr_teleoperate"
        cert_file = user_dir / "cert.pem"
        key_file  = user_dir / "key.pem"

    # 4. package root fallback
    if not (cert_file.exists() and key_file.exists()):
        pkg = Path(__file__).resolve().parent.parent.parent
        cert_file = pkg / "cert.pem"
        key_file  = pkg / "key.pem"

    return cert_file, key_file

class VuerWrapper:
    def __init__(self, server_ip: str, server_port: int,
             robot_description: str | None = None,
             joint_names_shared=None,
             joint_positions_shared=None,
             joint_count_shared=None,
             joint_state_updated=None,
             tf_frames=None,
             tf_poses_shared=None,
             tf_poses_updated=None,
             tf_last_seen=None):
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

        self.joint_names_shared = joint_names_shared
        self.joint_positions_shared = joint_positions_shared
        self.joint_count_shared = joint_count_shared
        self.joint_state_updated = joint_state_updated
        self._cached_names = None

        self.tf_frames = tf_frames or []
        self.tf_poses_shared = tf_poses_shared or {}
        self.tf_poses_updated = tf_poses_updated
        self.tf_last_seen = tf_last_seen

        self.display_fps = 60

        self.robot_description = robot_description
        self._static_dir = Path(tempfile.mkdtemp(prefix="vuer_urdf_"))
        self._urdf_url = None
        if robot_description is not None:
            # 1. Find every package://<pkg>/ reference in the URDF
            referenced_pkgs = set(re.findall(r"package://([^/]+)/", robot_description))

            # 2. Copy each package's share directory into the workspace,
            #    preserving the package name as a subdirectory so paths don't collide.
            for pkg in referenced_pkgs:
                try:
                    pkg_share = Path(get_package_share_directory(pkg))
                except PackageNotFoundError:
                    print(f"[VuerWrapper] WARNING: ROS package not found: {pkg}")
                    continue

                dest = self._static_dir / pkg
                if dest.exists():
                    continue
                shutil.copytree(
                    pkg_share,
                    dest,
                    dirs_exist_ok=True,
                    # skip the bits the browser will never ask for
                    ignore=shutil.ignore_patterns("package.xml", "*.cmake", "*.py", "*.xacro"),
                )
                print(f"[VuerWrapper] Copied {pkg} share -> {dest}")

            # 3. Rewrite: package://dual_arm_robot/meshes/base/DP.STL -> dual_arm_robot/meshes/base/DP.STL
            urdf_text = re.sub(r"package://([^/]+)/", r"\1/", robot_description)

            (self._static_dir / "dual_arm_robot.urdf").write_text(urdf_text)
            self._urdf_url = f"https://{server_ip}:{server_port}/workspace/dual_arm_robot.urdf"
            print(f"URDF URL: {self._urdf_url}")

        cert_file, key_file = resolve_cert_paths()
        if cert_file is None or key_file is None:
            raise ValueError(f"Empty cert")
        
        self.vuer = Vuer(
            host=server_ip, 
            port=server_port, 
            cert=cert_file, 
            key=key_file, 
            workspace=str(self._static_dir),
            queries=dict(grid=False), 
            queue_len=3
        )
        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)

        self.display_mode = "pass-through"
        
        if self.display_mode == "pass-through":
            fn = self.main_pass_through
        else:
            raise ValueError(f"[TeleVuer] Unknown display_mode: {self.display_mode}")
        
        self.vuer.spawn(start=False)(fn)
        self.stop_writer_event = Event()
        self.process = Process(target=self._vuer_run, args=(robot_description,),)
        self.process.daemon = True
        self.process.start()

    def _vuer_run(self, robot_description: str | None):
        self.robot_description = robot_description
        try:
            self.vuer.run()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Vuer encountered an error: {e}")
        finally:
            if hasattr(self, "stop_writer_event"):
                self.stop_writer_event.set()

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

        shutil.rmtree(self._static_dir, ignore_errors=True)

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
        scene_children = [
            MotionControllers(
                stream=True, 
                key="motion_controllers",
                left=True, 
                right=True
            ),
            Grid(
                key="grid", 
                rotation=[np.pi/2, 0, 0], 
                # infiniteGrid=False
            ),
        ]

        if self._urdf_url is not None:
            scene_children.append(Urdf(src=self._urdf_url, key="dual_arm_robot"))

        bg = [
            SceneCamera(
                key="my_camera",
                up=[0, 0, 1],
                lookAt=[-1.0, 0, 0],
                position=[2.0, 0.0, 2.5],
                far=15,
                makeDefault=True,
            ),
            OrbitControls(key="orbit_controls"),
        ]

        for frame in self.tf_frames:
            scene_children.append(CoordsMarker(position=[0,0,0], scale=0.15, key=frame))
        
        session.set(
            DefaultScene(
                *scene_children, 
                up=[0, 0, 1], 
                grid=False,
                bgChildren=bg,
                key="root",
            )
        )

        joints_task = None
        if self.joint_state_updated is not None:
            joints_task = asyncio.create_task(self._update_joints_loop(session))

        tf_task = None
        if self.tf_poses_updated is not None and self.tf_frames:
            tf_task = asyncio.create_task(self._update_tf_loop(session))

        with self.client_connected_shared.get_lock():
            self.client_connected_shared.value = True

        try:
            await session.forever()
        except asyncio.CancelledError:
            pass
        finally:
            for task in (joints_task, tf_task):
                if task is None:
                    continue
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

            with self.client_connected_shared.get_lock():
                self.client_connected_shared.value = False

            self._reset_motion_data()
            print("Client disconnected. Reset motion data and connection flag.")
            
    async def _update_joints_loop(self, session: VuerSession):
        last_sent: dict[str, float] = {}
        EPS = 1e-3
        need_full_send = True
        last_full_send_time = 0.0
        FULL_RESEND_INTERVAL = 1.0    # resend every joint every second
        first_send_logged = False

        try:
            while True:
                now = time.monotonic()
                joint_values = None

                with self.joint_state_updated.get_lock():
                    flag_set = self.joint_state_updated.value
                    if flag_set:
                        self.joint_state_updated.value = False

                # Force a full refresh on the very first successful read,
                # and again every FULL_RESEND_INTERVAL seconds thereafter.
                force_full = need_full_send or (now - last_full_send_time) > FULL_RESEND_INTERVAL

                if flag_set or force_full:
                    with self.joint_count_shared.get_lock():
                        n = self.joint_count_shared.value

                    if n > 0:
                        if self._cached_names is None or len(self._cached_names) != n:
                            self._cached_names = list(self.joint_names_shared)[:n]

                        with self.joint_positions_shared.get_lock():
                            positions = list(self.joint_positions_shared[:n])

                        joint_values = dict(zip(self._cached_names, positions))

                if joint_values:
                    if force_full:
                        changed = joint_values
                        need_full_send = False
                        last_full_send_time = now
                        if not first_send_logged:
                            print(f"[Vuer] joints: full send of {len(changed)} joints")
                            first_send_logged = True
                    else:
                        changed = {
                            k: v for k, v in joint_values.items()
                            if k not in last_sent or abs(v - last_sent[k]) > EPS
                        }

                    if changed:
                        session.update @ Urdf(
                            src=self._urdf_url,
                            jointValues=changed,
                            key="dual_arm_robot",
                        )
                        last_sent.update(changed)

                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            return
        
    async def _update_tf_loop(self, session: VuerSession):
        HIDE_AFTER_SEC = 1.0        # hide a marker after this many seconds of no data
        VISIBLE_SCALE = 0.15

        # Per-frame state we last pushed to the browser:
        #   True  -> marker is currently visible
        #   False -> marker is currently hidden (scale == 0)
        # Only transitions are sent, so no per-tick spam.
        last_state: dict[str, bool] = {}

        print(f"[Vuer] TF loop started, frames={self.tf_frames}, "
            f"auto-hide={self.tf_last_seen is not None}, "
            f"hide_after={HIDE_AFTER_SEC}s")

        try:
            while True:
                now = time.monotonic()

                with self.tf_poses_updated.get_lock():
                    if self.tf_poses_updated.value:
                        self.tf_poses_updated.value = False

                for frame in self.tf_frames:
                    mat = None

                    # --- Freshness check ---
                    if self.tf_last_seen is not None:
                        last_seen = self.tf_last_seen.get(frame, 0.0)
                        is_fresh = (now - last_seen) < HIDE_AFTER_SEC
                    else:
                        is_fresh = True   # auto-hide disabled

                    # --- Pose check (only if fresh) ---
                    if is_fresh:
                        arr = self.tf_poses_shared.get(frame)
                        if arr is not None:
                            with arr.get_lock():
                                m = np.array(arr[:]).reshape(4, 4, order="F")
                            # Reject never-written (all zeros) and non-rotation matrices
                            if np.any(m):
                                rot = m[:3, :3]
                                det = np.linalg.det(rot)
                                if np.isfinite(det) and abs(det) >= 1e-3:
                                    mat = m

                    # --- Push update ---
                    if mat is not None:
                        pos = mat[:3, 3].tolist()
                        rpy = R.from_matrix(mat[:3, :3]).as_euler("XYZ").tolist()
                        session.update @ CoordsMarker(
                            key=frame,
                            position=pos,
                            rotation=rpy,
                            scale=VISIBLE_SCALE,
                        )
                        last_state[frame] = True
                    else:
                        # Hide only on transition (visible -> hidden)
                        if last_state.get(frame) is not False:
                            session.update @ CoordsMarker(
                                key=frame,
                                position=[0.0, 0.0, 0.0],
                                rotation=[0.0, 0.0, 0.0],
                                scale=0.0,
                            )
                            last_state[frame] = False

                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            return
        except Exception as e:
            print(f"[Vuer] TF loop CRASHED: {type(e).__name__}: {e}")
            raise
        
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