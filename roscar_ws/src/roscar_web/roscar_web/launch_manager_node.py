"""Launch manager node: manages robot operation modes via subprocess.

Exposes ROS2 services that the web dashboard calls to start/stop
different robot configurations (teleop, SLAM mapping, navigation, etc.).

The web infrastructure (rosbridge + web_video_server + this node) stays
running permanently. Only the robot nodes are started/stopped per mode.
"""

import os
import signal
import subprocess
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Empty, Int8
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose

import glob as globmod

from roscar_interfaces.srv import SetMode, SaveMap, GetStatus, ListMaps


AVAILABLE_MODES = ['idle', 'teleop', 'slam', 'navigation', 'slam_nav']

# Map mode names to ros2 launch commands
MODE_COMMANDS = {
    'teleop':     ['ros2', 'launch', 'roscar_bringup', 'robot.launch.py'],
    'slam':       ['ros2', 'launch', 'roscar_bringup', 'slam.launch.py'],
    'navigation': ['ros2', 'launch', 'roscar_bringup', 'navigation.launch.py'],
    'slam_nav':   ['ros2', 'launch', 'roscar_bringup', 'slam_nav.launch.py'],
}


class LaunchManagerNode(Node):
    def __init__(self):
        super().__init__('launch_manager')

        self._current_mode = 'idle'
        self._process = None          # subprocess.Popen of active launch
        self._process_lock = threading.Lock()

        # Cancellation flag for the slam→nav carryover background thread.
        # set() any time the user changes mode so an in-flight carryover
        # publisher stops before publishing /initialpose into the wrong mode.
        self._carryover_cancel = threading.Event()
        self._carryover_cancel.set()  # idle: no carryover in flight

        # TF + initial-pose publisher used to carry the robot's pose across a
        # slam → navigation handoff. Without this, AMCL starts at the
        # nav2_params default (0,0,0) and the robot ends up localized in
        # the wrong place on the just-saved map.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self._set_mode_srv = self.create_service(
            SetMode, '/web/set_mode', self._set_mode_callback)
        self._save_map_srv = self.create_service(
            SaveMap, '/web/save_map', self._save_map_callback)
        self._get_status_srv = self.create_service(
            GetStatus, '/web/get_status', self._get_status_callback)
        self._list_maps_srv = self.create_service(
            ListMaps, '/web/list_maps', self._list_maps_callback)

        # Nav-goal shim. Bundled roslibjs (1.x) only knows ROS1-style
        # topic-based actionlib. ROS2 actions are service-based and the
        # dashboard can't reach them directly. Workaround: dashboard
        # publishes a PoseStamped to /web/nav_goal, this node forwards it
        # to /navigate_to_pose. /web/cancel_nav_goal cancels the active goal.
        self._nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_goal_lock = threading.Lock()
        self.create_subscription(
            PoseStamped, '/web/nav_goal', self._on_nav_goal, 10)
        self.create_subscription(
            Empty, '/web/cancel_nav_goal', self._on_cancel_nav_goal, 10)
        # Result feedback to the dashboard. Int8 with the action_msgs/GoalStatus
        # value (4=succeeded, 5=cancelled, 6=aborted, plus any negative codes
        # for our own errors).
        self._nav_result_pub = self.create_publisher(
            Int8, '/web/nav_goal_result', 10)

        self.get_logger().info('Launch manager ready. Mode: idle')

    # ------------------------------------------------------------------ #
    #  Service callbacks                                                   #
    # ------------------------------------------------------------------ #

    def _set_mode_callback(self, request, response):
        mode = request.mode.strip()
        self.get_logger().info(
            f'set_mode request received: mode={mode!r} '
            f'map_path={request.map_path!r} use_depth={request.use_depth}'
        )

        # Any mode change cancels an in-flight carryover publisher. The bg
        # thread checks this between sleeps and before each publish, so we
        # set it here unconditionally (before validation) to make sure even
        # rapid bounces (slam_nav→navigation→teleop) interrupt cleanly.
        self._carryover_cancel.set()

        if mode not in AVAILABLE_MODES:
            response.success = False
            response.message = f'Unknown mode "{mode}". Valid: {AVAILABLE_MODES}'
            return response

        if mode == self._current_mode:
            response.success = True
            response.message = f'Already in {mode} mode'
            return response

        # Build command
        if mode == 'idle':
            cmd = None
        else:
            cmd = list(MODE_COMMANDS[mode])
            if mode == 'navigation':
                map_path = request.map_path.strip()
                if not map_path:
                    response.success = False
                    response.message = 'map_path required for navigation mode'
                    return response
                cmd.append(f'map:={map_path}')

            # Forward use_depth to any mode — all four parent launch files
            # (teleop=robot, slam, navigation, slam_nav) declare the arg.
            cmd.append(f'use_depth:={"true" if request.use_depth else "false"}')

        # Capture current robot pose if we're transitioning slam* → navigation
        # so AMCL can start localized where slam_toolbox left the robot,
        # rather than at the (0,0,0) default in nav2_params.yaml.
        carryover_pose = None
        if mode == 'navigation' and self._current_mode in ('slam', 'slam_nav'):
            carryover_pose = self._snapshot_map_pose()
            if carryover_pose:
                self.get_logger().info(
                    f'Captured slam→nav pose carryover: '
                    f'x={carryover_pose[0]:.2f} y={carryover_pose[1]:.2f} '
                    f'yaw={math.degrees(carryover_pose[2]):.0f}°')
            else:
                self.get_logger().warn(
                    'slam→nav transition: could not capture map→base_footprint, '
                    'AMCL will start at nav2_params default')

        # Stop current launch
        self._stop_current()

        # Start new launch
        if cmd:
            try:
                env = os.environ.copy()
                # Ensure ROS2 environment is available to subprocess
                self._process = subprocess.Popen(
                    cmd,
                    env=env,
                    preexec_fn=os.setsid,   # own process group for clean kill
                )
                self.get_logger().info(f'Started mode: {mode} (PID {self._process.pid})')
            except Exception as e:
                response.success = False
                response.message = f'Failed to start {mode}: {e}'
                return response

        self._current_mode = mode

        # Schedule delayed /initialpose publish so AMCL has time to come up
        # via lifecycle_manager. We publish multiple times across the window
        # in case the first one races AMCL's subscription readiness.
        # Clear the cancel flag right before launching so the new thread is
        # allowed to run; any subsequent _set_mode_callback will set() it
        # again and interrupt this thread.
        if carryover_pose:
            self._carryover_cancel.clear()
            threading.Thread(
                target=self._publish_carryover_pose,
                args=(carryover_pose,),
                daemon=True,
            ).start()

        response.success = True
        response.message = f'Switched to {mode}'
        return response

    def _snapshot_map_pose(self):
        """Look up map → base_footprint, return (x, y, yaw) or None on failure.

        slam_toolbox's map→odom publish is bursty — it pauses between scan
        matches and may briefly be >1s stale. A single 1s-timeout lookup can
        extrapolate-fail in that gap, so we retry within a ~1s budget and
        only log+return None if every attempt fails. We never raise out of
        here: callers fall back to "no carryover" rather than failing the
        service call.
        """
        deadline = time.monotonic() + 1.0
        attempt = 0
        last_err = None
        while time.monotonic() < deadline:
            attempt += 1
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(),                # latest available
                    timeout=Duration(seconds=0.3),
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                qz = tf.transform.rotation.z
                qw = tf.transform.rotation.w
                yaw = 2.0 * math.atan2(qz, qw)
                if attempt > 1:
                    self.get_logger().info(
                        f'map→base_footprint TF acquired on attempt {attempt}')
                return (x, y, yaw)
            except Exception as e:
                last_err = e
                # Brief pause before retry — gives slam_toolbox time to
                # publish the next map→odom update.
                time.sleep(0.15)
        self.get_logger().warn(
            f'TF lookup map→base_footprint failed after {attempt} attempts: '
            f'{last_err}')
        return None

    def _publish_carryover_pose(self, pose):
        """Publish /initialpose at t+3, t+6, t+10 seconds after launch.

        AMCL is brought up via lifecycle_manager (configure → activate)
        after navigation.launch.py starts; it typically takes 5-10s before
        the /initialpose subscription is ready. Publishing 3 times gives
        whichever event comes first a chance to land.

        Cancellation: we check self._carryover_cancel between sleeps and
        before each publish. _set_mode_callback set()s this on every mode
        change, so a user switching away from navigation mid-wait stops
        further /initialpose publishes immediately. This avoids the prior
        race where reading self._current_mode without a lock could let a
        publish slip out after a mode change.
        """
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(math.cos(yaw / 2.0))
        cov = [0.0] * 36
        cov[0]  = 0.05 ** 2                # 5 cm σ — we trust this pose
        cov[7]  = 0.05 ** 2
        cov[35] = (math.pi / 36.0) ** 2    # 5° σ
        msg.pose.covariance = cov

        last = 0.0
        for t in (3.0, 6.0, 10.0):
            # Sleep in small slices so cancellation kicks in promptly even
            # during the long t+10 leg.
            target = time.monotonic() + (t - last)
            last = t
            while time.monotonic() < target:
                if self._carryover_cancel.is_set():
                    return
                time.sleep(min(0.1, target - time.monotonic()))
            if self._carryover_cancel.is_set():
                return
            msg.header.stamp = self.get_clock().now().to_msg()
            try:
                self._initialpose_pub.publish(msg)
                self.get_logger().info(
                    f'Published slam→nav carryover /initialpose (t+{t:.0f}s)')
            except Exception as e:
                self.get_logger().warn(f'/initialpose publish failed: {e}')

    def _save_map_callback(self, request, response):
        map_name = request.map_name.strip() or 'map'
        # Sanitize: allow only alphanumeric, dashes, underscores
        safe = all(c.isalnum() or c in '-_' for c in map_name)
        if not safe:
            response.success = False
            response.path = ''
            response.message = 'Invalid map name (use alphanumeric, dash, underscore only)'
            return response

        maps_dir = os.path.expanduser('~/maps')
        os.makedirs(maps_dir, exist_ok=True)
        map_path = os.path.join(maps_dir, map_name)

        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path]
        try:
            result = subprocess.run(cmd, timeout=15, capture_output=True, text=True)
            if result.returncode == 0:
                response.success = True
                response.path = map_path + '.yaml'
                response.message = f'Map saved to {map_path}.yaml'
                self.get_logger().info(f'Map saved: {map_path}')
                # Snapshot the live learned-markers file (if any) into a
                # per-map sidecar. navigation.launch.py reads
                # <map_path>.markers.yaml when this map is loaded for nav.
                # Without this snapshot, markers learned during slam_nav
                # would either be lost on relaunch or contaminate other
                # maps via the shared global file.
                live_markers = os.path.expanduser('~/roscar_ws/learned_markers.yaml')
                sidecar = map_path + '.markers.yaml'
                try:
                    if os.path.exists(live_markers):
                        import shutil
                        shutil.copy2(live_markers, sidecar)
                        self.get_logger().info(f'Markers snapshot saved: {sidecar}')
                    else:
                        # No markers learned this session — write an empty
                        # sidecar so nav can't accidentally load a leftover
                        # from an older session under the same map name.
                        with open(sidecar, 'w') as f:
                            f.write('{}\n')
                        self.get_logger().info(f'No live markers; wrote empty sidecar {sidecar}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to snapshot markers sidecar: {e}')
            else:
                response.success = False
                response.path = ''
                response.message = result.stderr.strip() or 'map_saver_cli failed'
        except subprocess.TimeoutExpired:
            response.success = False
            response.path = ''
            response.message = 'Map save timed out (15s)'
        except Exception as e:
            response.success = False
            response.path = ''
            response.message = str(e)
        return response

    def _get_status_callback(self, request, response):
        response.mode = self._current_mode
        response.available_modes = AVAILABLE_MODES
        response.robot_running = (self._process is not None
                                  and self._process.poll() is None)

        # Get running ROS2 node names via subprocess (fast, non-blocking)
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                timeout=3, capture_output=True, text=True
            )
            nodes = [n.strip() for n in result.stdout.splitlines() if n.strip()]
        except Exception:
            nodes = []
        response.active_nodes = nodes
        return response

    def _list_maps_callback(self, request, response):
        maps_dir = os.path.expanduser('~/maps')
        if not os.path.isdir(maps_dir):
            response.map_names = []
            return response

        yaml_files = globmod.glob(os.path.join(maps_dir, '*.yaml'))
        # Return basenames without extension, sorted alphabetically
        names = sorted(
            os.path.splitext(os.path.basename(f))[0] for f in yaml_files
        )
        response.map_names = names
        return response

    # ------------------------------------------------------------------ #
    #  Process management                                                  #
    # ------------------------------------------------------------------ #

    def _stop_current(self):
        """Stop the currently running launch process gracefully."""
        with self._process_lock:
            if self._process is None:
                return
            if self._process.poll() is not None:
                self._process = None
                return

            self.get_logger().info(
                f'Stopping {self._current_mode} (PID {self._process.pid})')
            try:
                # Send SIGINT to the entire process group (same as Ctrl-C)
                os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Graceful stop timed out, using SIGTERM')
                try:
                    os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass  # Process already gone
            finally:
                self._process = None

    # ------------------------------------------------------------------ #
    #  Nav-goal shim                                                       #
    # ------------------------------------------------------------------ #

    def _on_nav_goal(self, msg: PoseStamped):
        """Forward a PoseStamped published on /web/nav_goal to Nav2."""
        if not self._nav_action.server_is_ready():
            self.get_logger().warn(
                '/navigate_to_pose action server not ready (is nav mode active?)')
            return
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self.get_logger().info(
            f'Forwarding nav goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}) frame={msg.header.frame_id}')
        future = self._nav_action.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'send_goal_async failed: {e}')
            self._publish_nav_result(-1)  # custom: send-goal failed
            return
        if not handle.accepted:
            self.get_logger().warn('Nav goal REJECTED by /navigate_to_pose')
            self._publish_nav_result(-2)  # custom: rejected
            return
        with self._nav_goal_lock:
            self._nav_goal_handle = handle
        self.get_logger().info('Nav goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'get_result_async failed: {e}')
            self._publish_nav_result(-3)  # custom: get-result failed
            return
        # action_msgs/GoalStatus: 4=succeeded, 5=cancelled, 6=aborted
        status = int(getattr(result, 'status', 0))
        self.get_logger().info(f'Nav goal finished (status={status})')
        self._publish_nav_result(status)
        with self._nav_goal_lock:
            self._nav_goal_handle = None

    def _publish_nav_result(self, status: int):
        """Notify dashboard a goal has finished so the hint can re-show."""
        try:
            self._nav_result_pub.publish(Int8(data=int(status)))
        except Exception as e:
            self.get_logger().warn(f'nav_goal_result publish failed: {e}')

    def _on_cancel_nav_goal(self, _msg: Empty):
        """Cancel the active goal, if any."""
        with self._nav_goal_lock:
            handle = self._nav_goal_handle
        if handle is None:
            self.get_logger().info('cancel_nav_goal: no active goal')
            return
        self.get_logger().info('Cancelling active nav goal')
        handle.cancel_goal_async()

    def destroy_node(self):
        self._stop_current()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
