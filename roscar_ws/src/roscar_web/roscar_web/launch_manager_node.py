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

import rclpy
from rclpy.node import Node

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

        self._set_mode_srv = self.create_service(
            SetMode, '/web/set_mode', self._set_mode_callback)
        self._save_map_srv = self.create_service(
            SaveMap, '/web/save_map', self._save_map_callback)
        self._get_status_srv = self.create_service(
            GetStatus, '/web/get_status', self._get_status_callback)
        self._list_maps_srv = self.create_service(
            ListMaps, '/web/list_maps', self._list_maps_callback)

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
        response.success = True
        response.message = f'Switched to {mode}'
        return response

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
