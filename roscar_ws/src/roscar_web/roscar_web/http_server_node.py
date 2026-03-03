"""HTTP server node: serves the web dashboard static files.

Runs Python's built-in http.server in a background thread, serving
the web/ directory on the configured port. The user navigates to
http://<robot-ip>:<port>/ to access the dashboard.
"""

import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SilentHTTPHandler(SimpleHTTPRequestHandler):
    """HTTP handler that suppresses per-request log noise."""

    def log_message(self, format, *args):
        pass  # Suppress default stdout logging

    def log_error(self, format, *args):
        pass


class HttpServerNode(Node):
    def __init__(self):
        super().__init__('http_server')

        self.declare_parameter('port', 8888)
        port = self.get_parameter('port').value

        # Locate the web/ directory from our installed package share
        pkg_share = get_package_share_directory('roscar_web')
        web_dir = os.path.join(pkg_share, 'web')

        if not os.path.isdir(web_dir):
            self.get_logger().error(f'Web directory not found: {web_dir}')
            return

        handler = partial(SilentHTTPHandler, directory=web_dir)
        self._server = HTTPServer(('0.0.0.0', port), handler)

        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'Web dashboard: http://0.0.0.0:{port}/ (serving {web_dir})')

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HttpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
