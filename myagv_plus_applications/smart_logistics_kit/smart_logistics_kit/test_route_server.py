#! /usr/bin/env python3
"""
Interactive real-robot Route Server test for myAGV Plus.

The script is intended to be launched by ros2 run. It validates node IDs against
the local route graph before sending requests to route_server.
"""

from collections import deque
import json
import math
from pathlib import Path as FilePath
import time
from typing import Optional

try:
    import readline
except ImportError:
    readline = None

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Route
from nav_msgs.msg import Path as NavPath
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int16MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


PACKAGE_NAME = 'smart_logistics_kit'


class RouteGraphError(Exception):
    pass


class RouteGraph:
    def __init__(self, graph_file: str):
        self.graph_file = FilePath(graph_file)
        self.nodes = {}
        self.edges = []
        self.adjacency = {}
        self.load()

    def load(self) -> None:
        try:
            with self.graph_file.open('r', encoding='utf-8') as graph:
                data = json.load(graph)
        except FileNotFoundError as error:
            raise RouteGraphError(f'route graph file does not exist: {self.graph_file}') from error
        except json.JSONDecodeError as error:
            raise RouteGraphError(f'invalid route graph JSON/GeoJSON: {self.graph_file}, {error}') from error

        nodes = {}
        edges = []
        for feature in data.get('features', []):
            geometry = feature.get('geometry', {})
            properties = feature.get('properties', {})

            if geometry.get('type') == 'Point' and 'id' in properties:
                coords = geometry.get('coordinates', [])
                if len(coords) >= 2:
                    nodes[int(properties['id'])] = (float(coords[0]), float(coords[1]))
                continue

            if 'startid' in properties and 'endid' in properties:
                edges.append((
                    int(properties['startid']),
                    int(properties['endid']),
                    int(properties.get('id', -1)),
                ))

        adjacency = {node_id: [] for node_id in nodes}
        for start_id, end_id, edge_id in edges:
            adjacency.setdefault(start_id, []).append((end_id, edge_id))

        self.nodes = nodes
        self.edges = edges
        self.adjacency = adjacency

        if not self.nodes:
            raise RouteGraphError(f'route graph has no Point nodes: {self.graph_file}')
        if not self.edges:
            raise RouteGraphError(f'route graph has no LineString/MultiLineString edges: {self.graph_file}')

    def has_node(self, node_id: int) -> bool:
        return node_id in self.nodes

    def nearest_node(self, x: float, y: float) -> tuple[int, float]:
        best_node = None
        best_dist = float('inf')
        for node_id, (node_x, node_y) in self.nodes.items():
            dist = math.hypot(node_x - x, node_y - y)
            if dist < best_dist:
                best_node = node_id
                best_dist = dist
        if best_node is None:
            raise ValueError('route graph has no nodes')
        return best_node, best_dist

    def find_path(self, start_id: int, goal_id: int) -> Optional[list[int]]:
        if start_id == goal_id:
            return [start_id]

        queue = deque([(start_id, [start_id])])
        visited = {start_id}
        while queue:
            current_id, path = queue.popleft()
            for next_id, _edge_id in self.adjacency.get(current_id, []):
                if next_id in visited:
                    continue
                next_path = path + [next_id]
                if next_id == goal_id:
                    return next_path
                visited.add(next_id)
                queue.append((next_id, next_path))

        return None

class RouteServerRealTest(Node):
    def __init__(self):
        super().__init__('route_server_real_test')

        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_nodes_pub = self.create_publisher(Int16MultiArray, 'route_bridge/goal_nodes', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'route_bridge/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Bool, 'route_bridge/cancel', 10)
        self.create_subscription(String, 'route_bridge/feedback', self.bridge_feedback_callback, 10)
        self.bridge_feedback = None

        self.run_motion = self.declare_parameter('run_motion', True).value
        self.bridge_feedback_timeout = float(
            self.declare_parameter('bridge_feedback_timeout', 600.0).value)
        self.wait_nav2_active = self.declare_parameter('wait_nav2_active', False).value
        self.set_initial_pose_enable = self.declare_parameter('set_initial_pose', False).value
        self.route_frame = self.declare_parameter('route_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.tf_timeout = float(self.declare_parameter('tf_timeout', 2.0).value)

        try:
            pkg_dir = get_package_share_directory(PACKAGE_NAME)
            workspace_dir = FilePath(pkg_dir).resolve().parents[3]
            default_graph_file = str(
                workspace_dir / 'src' / 'myagv_plus_applications' /
                PACKAGE_NAME / 'graphs' / 'map.geojson')
        except PackageNotFoundError:
            default_graph_file = str(
                FilePath.cwd() / 'src' / 'myagv_plus_applications' /
                PACKAGE_NAME / 'graphs' / 'map.geojson')
        self.graph_file = self.declare_parameter('graph_file', default_graph_file).value

        self.initial_x = self.declare_parameter('initial_x', 0.317).value
        self.initial_y = self.declare_parameter('initial_y', 0.166).value
        self.initial_yaw = self.declare_parameter('initial_yaw', 0.0).value

        self.graph = RouteGraph(self.graph_file)
        self.get_logger().info(
            f'Loaded route graph: {self.graph_file}, '
            f'nodes={len(self.graph.nodes)}, edges={len(self.graph.edges)}, '
            f'run_motion={self.run_motion}'
        )

    def bridge_feedback_callback(self, msg: String) -> None:
        self.bridge_feedback = msg.data
        self.get_logger().info(f'route_bridge feedback: {msg.data}')

    def setup_terminal_input(self) -> None:
        if readline is None:
            return

        readline.set_history_length(100)
        readline.parse_and_bind('set editing-mode emacs')
        readline.parse_and_bind(r'"\e[3~": delete-char')
        readline.parse_and_bind(r'"\C-h": backward-delete-char')
        readline.parse_and_bind(r'"\C-?": backward-delete-char')

    def make_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.route_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
        return pose

    def make_pose_with_zw(self, x: float, y: float, z: float, w: float) -> Optional[PoseStamped]:
        norm = math.hypot(z, w)
        if norm < 1e-6:
            self.get_logger().warn('Invalid orientation: z and w cannot both be 0.')
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.route_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(z) / norm
        pose.pose.orientation.w = float(w) / norm
        return pose

    def prepare_nav2(self) -> None:
        if self.set_initial_pose_enable:
            initial_pose = self.make_pose(self.initial_x, self.initial_y, self.initial_yaw)
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info(
                f'Set initial pose: x={self.initial_x:.3f}, y={self.initial_y:.3f}, '
                f'yaw={self.initial_yaw:.3f}'
            )

        if self.wait_nav2_active:
            self.get_logger().info('Waiting until Nav2 active...')
            self.navigator.waitUntilNav2Active()

    def get_current_xy(self) -> Optional[tuple[float, float]]:
        deadline = time.monotonic() + self.tf_timeout
        last_error = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.route_frame,
                    self.base_frame,
                    Time(),
                )
                translation = transform.transform.translation
                return translation.x, translation.y
            except TransformException as error:
                last_error = error

        self.get_logger().warn(
            f'Cannot get TF {self.route_frame} -> {self.base_frame}: {last_error}'
        )
        return None

    def validate_id_route(self, start_id: int, goal_id: int) -> Optional[list[int]]:
        missing = [node_id for node_id in (start_id, goal_id) if not self.graph.has_node(node_id)]
        if missing:
            self.get_logger().warn(f'Invalid route node id: {missing}. Skip request.')
            return None

        graph_path = self.graph.find_path(start_id, goal_id)
        if graph_path is None:
            self.get_logger().warn(
                f'No directed route in graph: {start_id} -> {goal_id}. Skip request.'
            )
            return None

        self.get_logger().info(f'Graph precheck path: {" -> ".join(map(str, graph_path))}')
        return graph_path

    def wait_for_bridge_subscriber(self, publisher, topic_name: str) -> bool:
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No route_bridge subscriber found on {topic_name}. '
            f'Start navigation2_active.launch.py first.'
        )
        return False

    def wait_for_bridge_result(self) -> None:
        final_statuses = {
            'route_bridge_succeeded',
            'route_bridge_failed',
            'route_bridge_canceled',
        }
        deadline = time.monotonic() + self.bridge_feedback_timeout

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.bridge_feedback in final_statuses:
                    return
        except KeyboardInterrupt:
            self.publish_cancel()
            return

        self.get_logger().warn(
            f'No final route_bridge feedback within {self.bridge_feedback_timeout:.1f} s.'
        )

    def publish_cancel(self) -> None:
        if not self.wait_for_bridge_subscriber(self.cancel_pub, '/route_bridge/cancel'):
            return
        self.cancel_pub.publish(Bool(data=True))
        self.get_logger().warn('Cancel request sent to route_bridge.')

    def publish_bridge_request(self, request: tuple) -> None:
        self.bridge_feedback = None
        mode = request[0]

        if mode == 'ids':
            start_id = int(request[1])
            goal_id = int(request[2])
            if self.validate_id_route(start_id, goal_id) is None:
                return
            if not self.wait_for_bridge_subscriber(self.goal_nodes_pub, '/route_bridge/goal_nodes'):
                return
            self.goal_nodes_pub.publish(Int16MultiArray(data=[start_id, goal_id]))
            self.get_logger().info(f'Sent route_bridge node request: {start_id} -> {goal_id}')

        elif mode == 'current':
            goal_id = int(request[1])
            if not self.graph.has_node(goal_id):
                self.get_logger().warn(f'Invalid goal node id: {goal_id}. Skip request.')
                return

            current_xy = self.get_current_xy()
            if current_xy is None:
                self.get_logger().warn('Cannot determine current nearest route node. Skip request.')
                return
            nearest_id, distance = self.graph.nearest_node(*current_xy)
            if self.validate_id_route(nearest_id, goal_id) is None:
                return

            if not self.wait_for_bridge_subscriber(self.goal_nodes_pub, '/route_bridge/goal_nodes'):
                return
            self.goal_nodes_pub.publish(Int16MultiArray(data=[goal_id]))
            self.get_logger().info(
                f'Sent route_bridge current-nearest request: nearest_node={nearest_id} '
                f'({distance:.3f} m), goal_id={goal_id}'
            )

        elif mode == 'pose':
            goal_pose = self.make_pose_with_zw(request[1], request[2], request[3], request[4])
            if goal_pose is None:
                return

            current_xy = self.get_current_xy()
            if current_xy is None:
                self.get_logger().warn('Cannot determine current nearest route node. Skip request.')
                return
            start_id, start_distance = self.graph.nearest_node(*current_xy)
            goal_id, goal_distance = self.graph.nearest_node(
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
            )
            if self.validate_id_route(start_id, goal_id) is None:
                return

            if not self.wait_for_bridge_subscriber(self.goal_pose_pub, '/route_bridge/goal_pose'):
                return
            self.goal_pose_pub.publish(goal_pose)
            self.get_logger().info(
                f'Sent route_bridge pose request: target=({goal_pose.pose.position.x:.3f}, '
                f'{goal_pose.pose.position.y:.3f}), nearest_start={start_id} '
                f'({start_distance:.3f} m), nearest_goal={goal_id} ({goal_distance:.3f} m)'
            )

        else:
            self.get_logger().warn(f'Unsupported request mode: {mode}')
            return

        self.wait_for_bridge_result()

    def request_route_by_ids(self, start_id: int, goal_id: int) -> Optional[list]:
        if self.validate_id_route(start_id, goal_id) is None:
            return None

        self.get_logger().info(f'Request route by node id: {start_id} -> {goal_id}')
        return self.finish_route_request(self.navigator.getRoute(start=start_id, goal=goal_id))

    def request_route_from_current(self, goal_id: int) -> Optional[list]:
        if not self.graph.has_node(goal_id):
            self.get_logger().warn(f'Invalid goal node id: {goal_id}. Skip request.')
            return None

        current_xy = self.get_current_xy()
        if current_xy is None:
            self.get_logger().warn('Cannot determine current nearest route node. Skip request.')
            return None

        nearest_id, distance = self.graph.nearest_node(*current_xy)
        if self.validate_id_route(nearest_id, goal_id) is None:
            return None

        self.get_logger().info(
            f'Request route by nearest node. Current=({current_xy[0]:.3f}, {current_xy[1]:.3f}), '
            f'nearest_node={nearest_id}, nearest_distance={distance:.3f} m, goal_id={goal_id}'
        )
        return self.finish_route_request(self.navigator.getRoute(start=nearest_id, goal=goal_id))

    def preview_pose_request(self, goal_pose: PoseStamped) -> None:
        current_xy = self.get_current_xy()
        if current_xy is None:
            self.get_logger().warn('Cannot determine current nearest route node. Skip request.')
            return

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        start_id, start_distance = self.graph.nearest_node(*current_xy)
        goal_id, goal_distance = self.graph.nearest_node(goal_x, goal_y)
        graph_path = self.validate_id_route(start_id, goal_id)
        if graph_path is None:
            return

        self.get_logger().info(
            f'Pose target=({goal_x:.3f}, {goal_y:.3f}, '
            f'z={goal_pose.pose.orientation.z:.3f}, w={goal_pose.pose.orientation.w:.3f})'
        )
        self.get_logger().info(
            f'Route segment: nearest_start={start_id} ({start_distance:.3f} m), '
            f'nearest_goal={goal_id} ({goal_distance:.3f} m)'
        )

        if start_id != goal_id:
            self.get_logger().info(f'Request route by node id: {start_id} -> {goal_id}')
            result = self.finish_route_request(self.navigator.getRoute(start=start_id, goal=goal_id))
            if result is None:
                return

            path, route = result
            self.log_route_result(path, route)
        else:
            self.get_logger().info('Current pose and target pose map to the same nearest route node.')

        self.get_logger().info('Planning-only pose request complete. Final route_bridge goToPose skipped.')

    def finish_route_request(self, result: Optional[list]) -> Optional[list]:
        if result is None:
            error_code, error_msg = self.navigator.getTaskError()
            self.get_logger().error(f'Route request failed: code={error_code}, msg={error_msg}')
            return None
        return result

    def log_route_result(self, path: NavPath, route: Route) -> None:
        self.get_logger().info(f'Dense path poses: {len(path.poses)}')
        if path.poses:
            first = path.poses[0].pose.position
            last = path.poses[-1].pose.position
            self.get_logger().info(
                f'Path start=({first.x:.3f}, {first.y:.3f}), '
                f'end=({last.x:.3f}, {last.y:.3f})'
            )

        node_ids = [str(node.nodeid) for node in route.nodes]
        edge_ids = [str(edge.edgeid) for edge in route.edges]
        self.get_logger().info(f'Route nodes: {" -> ".join(node_ids) if node_ids else "(none)"}')
        self.get_logger().info(f'Route edges: {" -> ".join(edge_ids) if edge_ids else "(none)"}')

    def execute_plan_request(self, request: tuple) -> None:
        mode = request[0]
        if mode == 'ids':
            first_id = request[1]
            second_id = request[2]
            result = self.request_route_by_ids(first_id, int(second_id))
        elif mode == 'current':
            first_id = request[1]
            result = self.request_route_from_current(first_id)
        elif mode == 'pose':
            goal_pose = self.make_pose_with_zw(request[1], request[2], request[3], request[4])
            if goal_pose is None:
                return
            self.preview_pose_request(goal_pose)
            return
        else:
            self.get_logger().warn(f'Unsupported request mode: {mode}')
            return

        if result is None:
            return

        path, route = result
        self.log_route_result(path, route)
        self.get_logger().info('Planning-only request complete.')

    def execute_request(self, request: tuple, plan_only: bool) -> None:
        if plan_only:
            self.execute_plan_request(request)
            return

        if not self.run_motion:
            self.get_logger().info('run_motion is false. Request will not be sent to route_bridge.')
            return

        self.publish_bridge_request(request)

    def print_menu(self) -> None:
        print('')
        print('Route Bridge Test')
        print('Input:')
        print('  <start_id> <goal_id>    Send route node request, example: 0 1')
        print('  <goal_id>               Send current-nearest route node request, example: 5')
        print('  <x> <y> <z> <w>         Send pose request with final ordinary Nav2 segment')
        print('  p <start_id> <goal_id>  Plan only, example: p 0 1')
        print('  p <goal_id>             Plan only from current nearest route node, example: p 5')
        print('  p <x> <y> <z> <w>       Plan only for pose target route segment')
        print('  c                       Cancel current route_bridge task')
        print('  q                       Quit')
        print(f'Default: run_motion={self.run_motion}')
        print('')

    def parse_request(self, line: str) -> Optional[tuple[tuple, bool]]:
        parts = line.split()
        if not parts:
            return None

        plan_only = False
        if parts[0].lower() in {'p', 'plan'}:
            plan_only = True
            parts = parts[1:]

        if len(parts) == 1:
            try:
                goal_id = int(parts[0])
            except ValueError:
                self.get_logger().warn(f'Invalid input: {line}')
                return None
            return ('current', goal_id, None), plan_only

        if len(parts) == 2:
            try:
                start_id = int(parts[0])
                goal_id = int(parts[1])
            except ValueError:
                self.get_logger().warn(f'Invalid input: {line}')
                return None
            return ('ids', start_id, goal_id), plan_only

        if len(parts) == 4:
            try:
                x, y, z, w = (float(value) for value in parts)
            except ValueError:
                self.get_logger().warn(f'Invalid input: {line}')
                return None
            return ('pose', x, y, z, w), plan_only

        self.get_logger().warn(f'Invalid input: {line}')
        return None

    def run(self) -> None:
        self.prepare_nav2()
        self.setup_terminal_input()
        self.print_menu()

        while rclpy.ok():
            try:
                line = input('route> ').strip()
            except EOFError:
                break

            if not line:
                continue

            command = line.lower()
            if command in {'q', 'quit', 'exit'}:
                break
            if command in {'c', 'cancel'}:
                self.publish_cancel()
                self.print_menu()
                continue
            if command in {'h', 'help', '?'}:
                self.print_menu()
                continue

            parsed = self.parse_request(line)
            if parsed is None:
                continue

            request, plan_only = parsed
            self.execute_request(request, plan_only)
            self.print_menu()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RouteServerRealTest()
        node.run()
    except RouteGraphError as error:
        print(f'[test_route_server] {error}')
        print('[test_route_server] Check src/myagv_plus_applications/smart_logistics_kit/graphs/map.geojson.')
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Interrupted by user.')
            node.navigator.cancelTask()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
