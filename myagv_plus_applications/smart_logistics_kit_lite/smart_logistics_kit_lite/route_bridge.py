#!/usr/bin/env python3

from collections import deque
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Optional

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Int16MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


PACKAGE_NAME = 'smart_logistics_kit_lite'


class RouteGraphError(Exception):
    pass


class RouteGraph:
    def __init__(self, graph_file: str):
        self.graph_file = Path(graph_file)
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
            raise RouteGraphError('route graph has no nodes')
        return best_node, best_dist

    def find_path(self, start_id: int, goal_id: int) -> Optional[list[int]]:
        if start_id == goal_id:
            return [start_id]

        queue_items = deque([(start_id, [start_id])])
        visited = {start_id}
        while queue_items:
            current_id, path = queue_items.popleft()
            for next_id, _edge_id in self.adjacency.get(current_id, []):
                if next_id in visited:
                    continue
                next_path = path + [next_id]
                if next_id == goal_id:
                    return next_path
                visited.add(next_id)
                queue_items.append((next_id, next_path))

        return None


class RouteBridge(Node):
    def __init__(self):
        super().__init__('route_bridge')

        default_graph_file = self.get_default_graph_file()
        self.graph_file = self.declare_parameter('roadnet_file', default_graph_file).value
        self.route_frame = self.declare_parameter('route_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.tf_timeout = float(self.declare_parameter('tf_timeout', 2.0).value)
        self.feedback_log_interval = float(self.declare_parameter('feedback_log_interval', 1.0).value)
        self.final_pose_enabled = self.declare_parameter('final_pose_enabled', True).value
        self.final_distance_warn_threshold = float(
            self.declare_parameter('final_distance_warn_threshold', 1.0).value)
        self.route_retry_count = int(self.declare_parameter('route_retry_count', 1).value)

        self.graph = RouteGraph(self.graph_file)
        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_event_queue = queue.Queue(maxsize=5)
        self.cancel_requested = False
        self.navigation_active = False

        self.create_subscription(PoseStamped, 'route_bridge/goal_pose', self.pose_nav_callback, 10)
        self.create_subscription(Int16MultiArray, 'route_bridge/goal_nodes', self.node_nav_callback, 10)
        self.create_subscription(Bool, 'route_bridge/cancel', self.cancel_nav_callback, 10)
        self.action_feedback_pub = self.create_publisher(String, 'route_bridge/feedback', 5)

        self.worker = threading.Thread(target=self.handle_nav_events, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'route_bridge started. graph={self.graph_file}, '
            f'nodes={len(self.graph.nodes)}, edges={len(self.graph.edges)}'
        )

    def get_default_graph_file(self) -> str:
        try:
            pkg_dir = get_package_share_directory(PACKAGE_NAME)
            workspace_dir = Path(pkg_dir).resolve().parents[3]
            return str(
                workspace_dir / 'src' / 'myagv_plus_applications' /
                PACKAGE_NAME / 'graphs' / 'map.geojson')
        except PackageNotFoundError:
            return str(
                Path.cwd() / 'src' / 'myagv_plus_applications' /
                PACKAGE_NAME / 'graphs' / 'map.geojson')

    def enqueue_event(self, event) -> None:
        try:
            self.nav_event_queue.put_nowait(event)
        except queue.Full:
            self.get_logger().warn('route_bridge event queue is full. Drop new navigation request.')

    def clear_nav_event_queue(self) -> None:
        while True:
            try:
                self.nav_event_queue.get_nowait()
            except queue.Empty:
                return

    def node_nav_callback(self, msg: Int16MultiArray) -> None:
        if len(msg.data) == 1:
            self.enqueue_event(('current', int(msg.data[0])))
            return
        if len(msg.data) < 2:
            self.get_logger().warn('route_bridge/goal_nodes requires one goal ID or two IDs: [start_id, goal_id].')
            return
        self.enqueue_event(('id', int(msg.data[0]), int(msg.data[1])))

    def pose_nav_callback(self, msg: PoseStamped) -> None:
        self.enqueue_event(('pose', msg))

    def cancel_nav_callback(self, msg: Bool) -> None:
        if msg.data:
            self.clear_nav_event_queue()
            self.cancel_requested = self.navigation_active
            self.navigator.cancelTask()
            self.action_feedback_pub.publish(String(data='route_bridge_canceled'))
            self.get_logger().warn('Navigation task canceled via route_bridge/cancel.')

    def handle_nav_events(self) -> None:
        while rclpy.ok():
            try:
                event = self.nav_event_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            event_type = event[0]
            self.cancel_requested = False
            self.navigation_active = True
            try:
                if event_type == 'id':
                    _event_type, start_id, goal_id = event
                    self.handle_id_navigation(start_id, goal_id)
                elif event_type == 'current':
                    _event_type, goal_id = event
                    self.handle_current_navigation(goal_id)
                elif event_type == 'pose':
                    _event_type, goal_pose = event
                    self.handle_pose_navigation(goal_pose)
            except Exception as error:
                self.get_logger().error(f'route_bridge navigation failed: {error}')
                self.action_feedback_pub.publish(String(data='route_bridge_failed'))
            finally:
                self.navigation_active = False
                self.cancel_requested = False

    def handle_id_navigation(self, start_id: int, goal_id: int) -> None:
        self.get_logger().info(f'ID route navigation: {start_id} -> {goal_id}')
        if self.follow_route_with_retries(start_id, goal_id):
            self.action_feedback_pub.publish(String(data='route_bridge_succeeded'))
        elif self.cancel_requested:
            self.action_feedback_pub.publish(String(data='route_bridge_canceled'))
        else:
            self.action_feedback_pub.publish(String(data='route_bridge_failed'))

    def handle_current_navigation(self, goal_id: int) -> None:
        current_xy = self.get_current_xy()
        if current_xy is None:
            self.action_feedback_pub.publish(String(data='route_bridge_failed'))
            return

        start_id, start_distance = self.graph.nearest_node(*current_xy)
        self.get_logger().info(
            f'Current-nearest route navigation: nearest_start={start_id} '
            f'({start_distance:.3f} m), goal_id={goal_id}'
        )
        self.handle_id_navigation(start_id, goal_id)

    def handle_pose_navigation(self, goal_pose: PoseStamped) -> None:
        goal_pose.header.frame_id = goal_pose.header.frame_id or self.route_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        current_xy = self.get_current_xy()
        if current_xy is None:
            self.action_feedback_pub.publish(String(data='route_bridge_failed'))
            return

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        start_id, start_distance = self.graph.nearest_node(*current_xy)
        goal_id, goal_distance = self.graph.nearest_node(goal_x, goal_y)

        self.get_logger().info(
            f'Pose route navigation: target=({goal_x:.3f}, {goal_y:.3f}), '
            f'nearest_start={start_id} ({start_distance:.3f} m), '
            f'nearest_goal={goal_id} ({goal_distance:.3f} m)'
        )
        if goal_distance > self.final_distance_warn_threshold:
            self.get_logger().warn(
                f'Final ordinary Nav2 segment is {goal_distance:.3f} m, '
                f'larger than threshold {self.final_distance_warn_threshold:.3f} m.'
            )

        if start_id != goal_id and not self.follow_route_with_retries(start_id, goal_id):
            if self.cancel_requested:
                self.action_feedback_pub.publish(String(data='route_bridge_canceled'))
            else:
                self.action_feedback_pub.publish(String(data='route_bridge_failed'))
            return

        if self.final_pose_enabled:
            if self.go_to_final_pose(goal_pose):
                self.action_feedback_pub.publish(String(data='route_bridge_succeeded'))
            elif self.cancel_requested:
                self.action_feedback_pub.publish(String(data='route_bridge_canceled'))
            else:
                self.action_feedback_pub.publish(String(data='route_bridge_failed'))
        else:
            self.action_feedback_pub.publish(String(data='route_bridge_succeeded'))

    def get_current_xy(self) -> Optional[tuple[float, float]]:
        deadline = time.monotonic() + self.tf_timeout
        last_error = None

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.route_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                translation = transform.transform.translation
                return translation.x, translation.y
            except TransformException as error:
                last_error = error

        self.get_logger().warn(
            f'Cannot get TF {self.route_frame} -> {self.base_frame}: {last_error}'
        )
        return None

    def validate_id_route(self, start_id: int, goal_id: int) -> bool:
        missing = [node_id for node_id in (start_id, goal_id) if not self.graph.has_node(node_id)]
        if missing:
            self.get_logger().warn(f'Invalid route node id: {missing}.')
            return False

        graph_path = self.graph.find_path(start_id, goal_id)
        if graph_path is None:
            self.get_logger().warn(f'No directed route in graph: {start_id} -> {goal_id}.')
            return False

        self.get_logger().info(f'Graph precheck path: {" -> ".join(map(str, graph_path))}')
        return True

    def follow_route_with_retries(self, start_id: int, goal_id: int) -> bool:
        max_attempts = max(1, self.route_retry_count + 1)
        current_start_id = start_id

        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                current_xy = self.get_current_xy()
                if current_xy is None:
                    return False
                current_start_id, start_distance = self.graph.nearest_node(*current_xy)
                self.get_logger().warn(
                    f'Retry route attempt {attempt}/{max_attempts}: '
                    f'nearest_start={current_start_id} ({start_distance:.3f} m), goal_id={goal_id}'
                )

            if current_start_id == goal_id:
                self.get_logger().info(f'Already nearest to route goal node {goal_id}.')
                return True
            if not self.validate_id_route(current_start_id, goal_id):
                return False
            if self.follow_route_segment(current_start_id, goal_id):
                return True
            if self.cancel_requested:
                return False
            if attempt < max_attempts:
                self.get_logger().warn('Route execution failed. Replanning from current robot pose.')

        return False

    def follow_route_segment(self, start_id: int, goal_id: int) -> bool:
        self.get_logger().info(f'Request route by node id: {start_id} -> {goal_id}')
        result = self.navigator.getRoute(start=start_id, goal=goal_id)
        if result is None:
            error_code, error_msg = self.navigator.getTaskError()
            self.get_logger().error(f'Route request failed: code={error_code}, msg={error_msg}')
            return False

        path, route = result
        self.log_route_result(path, route)
        return self.follow_path(path)

    def log_route_result(self, path: NavPath, route) -> None:
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

    def format_feedback(self, feedback) -> Optional[str]:
        items = []
        if hasattr(feedback, 'distance_remaining'):
            items.append(f'distance_remaining={feedback.distance_remaining:.3f} m')
        if hasattr(feedback, 'distance_to_goal'):
            items.append(f'distance_to_goal={feedback.distance_to_goal:.3f} m')
        if hasattr(feedback, 'speed'):
            items.append(f'speed={feedback.speed:.3f} m/s')
        if hasattr(feedback, 'navigation_time'):
            items.append(f'navigation_time={feedback.navigation_time.sec} s')
        return ', '.join(items) if items else None

    def follow_path(self, path: NavPath) -> bool:
        self.get_logger().warn('Robot will follow the Route Server path.')
        task = self.navigator.followPath(path)
        if task is None:
            task = RunningTask.FOLLOW_PATH

        last_feedback_log_time = 0.0
        while not self.navigator.isTaskComplete(task=task):
            if self.cancel_requested:
                return False

            feedback = self.navigator.getFeedback(task=task)
            now = time.monotonic()
            if feedback is not None and now - last_feedback_log_time >= self.feedback_log_interval:
                feedback_text = self.format_feedback(feedback)
                if feedback_text:
                    self.get_logger().info(feedback_text)
                    last_feedback_log_time = now
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('FollowPath succeeded.')
            return True
        if result == TaskResult.CANCELED:
            self.get_logger().warn('FollowPath canceled.')
            return False

        error_code, error_msg = self.navigator.getTaskError()
        self.get_logger().error(f'FollowPath failed: result={result}, code={error_code}, msg={error_msg}')
        return False

    def go_to_final_pose(self, goal_pose: PoseStamped) -> bool:
        self.get_logger().warn(
            f'Robot will use ordinary Nav2 for final pose: '
            f'({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})'
        )
        task = self.navigator.goToPose(goal_pose)
        if task is None:
            error_code, error_msg = self.navigator.getTaskError()
            self.get_logger().error(f'goToPose rejected: code={error_code}, msg={error_msg}')
            return False

        last_feedback_log_time = 0.0
        while not self.navigator.isTaskComplete(task=task):
            if self.cancel_requested:
                return False

            feedback = self.navigator.getFeedback(task=task)
            now = time.monotonic()
            if feedback is not None and now - last_feedback_log_time >= self.feedback_log_interval:
                feedback_text = self.format_feedback(feedback)
                if feedback_text:
                    self.get_logger().info(feedback_text)
                    last_feedback_log_time = now
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Final goToPose succeeded.')
            return True
        if result == TaskResult.CANCELED:
            self.get_logger().warn('Final goToPose canceled.')
            return False

        error_code, error_msg = self.navigator.getTaskError()
        self.get_logger().error(f'Final goToPose failed: result={result}, code={error_code}, msg={error_msg}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = RouteBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except RouteGraphError as error:
        print(f'[route_bridge] {error}')
        print('[route_bridge] Check src/myagv_plus_applications/smart_logistics_kit_lite/graphs/map.geojson.')
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Interrupted by user.')
            node.navigator.cancelTask()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
