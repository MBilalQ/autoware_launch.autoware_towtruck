#!/usr/bin/env python3

import math
from collections import defaultdict, deque
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from autoware_internal_planning_msgs.msg import PathWithLaneId
except ImportError:  # pragma: no cover - lets the node run in slimmer workspaces.
    PathWithLaneId = None


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    matrix = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), t.x],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), t.y],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), t.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return matrix


def apply_transform(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    if points.size == 0:
        return points
    homogeneous = np.ones((points.shape[0], 4), dtype=np.float64)
    homogeneous[:, :3] = points[:, :3]
    return (homogeneous @ transform_to_matrix(transform).T)[:, :3]


def as_voxel(point: Sequence[float], size: float) -> Tuple[int, int, int]:
    return (
        int(round(point[0] / size)),
        int(round(point[1] / size)),
        int(round(point[2] / size)),
    )


def path_arc_lengths(path: Sequence[Point2]) -> List[float]:
    arcs = [0.0]
    for i in range(1, len(path)):
        arcs.append(arcs[-1] + math.dist(path[i - 1], path[i]))
    return arcs


def project_to_path(point: Point2, path: Sequence[Point2], arcs: Sequence[float]) -> Tuple[float, float]:
    best_lateral = float("inf")
    best_arc = 0.0
    px, py = point
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx, vy = bx - ax, by - ay
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq <= 1e-9:
            continue
        ratio = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / seg_len_sq))
        cx, cy = ax + ratio * vx, ay + ratio * vy
        lateral = math.hypot(px - cx, py - cy)
        if lateral < best_lateral:
            best_lateral = lateral
            best_arc = arcs[i] + math.sqrt(seg_len_sq) * ratio
    return best_lateral, best_arc


class PathCorridorObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__("path_corridor_obstacle_detector")

        self.target_frame = str(self.declare_parameter("target_frame", "map").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.input_pointcloud_topic = str(
            self.declare_parameter(
                "input_pointcloud_topic", "/sensing/lidar/concatenated/pointcloud"
            ).value
        )
        self.map_pointcloud_topic = str(
            self.declare_parameter("map_pointcloud_topic", "/map/pointcloud_map").value
        )
        self.trajectory_topic = str(
            self.declare_parameter(
                "trajectory_topic", "/planning/scenario_planning/trajectory"
            ).value
        )
        self.behavior_path_topic = str(
            self.declare_parameter(
                "behavior_path_topic",
                "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
            ).value
        )
        self.nav_path_topic = str(
            self.declare_parameter("nav_path_topic", "/planning/scenario_planning/path").value
        )
        self.output_pointcloud_topic = str(
            self.declare_parameter(
                "output_pointcloud_topic", "/perception/obstacle_segmentation/pointcloud"
            ).value
        )
        self.distance_topic = str(
            self.declare_parameter(
                "distance_topic", "/perception/obstacle_segmentation/nearest_obstacle_distance"
            ).value
        )

        self.ground_grid_size = float(self.declare_parameter("ground_grid_size", 0.4).value)
        self.obstacle_height_threshold = float(
            self.declare_parameter("obstacle_height_threshold", 0.09).value
        )
        self.map_voxel_size = float(self.declare_parameter("map_voxel_size", 0.15).value)
        self.map_distance_threshold = float(
            self.declare_parameter("map_distance_threshold", 0.22).value
        )
        self.cluster_tolerance = float(self.declare_parameter("cluster_tolerance", 0.45).value)
        self.min_cluster_points = int(self.declare_parameter("min_cluster_points", 4).value)
        self.max_cluster_points = int(self.declare_parameter("max_cluster_points", 5000).value)
        self.path_corridor_width = float(self.declare_parameter("path_corridor_width", 1.6).value)
        self.path_corridor_margin = float(self.declare_parameter("path_corridor_margin", 0.25).value)
        self.min_forward_obstacle_distance = float(
            self.declare_parameter("min_forward_obstacle_distance", 0.5).value
        )
        self.max_detection_range = float(self.declare_parameter("max_detection_range", 35.0).value)
        self.max_input_points = int(self.declare_parameter("max_input_points", 120000).value)
        self.publish_no_obstacle_distance = bool(
            self.declare_parameter("publish_no_obstacle_distance", True).value
        )
        self.self_filter_min_x = float(self.declare_parameter("self_filter_min_x", -2.2).value)
        self.self_filter_max_x = float(self.declare_parameter("self_filter_max_x", 1.6).value)
        self.self_filter_min_y = float(self.declare_parameter("self_filter_min_y", -0.85).value)
        self.self_filter_max_y = float(self.declare_parameter("self_filter_max_y", 0.85).value)
        self.self_filter_min_z = float(self.declare_parameter("self_filter_min_z", -0.5).value)
        self.self_filter_max_z = float(self.declare_parameter("self_filter_max_z", 2.2).value)

        self.map_voxels: Set[Tuple[int, int, int]] = set()
        self.path_xy: List[Point2] = []
        self.path_arcs: List[float] = []
        self.ego_xy: Optional[Point2] = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PointCloud2, self.input_pointcloud_topic, self.on_cloud, 1)
        self.create_subscription(PointCloud2, self.map_pointcloud_topic, self.on_map, map_qos)
        self.create_subscription(Trajectory, self.trajectory_topic, self.on_trajectory, 1)
        self.create_subscription(Path, self.nav_path_topic, self.on_nav_path, 1)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odometry, 1)
        if PathWithLaneId is not None:
            self.create_subscription(PathWithLaneId, self.behavior_path_topic, self.on_lane_path, 1)

        self.obstacle_pub = self.create_publisher(PointCloud2, self.output_pointcloud_topic, 10)
        self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)

        self.get_logger().info(
            "Detecting non-ground, non-map clusters from %s inside the planned path corridor"
            % self.input_pointcloud_topic
        )

    def on_map(self, msg: PointCloud2) -> None:
        points = self.read_xyz(msg)
        if points.size == 0:
            return
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            transform = self.lookup_transform(self.target_frame, msg.header.frame_id)
            if transform is None:
                return
            points = apply_transform(points, transform)
        self.map_voxels = {as_voxel(point, self.map_voxel_size) for point in points}
        self.get_logger().info("Loaded %d static-map voxels for obstacle subtraction" % len(self.map_voxels))

    def on_trajectory(self, msg: Trajectory) -> None:
        self.set_path(
            [(p.pose.position.x, p.pose.position.y) for p in msg.points],
            msg.header.frame_id,
        )

    def on_lane_path(self, msg) -> None:
        self.set_path(
            [(p.point.pose.position.x, p.point.pose.position.y) for p in msg.points],
            msg.header.frame_id,
        )

    def on_nav_path(self, msg: Path) -> None:
        self.set_path(
            [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
            msg.header.frame_id,
        )

    def on_odometry(self, msg: Odometry) -> None:
        if msg.header.frame_id == self.target_frame or not msg.header.frame_id:
            self.ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            return
        transform = self.lookup_transform(self.target_frame, msg.header.frame_id)
        if transform is None:
            return
        point = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]])
        mapped = apply_transform(point, transform)[0]
        self.ego_xy = (float(mapped[0]), float(mapped[1]))

    def set_path(self, path: List[Point2], frame_id: str) -> None:
        if len(path) < 2:
            return
        if frame_id and frame_id != self.target_frame:
            transform = self.lookup_transform(self.target_frame, frame_id)
            if transform is None:
                return
            points = np.array([[x, y, 0.0] for x, y in path], dtype=np.float64)
            mapped = apply_transform(points, transform)
            path = [(float(p[0]), float(p[1])) for p in mapped]
        self.path_xy = path
        self.path_arcs = path_arc_lengths(path)

    def on_cloud(self, msg: PointCloud2) -> None:
        if len(self.path_xy) < 2:
            self.publish_empty(msg.header.stamp)
            return

        points = self.read_xyz(msg)
        if points.size == 0:
            self.publish_empty(msg.header.stamp)
            return

        source_frame = msg.header.frame_id
        points_in_base = points
        if source_frame and source_frame != self.base_frame:
            transform = self.lookup_transform(self.base_frame, source_frame)
            if transform is None:
                self.publish_empty(msg.header.stamp)
                return
            points_in_base = apply_transform(points, transform)
        points_in_base = self.remove_self_points(points_in_base)

        if self.base_frame != self.target_frame:
            transform = self.lookup_transform(self.target_frame, self.base_frame)
            if transform is None:
                self.publish_empty(msg.header.stamp)
                return
            points = apply_transform(points_in_base, transform)
        else:
            points = points_in_base

        points = self.limit_range(points)
        points = self.remove_ground(points)
        points = self.remove_map_points(points)
        clusters = self.cluster(points)

        actionable_points: List[Point3] = []
        nearest_distance = float("inf")
        ego_arc = self.current_ego_arc()
        corridor_half_width = 0.5 * self.path_corridor_width + self.path_corridor_margin

        for cluster in clusters:
            lateral_and_arc = [
                project_to_path((float(point[0]), float(point[1])), self.path_xy, self.path_arcs)
                for point in cluster
            ]
            if not any(lateral <= corridor_half_width for lateral, _ in lateral_and_arc):
                continue
            actionable_points.extend((float(p[0]), float(p[1]), float(p[2])) for p in cluster)
            cluster_arc = min(arc for lateral, arc in lateral_and_arc if lateral <= corridor_half_width)
            distance = cluster_arc - ego_arc
            if distance < self.min_forward_obstacle_distance:
                continue
            nearest_distance = min(nearest_distance, distance)

        self.publish_cloud(actionable_points, msg.header.stamp)
        if math.isfinite(nearest_distance):
            self.publish_distance(nearest_distance)
        elif self.publish_no_obstacle_distance:
            self.publish_distance(-1.0)

    def read_xyz(self, msg: PointCloud2) -> np.ndarray:
        fields = [field.name for field in msg.fields]
        if not {"x", "y", "z"}.issubset(fields):
            self.get_logger().warn("PointCloud2 is missing x/y/z fields", throttle_duration_sec=5.0)
            return np.empty((0, 3), dtype=np.float64)

        points_raw = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if isinstance(points_raw, np.ndarray) and points_raw.dtype.names:
            points = np.column_stack(
                (
                    points_raw["x"].astype(np.float64, copy=False),
                    points_raw["y"].astype(np.float64, copy=False),
                    points_raw["z"].astype(np.float64, copy=False),
                )
            )
        else:
            points_iter: Iterable[Tuple[float, float, float]] = points_raw
            points = np.asarray(list(points_iter), dtype=np.float64)
        if points.ndim != 2 or points.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float64)
        if points.shape[0] > self.max_input_points:
            step = max(1, int(math.ceil(points.shape[0] / self.max_input_points)))
            points = points[::step]
        return points[:, :3]

    def limit_range(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0 or self.max_detection_range <= 0.0:
            return points
        center = self.ego_xy
        if center is None:
            center = self.path_xy[0]
        dx = points[:, 0] - center[0]
        dy = points[:, 1] - center[1]
        return points[(dx * dx + dy * dy) <= self.max_detection_range * self.max_detection_range]

    def remove_self_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        inside_vehicle = (
            (points[:, 0] >= self.self_filter_min_x)
            & (points[:, 0] <= self.self_filter_max_x)
            & (points[:, 1] >= self.self_filter_min_y)
            & (points[:, 1] <= self.self_filter_max_y)
            & (points[:, 2] >= self.self_filter_min_z)
            & (points[:, 2] <= self.self_filter_max_z)
        )
        return points[~inside_vehicle]

    def remove_ground(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        mins: Dict[Tuple[int, int], float] = {}
        for point in points:
            key = (int(math.floor(point[0] / self.ground_grid_size)), int(math.floor(point[1] / self.ground_grid_size)))
            mins[key] = min(mins.get(key, float("inf")), float(point[2]))

        keep = []
        for point in points:
            key = (int(math.floor(point[0] / self.ground_grid_size)), int(math.floor(point[1] / self.ground_grid_size)))
            keep.append(float(point[2]) >= mins[key] + self.obstacle_height_threshold)
        return points[np.asarray(keep, dtype=bool)]

    def remove_map_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0 or not self.map_voxels:
            return points
        radius = max(0, int(math.ceil(self.map_distance_threshold / self.map_voxel_size)))
        keep = []
        for point in points:
            vx, vy, vz = as_voxel(point, self.map_voxel_size)
            is_static = False
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        if (vx + dx, vy + dy, vz + dz) in self.map_voxels:
                            is_static = True
                            break
                    if is_static:
                        break
                if is_static:
                    break
            keep.append(not is_static)
        return points[np.asarray(keep, dtype=bool)]

    def cluster(self, points: np.ndarray) -> List[np.ndarray]:
        if points.size == 0:
            return []
        cell_size = self.cluster_tolerance
        cells: Dict[Tuple[int, int], List[int]] = defaultdict(list)
        for i, point in enumerate(points):
            cells[(int(math.floor(point[0] / cell_size)), int(math.floor(point[1] / cell_size)))].append(i)

        visited = np.zeros(points.shape[0], dtype=bool)
        clusters = []
        tolerance_sq = self.cluster_tolerance * self.cluster_tolerance
        for start in range(points.shape[0]):
            if visited[start]:
                continue
            queue = deque([start])
            visited[start] = True
            indices = []
            while queue:
                idx = queue.popleft()
                indices.append(idx)
                cx = int(math.floor(points[idx][0] / cell_size))
                cy = int(math.floor(points[idx][1] / cell_size))
                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        for neighbor in cells.get((nx, ny), []):
                            if visited[neighbor]:
                                continue
                            diff = points[neighbor, :2] - points[idx, :2]
                            if float(diff @ diff) <= tolerance_sq:
                                visited[neighbor] = True
                                queue.append(neighbor)
                if len(indices) > self.max_cluster_points:
                    break
            if self.min_cluster_points <= len(indices) <= self.max_cluster_points:
                clusters.append(points[indices])
        return clusters

    def current_ego_arc(self) -> float:
        ego = self.ego_xy
        if ego is None:
            transform = self.lookup_transform(self.target_frame, self.base_frame)
            if transform is not None:
                translation = transform.transform.translation
                ego = (translation.x, translation.y)
                self.ego_xy = ego
        if ego is None or len(self.path_xy) < 2:
            return 0.0
        _, arc = project_to_path(ego, self.path_xy, self.path_arcs)
        return arc

    def lookup_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as exc:
            self.get_logger().warn(
                "Missing transform %s <- %s: %s" % (target_frame, source_frame, exc),
                throttle_duration_sec=5.0,
            )
            return None

    def publish_cloud(self, points: Sequence[Point3], stamp) -> None:
        msg_header = self.create_header(stamp)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.obstacle_pub.publish(point_cloud2.create_cloud(msg_header, fields, points))

    def publish_empty(self, stamp) -> None:
        self.publish_cloud([], stamp)
        if self.publish_no_obstacle_distance:
            self.publish_distance(-1.0)

    def publish_distance(self, distance: float) -> None:
        msg = Float32()
        msg.data = float(distance)
        self.distance_pub.publish(msg)

    def create_header(self, stamp):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        return header


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathCorridorObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
