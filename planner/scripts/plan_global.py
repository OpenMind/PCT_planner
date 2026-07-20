"""
plan_global.py  —  PCT Planner with live robot pose + RViz2 goal
================================================================
Usage:
    python3 plan_global.py --scene Isaacsim [options]

Start position : read from TF (map → <robot_frame>)
Goal  position : from RViz2 "2D Goal Pose" (/goal_pose topic)
Goal Z (layer) : --goal_layer <int>   layer index  (0 = ground floor)
              OR --goal_z     <float> height in meters (auto-converted to layer)
"""

import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

# ── Argument parsing ────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description='PCT global planner with live pose and RViz2 goal')
parser.add_argument('--scene',        type=str,   default='Isaacsim',
                    help='Scene name (determines tomogram file). Available: Spiral, Building, Plaza, Isaacsim, Openmind')
parser.add_argument('--tomo_file',    type=str,   default=None,
                    help='Override tomogram filename (without .pickle). Default: derived from scene.')
parser.add_argument('--goal_layer',   type=int,   default=0,
                    help='Goal layer index (0 = ground floor). Used when --goal_z is not set.')
parser.add_argument('--goal_z',       type=float, default=None,
                    help='Goal height in METERS. Auto-converted to layer index. Overrides --goal_layer.')
parser.add_argument('--start_layer',  type=int,   default=0,
                    help='Start layer index. If --start_z is provided, this is ignored.')
parser.add_argument('--start_z',      type=float, default=None,
                    help='Override start height in METERS (auto-converted to layer). Default: use TF Z.')
parser.add_argument('--robot_frame',  type=str,   default='base_link',
                    help='TF frame of the robot base (default: base_link)')
parser.add_argument('--map_frame',    type=str,   default='map',
                    help='TF map frame (default: map)')
parser.add_argument('--goal_topic',   type=str,   default='/goal_pose',
                    help='Topic for 2D goal pose from RViz2 (default: /goal_pose)')
parser.add_argument('--pose_topic',   type=str,   default='/localization',
                    help='Fallback pose topic (nav_msgs/Odometry in map frame) used when TF lookup fails. '
                         'Default: /localization (published by fast_lio_localization_ros2)')
args, _ = parser.parse_known_args()

# ── Scene → tomogram file map ───────────────────────────────────────────────
SCENE_TOMO = {
    'Spiral':    'spiral0.3_2',
    'Building':  'building2_9',
    'Plaza':     'plaza3_10',
    'Isaacsim':  '3dfactory',
    'Openmind':  'scans_20260708_140737_ds37',
}

cfg = Config()
tomo_file = args.tomo_file or SCENE_TOMO.get(args.scene, args.scene.lower())


def z_to_layer(z: float, slice_h0: float, slice_dh: float, n_slice: int) -> int:
    """Convert a world Z height (meters) to the closest layer index.

    slice_h0 = points_min_z + slice_dh  (bottom of first slice + one step)
    Layer i covers [ points_min_z + i*slice_dh,  points_min_z + (i+1)*slice_dh )
    """
    points_min_z = slice_h0 - slice_dh
    layer = int(np.floor((z - points_min_z) / slice_dh))
    return int(np.clip(layer, 0, n_slice - 1))


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('pct_global_planner')

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.path_pub = self.create_publisher(Path, '/pct_path', latched_qos)
        self.planner  = TomogramPlanner(cfg)

        # TF listener for robot pose
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Fallback: subscribe to /localization (nav_msgs/Odometry in map frame)
        # Published by fast_lio_localization_ros2 when TF map→base_link is unavailable
        self._latest_odom: Odometry = None
        self.odom_sub = self.create_subscription(
            Odometry,
            args.pose_topic,
            lambda msg: setattr(self, '_latest_odom', msg),
            10
        )

        # Load tomogram once at startup
        self.get_logger().info(f'Loading tomogram: {tomo_file}')
        self.planner.loadTomogram(tomo_file)
        self.get_logger().info(
            f'Tomogram loaded — {self.planner.n_slice} layer(s), '
            f'resolution={self.planner.resolution}m, '
            f'slice_h0={self.planner.slice_h0:.2f}m, '
            f'slice_dh={self.planner.slice_dh:.2f}m'
        )

        # Subscribe to RViz2 "2D Goal Pose"
        self.goal_sub = self.create_subscription(
            PoseStamped,
            args.goal_topic,
            self._goal_callback,
            10
        )
        self.get_logger().info(f'Waiting for goal on [{args.goal_topic}] ...')
        self.get_logger().info(f'Use RViz2 → "2D Goal Pose" tool to set the goal.')

    # ── Goal callback ────────────────────────────────────────────────────────
    def _goal_callback(self, msg: PoseStamped):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        # ── Resolve goal layer ───────────────────────────────────────────────
        if args.goal_z is not None:
            goal_layer = z_to_layer(
                args.goal_z,
                self.planner.slice_h0,
                self.planner.slice_dh,
                self.planner.n_slice
            )
            self.get_logger().info(
                f'goal_z={args.goal_z:.2f}m  →  layer {goal_layer}'
            )
        else:
            goal_layer = max(0, min(args.goal_layer, self.planner.n_slice - 1))

        # ── Get robot position from TF ───────────────────────────────────────
        start_pos, start_layer, robot_z = self._get_robot_pose()
        if start_pos is None:
            self.get_logger().error(
                f'Cannot look up TF {args.map_frame} → {args.robot_frame}. '
                'Is localization running?'
            )
            return

        end_pos = np.array([goal_x, goal_y], dtype=np.float32)

        self.get_logger().info(
            f'Planning:  start={start_pos} layer={start_layer} (robot Z={robot_z:.2f}m)'
            f'  →  goal={end_pos} layer={goal_layer}'
        )

        # ── Run planner ──────────────────────────────────────────────────────
        traj_3d = self.planner.plan(start_pos, end_pos, start_layer, goal_layer)

        if traj_3d is None:
            self.get_logger().warn('Planner returned no path. Check start/goal are on traversable cells.')
            return

        path_msg = traj2ros(traj_3d)
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published with {len(path_msg.poses)} waypoints on /pct_path')

    # ── TF lookup ────────────────────────────────────────────────────────────
    def _get_robot_pose(self):
        """Return (start_pos [x,y], start_layer, robot_z) or (None, None, None) on failure.

        Priority:
          1. TF lookup: map → <robot_frame>
          2. Fallback: latest message on <pose_topic> (nav_msgs/Odometry in map frame)
             Published by fast_lio_localization_ros2 as /localization.
        """
        tx, ty, tz = None, None, None

        # ── 1. Try TF ────────────────────────────────────────────────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                args.map_frame,
                args.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        # ── 2. Fallback: /localization Odometry ──────────────────────────────
        if tx is None:
            if self._latest_odom is not None:
                p = self._latest_odom.pose.pose.position
                tx, ty, tz = p.x, p.y, p.z
                self.get_logger().info(
                    f'TF unavailable — using pose from [{args.pose_topic}]: '
                    f'x={tx:.2f} y={ty:.2f} z={tz:.2f}'
                )
            else:
                self.get_logger().error(
                    f'No robot pose available. '
                    f'TF [{args.map_frame}→{args.robot_frame}] not found and '
                    f'[{args.pose_topic}] has no messages yet.'
                )
                return None, None, None

        start_pos = np.array([tx, ty], dtype=np.float32)

        # Determine start layer
        robot_z_for_layer = args.start_z if args.start_z is not None else tz
        start_layer = z_to_layer(
            robot_z_for_layer,
            self.planner.slice_h0,
            self.planner.slice_dh,
            self.planner.n_slice
        )

        return start_pos, start_layer, tz


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    rclpy.init()
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
