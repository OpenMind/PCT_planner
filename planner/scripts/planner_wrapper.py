import os
import sys
import pickle
import argparse
import numpy as np

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


def _require_supported_numpy_version():
    if np.lib.NumpyVersion(np.__version__) >= '2.0.0':
        raise RuntimeError(
            'The planner native modules crash with NumPy 2.x. '
            'Install numpy<2 in pct_env, rebuild planner/, and rerun plan.py.'
        )


def _validate_tomogram_pickle_compat(tomo_path):
    if np.lib.NumpyVersion(np.__version__) < '2.0.0':
        with open(tomo_path, 'rb') as handle:
            if b'numpy._core' in handle.read(4096):
                raise RuntimeError(
                    'Tomogram pickle was generated with NumPy 2.x and cannot '
                    'be safely loaded under NumPy 1.x. Regenerate it with '
                    'tomography.py after downgrading numpy in pct_env.'
                )


class TomogramPlanner(object):
    def __init__(self, cfg):
        _require_supported_numpy_version()
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)
        self.cost_barrier = 50.0  # cells with trav >= cost_barrier are obstacles

    def loadTomogram(self, tomo_file):
        tomo_path = self.tomo_dir + tomo_file + '.pickle'
        _validate_tomogram_pickle_compat(tomo_path)
        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.trav = trav  # [n_slice, map_dim_x, map_dim_y] — kept for traversability checks

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos, start_layer=0, end_layer=0):
        start_idx_2d = self.pos2idx(start_pos)
        end_idx_2d   = self.pos2idx(end_pos)

        # ── Diagnostics ──────────────────────────────────────────────────────
        print(f'[PCT] Map center : {self.center}')
        print(f'[PCT] Map dim    : {self.map_dim}  resolution={self.resolution}m')
        print(f'[PCT] Start pos  : {start_pos}  → grid idx {start_idx_2d}  layer={start_layer}')
        print(f'[PCT] End   pos  : {end_pos}  → grid idx {end_idx_2d}  layer={end_layer}')

        # ── Snap start/end to nearest traversable cell if needed ─────────────
        start_idx_2d = self._snap_to_traversable(start_idx_2d, start_layer, label='start')
        end_idx_2d   = self._snap_to_traversable(end_idx_2d,   end_layer,   label='end')

        self.start_idx[0] = start_layer
        self.start_idx[1:] = start_idx_2d
        self.end_idx[0] = end_layer
        self.end_idx[1:] = end_idx_2d

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d
    
    def _is_traversable(self, idx_2d, layer):
        """Return True if the grid cell is traversable (cost < cost_barrier).
        idx_2d = [y_idx, x_idx] as returned by pos2idx.
        trav values: 0 <= v < cost_barrier = traversable, v >= cost_barrier = obstacle.
        Note: 0.0 is a legitimate "best case" cost (flat, clear ground), not just
        an "unscanned" sentinel — cells with no real data are excluded upstream
        via elev_g/NaN before ever reaching this array.
        """
        y_idx, x_idx = int(round(idx_2d[0])), int(round(idx_2d[1]))
        # bounds: y_idx in [0, map_dim[1]), x_idx in [0, map_dim[0])
        if not (0 <= y_idx < self.map_dim[1] and 0 <= x_idx < self.map_dim[0]):
            return False
        v = float(self.trav[layer, x_idx, y_idx])
        return 0.0 <= v < self.cost_barrier

    def _snap_to_traversable(self, idx_2d, layer, search_radius_m=5.0, label='point'):
        """
        If idx_2d is not traversable, search outward (up to search_radius_m)
        and return the nearest traversable cell index.  Prints a warning if snapping.
        idx_2d = [y_idx, x_idx] as returned by pos2idx.
        """
        if self._is_traversable(idx_2d, layer):
            return idx_2d

        y0, x0 = int(round(idx_2d[0])), int(round(idx_2d[1]))
        max_steps = int(search_radius_m / self.resolution)
        print(f'[PCT] WARNING: {label} cell y={y0},x={x0} layer={layer} is not traversable. '
              f'Searching within {search_radius_m}m ...')

        best = None
        best_dist = float('inf')
        for dy in range(-max_steps, max_steps + 1):
            for dx in range(-max_steps, max_steps + 1):
                ny, nx = y0 + dy, x0 + dx
                if not (0 <= ny < self.map_dim[1] and 0 <= nx < self.map_dim[0]):
                    continue
                if 0.0 <= float(self.trav[layer, nx, ny]) < self.cost_barrier:
                    dist = dy * dy + dx * dx
                    if dist < best_dist:
                        best_dist = dist
                        best = np.array([ny, nx], dtype=np.float32)

        if best is None:
            print(f'[PCT] ERROR: No traversable cell found near {label} within {search_radius_m}m.')
            return idx_2d  # return original; planner will report failure

        snapped_pos = self.idx2pos(best)
        print(f'[PCT] Snapped {label} to y={int(best[0])},x={int(best[1])} '
              f'≈ world pos {snapped_pos}  (dist={best_dist**0.5 * self.resolution:.2f}m)')
        return best

    def find_reachable_area(self, idx_2d, layer):
        """BFS flood-fill from idx_2d to find all traversable cells in the same connected component.
        Returns (world_positions, world_bbox) where world_bbox = (x_min, x_max, y_min, y_max).
        """
        from collections import deque
        y0, x0 = int(round(idx_2d[0])), int(round(idx_2d[1]))
        visited = set()
        queue = deque([(y0, x0)])
        visited.add((y0, x0))
        while queue:
            y, x = queue.popleft()
            for dy, dx in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                ny, nx = y + dy, x + dx
                if (ny, nx) in visited:
                    continue
                if not (0 <= ny < self.map_dim[1] and 0 <= nx < self.map_dim[0]):
                    continue
                if 0.0 <= float(self.trav[layer, nx, ny]) < self.cost_barrier:
                    visited.add((ny, nx))
                    queue.append((ny, nx))
        if not visited:
            return [], None
        xs = np.array([nx for (_, nx) in visited], dtype=np.float32)
        ys = np.array([ny for (ny, _) in visited], dtype=np.float32)
        x_world = (xs - self.offset[0]) * self.resolution + self.center[0]
        y_world = (ys - self.offset[1]) * self.resolution + self.center[1]
        bbox = (float(x_world.min()), float(x_world.max()),
                float(y_world.min()), float(y_world.max()))
        return list(visited), bbox

    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx

    def idx2pos(self, idx):
        """Inverse of pos2idx: convert [y_idx, x_idx] back to world [x, y]."""
        y_idx, x_idx = idx[0], idx[1]
        x = (x_idx - self.offset[0]) * self.resolution + self.center[0]
        y = (y_idx - self.offset[1]) * self.resolution + self.center[1]
        return np.array([x, y], dtype=np.float32)


# ── Combined "tomography + global planner" entrypoint ────────────────────────
#
# Running this file directly is equivalent to running, in order:
#   python3 tomography.py --scene <scene>
#   python3 plan_global.py --scene <scene> --goal_layer <goal_layer>
# in a single process, so the tomography visualization topics (/layer_G_*,
# /layer_C_*, /tomogram) stay alive in RViz2 at the same time as the live
# planner. See _load_tomography_scene() for why the tomography config package
# is imported in isolation instead of the usual `sys.path.append('../')`
# convention: both tomography/config and planner/config are importable as a
# bare `config` module, and Python only lets one module hold that name.

def _load_tomography_scene(scene_name):
    """Import tomography's Tomography class + Scene<name> config in isolation.

    tomography/config and planner/config are both importable as a bare
    `config` module. This process never needs planner/config as a module
    (TomogramPlanner takes its cfg as a plain constructor argument, built
    inline in main() below), so it is safe to claim the `config` name for
    tomography's package here without a later import clobbering it.
    """
    tomography_root = os.path.join(rsg_root, 'tomography')
    tomography_scripts = os.path.join(tomography_root, 'scripts')
    for p in (tomography_root, tomography_scripts):
        if p not in sys.path:
            sys.path.insert(0, p)

    import config as tomo_config_pkg
    from tomography import Tomography

    scene_cfg = getattr(tomo_config_pkg, 'Scene' + scene_name, None)
    if scene_cfg is None:
        raise ValueError(
            f'Unknown scene "{scene_name}" - no Scene{scene_name} in tomography/config.'
        )
    return Tomography, tomo_config_pkg.Config(), scene_cfg


def _build_planner_cfg():
    """Minimal stand-in for planner/config.Config(), built inline to avoid
    the `config` module-name collision with tomography/config (see above)."""
    class _ConfigPlanner:
        use_quintic = True
        max_heading_rate = 10

    class _ConfigWrapper:
        tomo_dir = '/rsc/tomogram/'

    class _Cfg:
        planner = _ConfigPlanner()
        wrapper = _ConfigWrapper()

    return _Cfg()


def z_to_layer(z, slice_h0, slice_dh, n_slice):
    """Convert a world Z height (meters) to the closest layer index."""
    points_min_z = slice_h0 - slice_dh
    layer = int(np.floor((z - points_min_z) / slice_dh))
    return int(np.clip(layer, 0, n_slice - 1))


class CombinedPlannerNode(Node):
    """Live RViz2-goal -> PCT path planner, driven by an already-loaded
    TomogramPlanner. Mirrors plan_global.py's GlobalPlannerNode, minus the
    tomogram loading (the caller already ran tomography + loadTomogram)."""

    def __init__(self, planner, args):
        super().__init__('pct_global_planner')
        self.planner = planner
        self.args = args

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.path_pub = self.create_publisher(Path, '/pct_path', latched_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped, args.goal_topic, self._goal_callback, 10
        )
        self.get_logger().info(f'Waiting for goal on [{args.goal_topic}] ...')
        self.get_logger().info('Use RViz2 -> "2D Goal Pose" tool to set the goal.')

    def _goal_callback(self, msg):
        args = self.args
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        if args.goal_z is not None:
            goal_layer = z_to_layer(
                args.goal_z, self.planner.slice_h0, self.planner.slice_dh, self.planner.n_slice
            )
        else:
            goal_layer = max(0, min(args.goal_layer, self.planner.n_slice - 1))

        start_pos, start_layer = self._get_robot_pose()
        if start_pos is None:
            self.get_logger().error(
                f'Cannot look up TF {args.map_frame} -> {args.robot_frame}. Is localization running?'
            )
            return

        end_pos = np.array([goal_x, goal_y], dtype=np.float32)
        self.get_logger().info(
            f'Planning: start={start_pos} layer={start_layer} -> goal={end_pos} layer={goal_layer}'
        )

        traj_3d = self.planner.plan(start_pos, end_pos, start_layer, goal_layer)
        if traj_3d is None:
            self.get_logger().warn('Planner returned no path. Check start/goal are on traversable cells.')
            return

        path_msg = traj2ros(traj_3d)
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published with {len(path_msg.poses)} waypoints on /pct_path')

    def _get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.robot_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

        start_pos = np.array(
            [tf.transform.translation.x, tf.transform.translation.y], dtype=np.float32
        )
        start_layer = z_to_layer(
            self.args.start_z if self.args.start_z is not None else tf.transform.translation.z,
            self.planner.slice_h0, self.planner.slice_dh, self.planner.n_slice
        )
        return start_pos, start_layer


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Generate the tomogram and run the PCT global planner in one process.'
    )
    parser.add_argument('--scene', type=str, default='Isaacsim',
                        help='Scene name, e.g. Isaacsim, Spiral, Building, Plaza, Openmind')
    parser.add_argument('--pcd', type=str, default=None,
                        help='Path to a PCD file, overriding the scene config\'s pcd.file_name. '
                             'Absolute paths are used as-is; relative paths are resolved against rsc/pcd/.')
    parser.add_argument('--goal_layer', type=int, default=0)
    parser.add_argument('--goal_z', type=float, default=None)
    parser.add_argument('--start_layer', type=int, default=0)
    parser.add_argument('--start_z', type=float, default=None)
    parser.add_argument('--robot_frame', type=str, default='base_link')
    parser.add_argument('--map_frame', type=str, default='map')
    parser.add_argument('--goal_topic', type=str, default='/goal_pose')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = _parse_args()
    rclpy.init()

    # 1. Tomography step - equivalent to `python3 tomography.py --scene <scene>`.
    #    Loads the scene's PCD, computes the tomogram, exports the pickle, and
    #    starts publishing /layer_G_*, /layer_C_*, /tomogram for RViz2.
    Tomography, tomo_cfg, scene_cfg = _load_tomography_scene(args.scene)
    if args.pcd is not None:
        scene_cfg.pcd.file_name = args.pcd
    tomography_node = Tomography(tomo_cfg, scene_cfg)

    # 2. Planning step - equivalent to `python3 plan_global.py --scene <scene>
    #    --goal_layer <goal_layer>`. tomo_file is derived the same way
    #    tomography.py names its export, so it always matches what was just
    #    generated above (no separate scene->tomo_file table to go stale).
    tomo_file = os.path.splitext(os.path.basename(scene_cfg.pcd.file_name))[0]
    planner = TomogramPlanner(_build_planner_cfg())
    planner.loadTomogram(tomo_file)
    planner_node = CombinedPlannerNode(planner, args)

    executor = MultiThreadedExecutor()
    executor.add_node(tomography_node)
    executor.add_node(planner_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        tomography_node.destroy_node()
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()