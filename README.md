# PCT Planner

## Overview

This is an implementation of paper **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** (accepted by TMECH).
It provides a highly efficient and extensible global navigation framework based on a tomographic understanding of the environment to navigate ground robots in multi-layer structures.

**Demonstrations**: [pct_planner](https://byangw.github.io/projects/tmech2024/)

![demo](rsc/docs/demo.png)

## Citing

If you use PCT Planner, please cite the following paper:

[Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography](https://ieeexplore.ieee.org/document/10531813)

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography}, 
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

## Prerequisites

### Environment

- Ubuntu >= 20.04
- ROS2 Humble with desktop-full installation (`ros-humble-desktop`)
- CUDA >= 11.7

### Python

- Python >= 3.10 (bundled with Ubuntu 22.04)
- [CuPy](https://docs.cupy.dev/en/stable/install.html) matching your CUDA version
- Open3d

## Setup

### 1. Source ROS2 Humble

Add this to your `~/.bashrc` (run once):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Create Virtual Environment

ROS2 Humble requires **Python 3.10**. The venv must use `--system-site-packages` so that ROS2 Python packages (`rclpy`, `sensor_msgs_py`, etc.) remain accessible:

```bash
python3.10 -m venv pct_env --system-site-packages
source pct_env/bin/activate
```

To activate the environment in future sessions:

```bash
source pct_env/bin/activate
```

### 3. Install Python Dependencies

```bash
pip install open3d "numpy<2"
# Install CuPy matching your CUDA version (check with: nvidia-smi)
# CUDA 12.x:
# pip install cupy-cuda12x
# CUDA 13.x:
pip install cupy-cuda13x
```

The planner native modules currently require **NumPy 1.x**. If you installed NumPy 2.x, downgrade first:

```bash
pip install "numpy==1.26.4"
```

## Build & Install

Inside the package, there are two modules: the point cloud tomography module for tomogram reconstruction (in **tomography/**) and the planner module for path planning and optimization (in **planner/**).
You only need to build the planner module before use.
In **planner/**, run **build_thirdparty.sh** first and then run **build.sh**. 

```bash
cd planner/
./build_thirdparty.sh
./build.sh
```

## Run Examples

Three example scenarios are provided: **"Spiral"**, **"Building"**, and **"Plaza"**.
- **"Spiral"**: A spiral overpass scenario released in the [3D2M planner](https://github.com/ZJU-FAST-Lab/3D2M-planner).
- **"Building"**: A multi-layer indoor scenario with various stairs, slopes, overhangs and obstacles.
- **"Plaza"**: A complex outdoor plaza for repeated trajectory generation evaluation.

### Tomogram Construction

To plan in a scenario, first you need to construct the scene tomogram using the pcd file.
- Unzip the pcd files in **rsc/pcd/pcd_files.zip** to **rsc/pcd/**.
- For scene **"Spiral"**, you can download the pcd file from [3D2M planner spiral0.3_2.pcd](https://github.com/ZJU-FAST-Lab/3D2M-planner/tree/main/planner/src/read_pcd/PCDFiles).
- Start **RViz2** with the provided config:

```bash
rviz2 -d rsc/rviz/pct_ros.rviz
```

- Activate your virtual environment and, in **tomography/scripts/**, run **tomography.py** with the **--scene** argument:

```bash
source pct_env/bin/activate
cd tomography/scripts/
python3 tomography.py --scene Spiral
python3 tomography.py --scene Isaacsim
```

- The generated tomogram is visualized as ROS2 PointCloud2 message in RViz2 and saved in **rsc/tomogram/**.

### Trajectory Generation 

After the tomogram is constructed, you can run the trajectory generation example.
- In **planner/scripts/**, run **plan.py** with the **--scene** argument:

```bash
source pct_env/bin/activate
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/YOUR/DIRECTORY/TO/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:/YOUR/DIRECTORY/TO/PCT_planner/planner/lib/build/src/common/smoothing
cd planner/scripts/
python3 plan.py --scene Spiral


export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/wendy/Documents/GitHub/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:/home/wendy/Documents/GitHub/PCT_planner/planner/lib/build/src/common/smoothing
cd planner/scripts/
python3 plan.py --scene Isaacsim
```

- The generated trajectory is visualized as ROS2 Path message in RViz2.

### Live Global Planning (robot pose + RViz2 goal)

`plan_global.py` subscribes to the robot's live localization (via TF `map → base_link`) as the start position and to RViz2's **"2D Goal Pose"** tool as the goal position, then publishes the planned path to `/pct_path`.

```bash
source pct_env/bin/activate
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/wendy/Documents/GitHub/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:/home/wendy/Documents/GitHub/PCT_planner/planner/lib/build/src/common/smoothing
cd planner/scripts/

# Goal layer by index (0 = ground floor)
python3 plan_global.py --scene Isaacsim --goal_layer 0

# Goal layer by height in meters (auto-converted to layer)
python3 plan_global.py --scene Isaacsim --goal_z 1.5


# Tomogram & Plan together
python3 planner_wrapper.py --scene Isaacsim --goal_layer 0

```
#### Arguments

| Argument | Default | Description |
|---|---|---|
| `--scene` | `Isaacsim` | Scene name (selects tomogram file) |
| `--tomo_file` | *(from scene)* | Override tomogram filename (without `.pickle`) |
| `--goal_layer` | `0` | Goal **layer index** (0 = ground floor) |
| `--goal_z` | *(not set)* | Goal height in **meters** — auto-converted to layer, overrides `--goal_layer` |
| `--start_layer` | `0` | Fallback start layer if TF Z is unavailable |
| `--start_z` | *(from TF)* | Override start height in meters instead of using TF Z |
| `--robot_frame` | `base_link` | TF frame of the robot base |
| `--map_frame` | `map` | TF map frame |
| `--goal_topic` | `/goal_pose` | RViz2 "2D Goal Pose" topic |
| `--pose_topic` | `/localization` | Fallback pose topic (`nav_msgs/Odometry` in map frame) used when TF lookup fails |

#### Notes

- `--goal_layer` uses a **layer index** (integer). To find how many layers your tomogram has, check the terminal output of `tomography.py` for `Num slices simp: N`.
- `--goal_z` uses **meters** in the map frame. The node converts it to the nearest layer using `slice_h0` and `slice_dh` from the tomogram.
- A new path is replanned every time you set a new "2D Goal Pose" in RViz2.

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).

## Quick Start (ROS2 Humble)

Open **three terminals**. In each, first run:

```bash
source ~/.bashrc
source /path/to/PCT_planner/pct_env/bin/activate
```

### Terminal 1 — Tomogram Construction

```bash
cd tomography/scripts/
python3 tomography.py --scene Spiral
```

### Terminal 2 — RViz2 Visualization

```bash
cd /path/to/PCT_planner
rviz2 -d rsc/rviz/pct_ros.rviz
```

### Terminal 3 — Trajectory Planning

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:/path/to/PCT_planner/planner/lib/build/src/common/smoothing
cd planner/scripts/
python3 plan.py --scene Spiral

python3 plan.py --scene Isaacsim
```

## Configuration

### Planner Input (`planner/scripts/plan.py`)

| Parameter | Description |
|---|---|
| `start_pos` | Start position in meters `[x, y]` (map frame) |
| `end_pos` | Goal position in meters `[x, y]` (map frame) |
| `start_layer` | Start layer index (0 = ground floor) |
| `end_layer` | Goal layer index |

The Z coordinate (height) is automatically determined by the tomogram layer. You only specify the 2D position and which layer to start/end on.

### Tomogram Slicing (`tomography/config/scene_*.py`)

| Parameter | Description | Default |
|---|---|---|
| `map.resolution` | Grid cell size in meters | `0.20` |
| `map.slice_dh` | Height per initial slice (meters) | `0.5` |
| `map.ground_h` | Ground plane height | `0.0` |

The number of **final layers** is automatically determined by layer simplification: initial slices that don't contain unique traversable surfaces are merged. Smaller `slice_dh` → more initial slices → potentially more final layers.

### Traversability Parameters (`tomography/config/scene_*.py`)

| Parameter | Description |
|---|---|
| `trav.slope_max` | Maximum traversable slope (radians) |
| `trav.step_max` | Maximum step height (meters) |
| `trav.safe_margin` | Obstacle clearance distance (meters) |
| `trav.inflation` | Cost inflation radius (meters) |

## Troubleshooting

| Issue | Fix |
|---|---|
| `cudaErrorUnknown` / `cuInit() = 999` | GPU driver in bad state. Run `sudo reboot` |
| CycloneDDS `enp130s0 does not match` | Set `CYCLONEDDS_URI` in `~/.bashrc` to use loopback: `export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'` |
| RViz2 shows empty topics | Start RViz2 **before** or at the same time as the publisher nodes (QoS: TRANSIENT_LOCAL) |
| `Segmentation fault` in planner | Ensure `libcommon_smoothing.so` path is in `LD_LIBRARY_PATH` |
| `source ~/.bashrc` deactivates venv | Re-activate with `source pct_env/bin/activate` after sourcing bashrc |
| `Segmentation fault` or `RuntimeError` around `pickle` / `init_map` | Use `numpy==1.26.4`, rebuild `planner/`, and regenerate the tomogram pickle with `tomography.py`. Old NumPy 2 generated pickles are not safe to load in the planner. |