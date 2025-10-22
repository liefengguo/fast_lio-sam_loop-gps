# fast_lio-sam_loop-gps

Incremental LiDAR-inertial mapping with loop closure and GPS fusion.

This fork keeps the FAST-LIO front-end but rewrites the map loader/back-end so you can:

- Resume mapping from a previous session by loading `pose_graph.g2o`, keyframe clouds, `raw_map.pcd`, and `origin.txt`.
- Lock historical poses while inserting new odometry/GPS/loop factors.
- Publish large preloaded maps asynchronously (no more startup/exit stalls).
- Guard ISAM2 updates against bad factors: failures are logged and skipped instead of aborting the process.

See [docs/old_map_integration.md](docs/old_map_integration.md) for the full design notes.

![System](fast_lio-sam_loop-gps.png)

---

## 1. Environment

| Component | Notes |
|-----------|-------|
| Ubuntu | 18.04 or 20.04 |
| ROS | Melodic / Noetic (catkin workspace) |
| GCC / G++ | ≥ 7.5 |
| PCL | ≥ 1.8 |
| Eigen | ≥ 3.3.3 |
| Livox ROS Driver | [Livox-SDK/livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) |
| GTSAM | 4.0.0 |
| libgeographic | `sudo apt install libgeographic-dev` |
| Optional | `rviz_satellite` for background imagery |

> **Reminder:** source the Livox driver before building or running, otherwise custom messages are missing.

---

## 2. Workspace Setup

```bash
mkdir -p ~/gps_mapping_ws/src
cd ~/gps_mapping_ws/src

git clone https://github.com/liefengguo/fast_lio-sam_loop-gps.git fast_lio-sam_loop-gps
cd fast_lio-sam_loop-gps
git submodule update --init

cd ~/gps_mapping_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

If you prefer `catkin build`, replace the last two lines with `catkin build fast_lio_sam_loop`.

---

## 3. Data Preparation

1. **Bags / live data**  
   Works with Livox or Velodyne + IMU. You can reuse the [LIO-SAM-6axis VLP-16 bags](https://github.com/JokerJohn/LIO_SAM_6AXIS) for quick tests.

2. **Map directory (for incremental mode)**  
   ```
   Map/
     pose_graph.g2o
     raw_map.pcd
     origin.txt
     pcd_buffer/
       0.pcd
       1.pcd
       ...
   ```
   These files are produced by the built-in save services (§6). When launching in incremental mode, the loader mirrors this directory to the runtime workspace.

3. **Configuration**  
   Edit `config/guangzhou_port_gps.yaml` (or your own copy) to match topics, extrinsics, GPS parameters, etc.

---

## 4. Running the System

### 4.1 Fresh Mapping

```bash
roslaunch fast_lio_sam_loop mapping_guangzhou_port_gps.launch \
    use_sim_time:=true rviz:=true

# in another terminal
rosbag play YOUR_DATASET.bag --clock
```

Set `use_sim_time:=false` for live sensors.

### 4.2 Incremental Mapping (continue from previous map)

```bash
roslaunch fast_lio_sam_loop mapping_guangzhou_port_gps_inc.launch \
    old_map:=/absolute/path/to/Map/ \
    use_sim_time:=true rviz:=true
```

- `old_map` is forwarded as the first CLI argument to `laserMapping`.  
- The node loads `pose_graph.g2o`, all keyframe clouds, `raw_map.pcd`, and `origin.txt`.  
- The first new keyframe receives an identity prior so the previous map is locked in place.  
- If `origin.txt` exists, `update_initial_guess()` reuses the stored ENU origin; otherwise it falls back to GNSS-based initialization.

### 4.3 Minimal command-line run

```bash
rosrun fast_lio_sam_loop fastlio_mapping /path/to/Map/
```

Omit the map argument for a fresh session.

---

## 5. Incremental Mapping Workflow

1. Run a **fresh session**, then export the map (see §6).  
2. Copy the resulting `Map/` folder to a persistent location (e.g. `/data/guangzhou_port/Map`).  
3. Launch incremental mode with `old_map:=/data/guangzhou_port/Map`.  
4. Play the new bag or drive the robot—new keyframes are optimized relative to the locked prior map.  
5. Inspect topics:
   - `/mapping`: accumulated map (down-sampled + raw)
   - `/map_path`: global trajectory
   - `/LidarOdometry`: corrected odometry pose
6. Optionally re-export the updated map when finished.

Troubleshooting highlights:

| Symptom | Cause | Remedy |
|---------|-------|--------|
| Startup/exit stalls | Large `raw_map.pcd` published on main thread | Fixed: async publisher (keep repo updated) |
| ENU origin jumps | `origin.txt` missing or inconsistent | Ensure it exists; or re-export map after session |
| ISAM crash in release | Degenerate factors | Errors are logged; inspect `/Log` for offending factors |

---

## 6. Exporting Maps & Trajectories

### Save map (PCD + pose graph)
```bash
rosservice call /service/save_map "destination: 'session_2025_04_07' resolution: 0.2"
```
- `destination` is relative to `savePCDDirectory` (absolute paths are also accepted).
- `resolution` controls voxel down-sampling; set to `0` to keep the default mapping leaf size.

The directory structure matches §3.2.

### Save poses (KITTI-style txt)
```bash
rosservice call /service/save_pose "destination: 'session_2025_04_07'"
```
Generates `optimized_pose.txt`, `without_optimized_pose.txt`, and `gnss_pose.txt`.

Archive the map directory together with the YAML config you used for full reproducibility.

---

## 7. Frequently Tuned Parameters

| Parameter | Location | Description |
|-----------|----------|-------------|
| `old_map` | `launch/mapping_guangzhou_port_gps_inc.launch` | Path to prior map for incremental mode |
| `use_sim_time` | Launch args | Enable `/clock` sync during rosbag replay |
| `gpc_factor_init_num` | `config/guangzhou_port_gps.yaml` | Number of GPS factors to accumulate before inserting into ISAM |
| `correct_fe_en` | YAML | Rebuild ikd-tree map after loop closure when `true` |
| `savePCDDirectory` | YAML / param server | Base directory for export services |

Refer to [docs/old_map_integration.md](docs/old_map_integration.md) for detailed explanations of the incremental variables (`old_map_keyframe_count`, `origin_loaded_from_map`, etc.).

---

## 8. Credits

This project builds upon the outstanding work from:

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [LIO-SAM-6axis](https://github.com/JokerJohn/LIO_SAM_6AXIS)

Please consider citing or starring those repositories—they laid the foundation that made incremental GPS-assisted FAST-LIO possible.
