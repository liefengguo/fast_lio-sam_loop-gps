# Old Map Integration Notes

> 目标：支持在启动时加载已有地图，并把旧关键帧与新的增量建图逻辑区分开，同时让传感器/回环模块按需复用这些历史数据。

## 1. 读取 pose_graph.g2o 的新流程

### 做了什么
- `LoadMap_gtsam()` 使用 `gtsam::readG2o()` 读出完整的 `NonlinearFactorGraph` 与 `Values`：
  ```c++
  boost::tie(graph_ptr, values_ptr) = gtsam::readG2o(pose_graph_path.string(), true);
  gtsam::NonlinearFactorGraph graph = *graph_ptr;
  gtsam::Values values = *values_ptr;
  ```
- 解析所有位姿节点，把姿态写回 `cloud_key_poses_3D/6D`，并同步副本：
  ```c++
  for (size_t idx = 0; idx < keys.size(); ++idx) {
      const gtsam::Pose3 &pose = values.at<gtsam::Pose3>(key);
      copy_pose_to_clouds(pose, idx);
  }
  *copy_cloud_key_poses_3D = *cloud_key_poses_3D;
  *copy_cloud_key_poses_6D = *cloud_key_poses_6D;
  ```
- 读取 `EDGE_DIS:VEC3` 行，构造 `gtsam::GPSFactor`，补充进运行时的 GPS 因子集合与索引表：
  ```c++
  auto records = parse_gps_edges(pose_graph_path);
  for (size_t i = 0; i < records.size(); ++i) {
      keyframeGPSfactor.push_back(records[i].factor);
      gps_index_container[records[i].factor.key()] = static_cast<int>(i);
      cloudKeyGPSPoses3D->push_back(records[i].measurement);
      graph.add(records[i].factor);
  }
  ```
- 将旧图的因子与初值推入 `ISAM2`，并用 `Constrained` 噪声固定旧节点：
  ```c++
  isam->update(graph, values);
  gtsam::NonlinearFactorGraph locking_factors;
  const auto lock_noise = gtsam::noiseModel::Constrained::All(6);
  locking_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, values.at<gtsam::Pose3>(key), lock_noise));
  isam->update(locking_factors, gtsam::Values());
  ```

### 为什么要这样做
- 读取 `g2o` 文件的全部图信息才能保持关键帧 id 连续，否则后端会因为缺少节点顺序而无法回放旧轨迹，例如 `keys` 中缺少 0 号节点会让 `loop_Index_container` 的初始化失真。
- 在写回 `cloud_key_poses_*` 后，`publish_global_map()` 能立即用这些点生成 KD-Tree；若跳过这一步，增量启动时局部地图为空，首帧会出现“附近没有关键帧，局部地图为空”的警告。
- GPS 因子保留历史索引可避免重复添加旧观测，尤其在增量模式下，如果不记录 `gps_index_container`，`add_gps_factor()` 会再次把旧图的 GPS 点写入 ISAM，导致权重失衡。
- 给旧节点加约束噪声几乎为零的先验（`Constrained::All`) 可以让旧图保持不动。例如在广州港地图中，如果不锁定历史节点，加载后第一帧 GNSS 会把旧轨迹整体平移，破坏原有地图基准。

## 2. 加载旧关键帧点云与原始地图

### 做了什么
- 遍历 `Map/pcd_buffer/*.pcd`，加载旧关键帧点云：
  ```c++
  const bfs::path file_path = buffer_dir / (std::to_string(idx) + ".pcd");
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  pcl::io::loadPCDFile(file_path.string(), *cloud);
  surf_cloud_keyframes.push_back(cloud);
  ```
- 读取 `raw_map.pcd` 并设置发布标记：
  ```c++
  pcl::io::loadPCDFile(raw_path.string(), *preloaded_raw_map);
  raw_map_needs_publish = !preloaded_raw_map->empty();
  ```
- 在主程序初始化 publisher 后调用 `PublishPreloadedRawMap()`：
  ```c++
  pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("mapping", 1);
  PublishPreloadedRawMap();
  ```
- `publish_global_map()` 每次执行前会尝试触发一次原始地图发布：
  ```c++
  const bool raw_published_now = PublishPreloadedRawMap();
  if (pubLaserCloudSurround.getNumSubscribers() == 0 && !compute_full_map) {
      if (!raw_published_now) return;
  }
  ```

### 为什么要这样做
- 旧关键帧点云能够让增量模式在第一帧就有历史地标。例如广州港测试地图，加载点云后 `loop_find_near_keyframes()` 能立即找到邻域点，否则第一帧会因为 `surf_cloud_keyframes` 为空而退化成纯里程计。
- `raw_map.pcd` 提供稠密点云，可在 RViz 中直接渲染旧地图。如果不自动发布，用户需要手动回放才看得见历史地图，影响现场部署效率。
- 只有在存在订阅者时才发布 raw map，可以避免在后台运行时发送无用大包体。比如离线回放时无人订阅 `mapping` 话题，此逻辑即可跳过 raw map 发布，节省网络带宽。

## 3. 回环索引的管理策略

### 做了什么
- `loop_Index_container` 在加载旧图后填充自环并记录历史回环：
  ```c++
  loop_Index_container.clear();
  for (size_t i = 0; i < old_map_keyframe_count; ++i)
      loop_Index_container[static_cast<int>(i)] = static_cast<int>(i);
  ```
  ```c++
  if (auto between = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
      const int newer = std::max(key1, key2);
      const int older = std::min(key1, key2);
      loop_Index_container[newer] = older;
  }
  ```
- 初始化时清空队列，防止重复触发旧回环：
  ```c++
  loop_Index_queue.clear();
  loop_pose_queue.clear();
  loop_noise_queue.clear();
  ```
- 新增循环前先检查容器：
  ```c++
  if (loop_Index_container.find(loopKeyCur) != loop_Index_container.end())
      return false;
  ```
- 在 `visualize_loop_closure()` 中增加越界检查：
  ```c++
  if (key_cur < 0 || key_pre < 0) continue;
  if (key_cur >= copy_cloud_key_poses_6D->size()) continue;
  if (key_cur == key_pre) continue;
  ```

### 为什么要这样做
- 自环标记帮助识别旧节点。例如旧图有 200 帧，若不记录自环，增量回环检测会把 0～199 号节点当成“未处理节点”，导致 `perform_loop_closure()` 每帧都尝试匹配历史点云，浪费计算资源。
- 清空队列避免重复触发旧回环。否则 `loop_Index_queue` 中残留的旧元素会在 `add_loop_factor()` 中再次插入，造成回环边重复，优化权重异常。
- 越界检查可防止数据缺失导致崩溃。例如历史 `pcd_buffer` 缺少第 5 帧点云时，如果不判断 `surf_cloud_keyframes[keyNear]` 是否存在，`transformPointCloud` 会触发空指针异常。

## 4. 新旧关键帧区分与里程计因子的改写

### 做了什么
- 在 `add_odom_factor()` / `add_odom_factor_fastlio()` 内判断“第一帧新节点”：
  ```c++
  const bool first_new_keyframe = has_old_map && (pose_count == old_map_keyframe_count);
  if (first_new_keyframe) {
      gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(pose_count, gtsam::Pose3(), priorNoise));
      initial_estimate.insert(pose_count, gtsam::Pose3());
  }
  ```
- 对第一帧新节点添加单位先验而非旧姿态：
  ```c++
  poseTo = gtsam::Pose3(); // Identity
  ```
- `add_gps_factor()` 跳过旧节点：
  ```c++
  if (pose_count <= std::max<size_t>(old_map_keyframe_count, 1)) return;
  ```
- `save_keyframes_and_factor_wt_update_ikf()` 中捕获 ISAM 异常并跳过当前帧：
  ```c++
  bool isam_update_ok = true;
  try {
      isam->update(gtsam_graph, initial_estimate);
  } catch (const std::exception &e) {
      ROS_ERROR_STREAM("ISAM2 update error: " << e.what());
      gtsam_graph.print("new factors");
      initial_estimate.print("new values");
      isam_update_ok = false;
  }
  if (!isam_update_ok) {
      gtsam_graph.resize(0);
      initial_estimate.clear();
      return;
  }
  ```

### 为什么要这样做
- 如果第一帧新节点沿用旧姿态，会把旧图的末端作为新会话起点。例如旧图最后停在码头东侧，而新会话在西侧启动，直接沿用旧位姿会导致估计在西侧瞬间跳到东侧，轨迹完全错位。
- 添加单位先验让新会话从“传感器当前估计”开始，相当于对旧图加一个固定参考。这样旧地图保持锁定，新地图仅围绕当前估计小范围调整。
- GPS 因子若重复作用于旧节点会叠加历史观测。例如历史 GPS 已锁定码头原点，如果增量阶段再次给旧节点加相同 GPS 因子，会人为放大该节点权重，导致新节点优化时无法正确匹配真实观测。
- release 模式下如果 `isam->update()` 抛出异常（常见于输入退化因子或数据同步异常），旧版本直接 `abort()` 会导致整节点退出。现在通过捕获异常记录现场并跳过当前帧，可以让系统继续运行，同时保留问题因子供线下排查。

## 5. 可视化与导出流程的健壮性

### 做了什么
- 在遍历点云向量时增加索引合法性判断：
  ```c++
  if (thisKeyInd >= surf_cloud_keyframes.size()) continue;
  if (!surf_cloud_keyframes[thisKeyInd]) continue;
  ```
- 保存地图/回环可视化时如缺少对应点云则跳过：
  ```c++
  if (i >= cloud_key_poses_6D->size()) continue;
  if (!surf_cloud_keyframes[i]) continue;
  ```
- `save_map_service()` 生成地图前检查指针：
  ```c++
  if (!cloud_key_poses_6D || cloud_key_poses_6D->empty()) return true;
  ```

### 为什么要这样做
- 旧图可能缺失某些关键帧点云（例如用户删除部分 PCD 以节省空间）。添加边界检查后，系统会跳过这些点而不是崩溃，保持增量建图持续运行。
- 在导出地图时跳过空指针可以避免写入空数据。否则保存的 `map.pcd` 可能包含大量原点处的虚假点，影响后续分析。
- 检查指针是否初始化能避免用户在完全没有关键帧的情况下触发保存逻辑，减少误操作。

## 6. 增量建图的运行状态

加载旧图后，程序保持两组核心变量来区分“旧节点”与“新增节点”，并在多个模块中据此采取不同策略：

| 变量/结构 | 加载旧图后的值 | 增量建图时的作用 |
|-----------|----------------|------------------|
| `old_map_keyframe_count` | 等于从 `pose_graph.g2o` 读取的关键帧数量 | 判断新增关键帧的起点（`size() == old_count` 表示第一帧新节点）；用于循环检测、因子构造时跳过旧节点 |
| `old_map_gps_factor_count` | 等于历史 `gtsam::GPSFactor` 数量 | `add_gps_factor()` 仅为索引 ≥ `old_map_gps_factor_count` 的关键帧追加新 GPS 因子，避免重复插入 |
| `originLLA` & `geo_converter` | 若 `origin.txt` 存在则直接加载，否则等待首次 GPS 初始化时写入 | 在增量建图模式下如果已经成功加载旧 origin，则不重置 `geo_converter`，保持旧图的 ENU 基准 |
| `origin_loaded_from_map` | 加载 `origin.txt` 成功则为 `true`，否则为 `false` | 决定 `update_initial_guess()` 是否允许 GPS 覆盖旧原点；旧图存在时保持原值，防止坐标系跳变 |
| `loop_Index_container` | 初始化为旧图节点的自环（`key → key`），并附加旧回环边 | 新增回环时先检查容器，若已有记录则跳过；可视化时区分旧回环与新增回环 |
| `surf_cloud_keyframes` / `laser_cloud_raw_keyframes` | 预填充旧图中每个关键帧的点云（若缺失则为空指针） | 增量建图时追加新帧点云，旧帧在局部地图/回环匹配中直接复用 |

### 工作流程总结
1. **加载阶段**：根据旧图填充关键帧、因子、点云，并设置上述基准变量。
2. **增量阶段**：当 `cloud_key_poses_*` 的大小超过 `old_map_keyframe_count` 时，表示开始写入新节点。此时：
   - 里程计因素会对首个新节点加单位先验。
   - GPS 因子写入时跳过旧索引。
   - `geo_converter`、`originLLA` 仅在首次建图时刷新。
   - 回环容器只允许新索引参与判定，避免旧节点重复触发。
   - `update_initial_guess()` 会在增量模式下保持原有 `originLLA`，只在纯新建图或缺失 origin 时重置：
     ```c++
     const bool incremental_mode = (old_map_keyframe_count > 0);
     const bool keep_existing_origin = incremental_mode && origin_loaded_from_map;
     if (!keep_existing_origin) {
         originLLA = currentLLA;
         geo_converter.Reset(originLLA[0], originLLA[1], originLLA[2]);
         origin_loaded_from_map = true;
     }
     ```

这样可避免增量模式下因为 GPS 初始化再次重置 ENU 原点，导致旧图与新图在坐标上产生数米级偏移；当旧 origin 缺失时仍会在首次 GPS 到来时自动写入，保持新建图流程不变。

通过这些变量的联动，系统在“旧图固定 + 新图增量”模式下保持一致的坐标系与优化状态。

## 7. 目录镜像与文件拷贝

### 做了什么
- `copy_directory_recursively()` 用于把输入的地图目录镜像到运行目录的 `Map/`。
- 如果目标目录已存在会先删除再复制，保证运行目录中的数据与输入保持一致。

### 为什么要这样做
- 运行过程中写出的 `pose_graph.g2o`、`pcd_buffer` 等文件默认落在 `Map/`，镜像后可与加载数据对齐，避免路径混乱。
- 复制失败会给出警告，但不阻断后续加载流程，以免因权限/磁盘原因导致整个程序退出。

---

以上步骤共同确保：
1. 启动后立即加载旧图并发布已有地图信息；
2. 新数据只在旧图的约束基础上增量优化；
3. 各模块能安全地复用历史点云/回环信息，不会因数据缺失或索引错误崩溃；
4. 原始地图保留在 `Map/`，可继续导出或增量更新。
