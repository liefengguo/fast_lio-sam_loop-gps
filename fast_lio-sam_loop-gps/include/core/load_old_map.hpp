#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <mutex>
#include <cstdlib>
#include <limits>

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "common_lib.h"

#include <gtsam/slam/dataset.h>

bool LoadMap_gtsam(const std::string &map_path)
{
    system("exec rm -r Map");
    std::string ss = "exec cp -r " + map_path + " Map";
    system(ss.c_str());
    namespace bfs = boost::filesystem;
    bfs::path map_dir(map_path);
    if (map_dir.empty() || !bfs::exists(map_dir) || !bfs::is_directory(map_dir))
    {
        ROS_WARN("Map directory %s is not valid.", map_path.c_str());
        return false;
    }

    const bfs::path pose_graph_path = map_dir / "/pose_graph.g2o";
    if (!bfs::exists(pose_graph_path))
    {
        ROS_WARN("Pose graph file %s does not exist.", pose_graph_path.string().c_str());
        return false;
    }

    try
    {
        // gtsam::NonlinearFactorGraph graph;
        // gtsam::Values initial;
        gtsam::Values::shared_ptr initial;
        gtsam::NonlinearFactorGraph::shared_ptr temp_graph;
        boost::tie(temp_graph, initial) = gtsam::readG2o(pose_graph_path.string(), true);
        for (const auto &key_value : *initial)
        {
            const gtsam::Pose3 &pose = key_value.value.cast<gtsam::Pose3>();
            PointTypePose keyframe_pose;
            keyframe_pose.x = pose.x();
            keyframe_pose.y = pose.y();
            keyframe_pose.z = pose.z();
            gtsam::Rot3 rot = pose.rotation();
            // double roll, pitch, yaw;
            // rot.eulerAngles(2, 1, 0, yaw, pitch, roll);
            // keyframe_pose.roll = roll;
            // keyframe_pose.pitch = pitch;
            // keyframe_pose.yaw = yaw;
            // keyframe_pose.time = 0.0; // Time information is not stored in g2o files
            // cloud_key_poses_6D->points.push_back(keyframe_pose);
            PointType pt;
            pt.x = pose.x();
            pt.y = pose.y();
            pt.z = pose.z();
            cloud_key_poses_3D->points.push_back(pt);
        }

        old_map_keyframe_count = cloud_key_poses_3D->points.size();
        ROS_INFO("Loaded %zu keyframes from previous map.", old_map_keyframe_count);// initial 对应  VERTEX_SE3:QUAT 会有多少关键帧位姿
        ROS_INFO("temp_graph size: %zu", temp_graph->size());       // temp_graph 对应 EDGE_SE3:QUAT 会有多少约束
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Failed to load pose graph from " << pose_graph_path.string() << ": " << e.what());
        return false;
    }

    return true;
}