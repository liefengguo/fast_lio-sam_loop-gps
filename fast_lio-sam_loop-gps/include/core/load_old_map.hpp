#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <mutex>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>
#include <string>

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "common_lib.h"

#include <gtsam/navigation/GPSFactor.h>
// #include <gtsam/noiseModel/Constrained.h>
// #include <gtsam/noiseModel/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

namespace
{
struct LoadedGPSRecord
{
    gtsam::GPSFactor factor;
    PointType measurement;
};

bool copy_directory_recursively(const boost::filesystem::path &source, const boost::filesystem::path &destination)
{
    namespace bfs = boost::filesystem;
    try
    {
        if (!bfs::exists(source) || !bfs::is_directory(source))
        {
            ROS_WARN_STREAM("Source map directory is invalid: " << source);
            return false;
        }

        boost::system::error_code ec;
        if (bfs::exists(destination))
        {
            bfs::remove_all(destination, ec);
            if (ec)
            {
                ROS_WARN_STREAM("Failed to clear destination directory " << destination << ": " << ec.message());
                return false;
            }
        }

        bfs::create_directories(destination);
        for (bfs::directory_iterator it(source); it != bfs::directory_iterator(); ++it)
        {
            const bfs::path &current = it->path();
            const bfs::path target = destination / current.filename();
            if (bfs::is_directory(current))
            {
                if (!copy_directory_recursively(current, target))
                {
                    return false;
                }
            }
            else if (bfs::is_regular_file(current))
            {
                bfs::copy_file(current, target, bfs::copy_option::overwrite_if_exists);
            }
        }
    }
    catch (const bfs::filesystem_error &e)
    {
        ROS_WARN_STREAM("Failed to copy map directory: " << e.what());
        return false;
    }
    return true;
}

inline bool validate_map_directory(const boost::filesystem::path &map_dir)
{
    if (map_dir.empty() || !boost::filesystem::exists(map_dir) || !boost::filesystem::is_directory(map_dir))
    {
        ROS_WARN("Map directory %s is not valid.", map_dir.string().c_str());
        return false;
    }
    return true;
}

inline boost::filesystem::path resolve_pose_graph_path(const boost::filesystem::path &map_dir)
{
    return map_dir / "pose_graph.g2o";
}

std::vector<LoadedGPSRecord> parse_gps_edges(const boost::filesystem::path &pose_graph_path)
{
    std::vector<LoadedGPSRecord> gps_records;

    std::ifstream ifs(pose_graph_path.string());
    if (!ifs.is_open())
    {
        ROS_WARN_STREAM("Failed to open pose graph file for GPS parsing: " << pose_graph_path);
        return gps_records;
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.rfind("EDGE_DIS:VEC3", 0) != 0)
        {
            continue;
        }

        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token)
        {
            tokens.push_back(token);
        }

        if (tokens.size() < 15)
        {
            ROS_WARN_STREAM("Malformed EDGE_DIS:VEC3 line ignored: " << line);
            continue;
        }

        try
        {
            const gtsam::Key pose_key = static_cast<gtsam::Key>(std::stoul(tokens[1]));
            const double meas_x = std::stod(tokens[3]);
            const double meas_y = std::stod(tokens[4]);
            const double meas_z = std::stod(tokens[5]);
            const double info_x = std::stod(tokens[6]);
            const double info_y = std::stod(tokens[10]);
            const double info_z = std::stod(tokens[14]);

            auto to_variance = [](double info) -> double
            {
                constexpr double max_variance = 1e6;
                if (info <= std::numeric_limits<double>::epsilon())
                {
                    return max_variance;
                }
                return 1.0 / info;
            };

            gtsam::Vector3 variances;
            variances << to_variance(info_x), to_variance(info_y), to_variance(info_z);
            auto noise = gtsam::noiseModel::Diagonal::Variances(variances);
            LoadedGPSRecord record{
                gtsam::GPSFactor(pose_key, gtsam::Point3(meas_x, meas_y, meas_z), noise),
                PointType()};
            record.measurement.x = meas_x;
            record.measurement.y = meas_y;
            record.measurement.z = meas_z;
            record.measurement.intensity = static_cast<float>(pose_key);
            gps_records.emplace_back(std::move(record));
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM("Failed to parse GPS edge: " << e.what() << " line: " << line);
        }
    }
    // ROS_INFO_STREAM("Parsed " << gps_records.size() << " GPS records from pose graph.");
    return gps_records;
}

void copy_pose_to_clouds(const gtsam::Pose3 &pose, const size_t index)
{
    if (!cloud_key_poses_3D || !cloud_key_poses_6D)
    {
        ROS_ERROR("Key pose clouds are not initialized before loading old map.");
        return;
    }

    PointType pt;
    pt.x = pose.translation().x();
    pt.y = pose.translation().y();
    pt.z = pose.translation().z();
    pt.intensity = static_cast<float>(index);
    cloud_key_poses_3D->push_back(pt);

    PointTypePose pose6D;
    pose6D.x = pt.x;
    pose6D.y = pt.y;
    pose6D.z = pt.z;
    pose6D.intensity = pt.intensity;
    const auto rpy = pose.rotation().rpy();
    pose6D.roll = rpy.x();
    pose6D.pitch = rpy.y();
    pose6D.yaw = rpy.z();
    pose6D.time = 0.0;
    cloud_key_poses_6D->push_back(pose6D);
}
} // namespace

bool LoadMap_gtsam(const std::string &map_path)
{
    namespace bfs = boost::filesystem;

    const bfs::path map_dir(map_path);
    if (!validate_map_directory(map_dir))
    {
        return false;
    }

    const bfs::path pose_graph_path = resolve_pose_graph_path(map_dir);
    if (!bfs::exists(pose_graph_path))
    {
        ROS_WARN("Pose graph file %s does not exist.", pose_graph_path.string().c_str());
        return false;
    }

    const bfs::path local_map_dir("Map");
    boost::system::error_code ec;
    if (!bfs::equivalent(map_dir, local_map_dir, ec))
    {
        if (!copy_directory_recursively(map_dir, local_map_dir))
        {
            ROS_WARN_STREAM("Failed to mirror map directory to workspace Map/. Continuing without local copy.");
        }
    }

    gtsam::NonlinearFactorGraph::shared_ptr graph_ptr;
    gtsam::Values::shared_ptr values_ptr;

    try
    {
        boost::tie(graph_ptr, values_ptr) = gtsam::readG2o(pose_graph_path.string(), true);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Failed to load pose graph from " << pose_graph_path << ": " << e.what());
        return false;
    }

    if (!graph_ptr || !values_ptr)
    {
        ROS_WARN_STREAM("Pose graph " << pose_graph_path << " did not produce valid graph/values.");
        return false;
    }

    gtsam::NonlinearFactorGraph graph = *graph_ptr;
    gtsam::Values values = *values_ptr;

    const std::vector<LoadedGPSRecord> gps_records = parse_gps_edges(pose_graph_path);

    if (!cloud_key_poses_3D || !cloud_key_poses_6D || !copy_cloud_key_poses_3D || !copy_cloud_key_poses_6D || !cloudKeyGPSPoses3D)
    {
        ROS_ERROR("Key pose clouds are not initialized.");
        return false;
    }

    cloud_key_poses_3D->clear();
    cloud_key_poses_6D->clear();
    if (cloud_key_poses_6D_unoptimized)
    {
        cloud_key_poses_6D_unoptimized->clear();
    }
    copy_cloud_key_poses_3D->clear();
    copy_cloud_key_poses_6D->clear();
    cloudKeyGPSPoses3D->clear();
    keyframeGPSfactor.clear();
    gps_index_container.clear();

    gtsam::KeyVector keys = values.keys();
    std::sort(keys.begin(), keys.end());
    for (size_t idx = 0; idx < keys.size(); ++idx)
    {
        const gtsam::Key key = keys[idx];
        if (!values.exists(key))
        {
            continue;
        }
        if (key != idx)
        {
            ROS_WARN_THROTTLE(1.0, "Loaded pose key %lu differs from sequential index %lu. Runtime assumes contiguous keys.", static_cast<unsigned long>(key), static_cast<unsigned long>(idx));
        }
        copy_pose_to_clouds(values.at<gtsam::Pose3>(key), idx);
    }

    cloud_key_poses_3D->width = static_cast<uint32_t>(cloud_key_poses_3D->size());
    cloud_key_poses_3D->height = 1;
    cloud_key_poses_3D->is_dense = true;
    cloud_key_poses_6D->width = static_cast<uint32_t>(cloud_key_poses_6D->size());
    cloud_key_poses_6D->height = 1;
    cloud_key_poses_6D->is_dense = true;
    *copy_cloud_key_poses_3D = *cloud_key_poses_3D;
    *copy_cloud_key_poses_6D = *cloud_key_poses_6D;
    if (cloud_key_poses_6D_unoptimized)
    {
        *cloud_key_poses_6D_unoptimized = *cloud_key_poses_6D;
    }

    if (!gps_records.empty())
    {
        for (size_t i = 0; i < gps_records.size(); ++i)
        {
            const LoadedGPSRecord &record = gps_records[i];
            keyframeGPSfactor.push_back(record.factor);
            gps_index_container[static_cast<int>(record.factor.key())] = static_cast<int>(i);
            cloudKeyGPSPoses3D->push_back(record.measurement);
            graph.add(record.factor);
        }
        cloudKeyGPSPoses3D->width = static_cast<uint32_t>(cloudKeyGPSPoses3D->size());
        cloudKeyGPSPoses3D->height = 1;
        cloudKeyGPSPoses3D->is_dense = true;
        gpsTransfromInit = true;
        // Total  gps_records to append: 1980, keyframeGPSfactor :1980,gps_index_container: 1980, cloudKeyGPSPoses3D size : 1980
        // Add gps factor at keyframe: 10,gps_index_container: 5,size: 6 
        ROS_INFO("Total  gps_records to append: %zu, keyframeGPSfactor :%zu,gps_index_container: %zu, cloudKeyGPSPoses3D size : %zu", gps_records.size(), keyframeGPSfactor.size(), gps_index_container.size(), cloudKeyGPSPoses3D->size());
    }

    try
    {
        isam->update(graph, values);
        gtsam::NonlinearFactorGraph locking_factors;
        const auto lock_noise = gtsam::noiseModel::Constrained::All(6);
        for (const auto key : keys)
        {
            locking_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, values.at<gtsam::Pose3>(key), lock_noise));
        }
        if (!locking_factors.empty())
        {
            isam->update(locking_factors, gtsam::Values());
        }
        isam->update();
        isam_current_estimate = isam->calculateEstimate();
        correct_Tmo = gtsam::Pose3();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Failed to initialize ISAM with old map: " << e.what());
        return false;
    }

    old_map_keyframe_count = cloud_key_poses_3D->size();
    old_map_gps_factor_count = keyframeGPSfactor.size();
    ROS_INFO("Loaded %zu keyframes from previous map.", old_map_keyframe_count);
    ROS_INFO("Loaded factor graph with %zu factors.", graph.size());

    // gtsam_graph.resize(0);
    // initial_estimate.clear();

    return true;
}
