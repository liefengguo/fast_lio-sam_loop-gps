#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <map>
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

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

extern std::vector<pcl::PointCloud<PointType>::Ptr> surf_cloud_keyframes;
extern std::vector<pcl::PointCloud<PointType>::Ptr> laser_cloud_raw_keyframes;
extern pcl::PointCloud<PointType>::Ptr preloaded_raw_map;
extern bool raw_map_needs_publish;
extern ros::Publisher pubLaserCloudSurround;
extern std::string map_frame;
extern std::map<int, int> loop_Index_container;
extern size_t old_map_gps_factor_count;
extern std::vector<std::pair<int, int>> loop_Index_queue;
extern std::vector<gtsam::Pose3> loop_pose_queue;
extern std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loop_noise_queue;
extern Eigen::Vector3d originLLA;
extern GeographicLib::LocalCartesian geo_converter;
extern bool origin_loaded_from_map;

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

bool load_keyframe_pointclouds(const boost::filesystem::path &map_dir, const size_t keyframe_count)
{
    namespace bfs = boost::filesystem;

    surf_cloud_keyframes.clear();
    laser_cloud_raw_keyframes.clear();

    if (keyframe_count == 0)
    {
        return true;
    }

    surf_cloud_keyframes.reserve(keyframe_count);
    laser_cloud_raw_keyframes.reserve(keyframe_count);

    const bfs::path buffer_dir = map_dir / "pcd_buffer";
    const bool buffer_available = bfs::exists(buffer_dir) && bfs::is_directory(buffer_dir);
    if (!buffer_available)
    {
        ROS_WARN_STREAM("pcd_buffer directory not found at " << buffer_dir.string() << ". Loaded clouds will be empty.");
    }

    size_t missing_files = 0;
    size_t failed_files = 0;

    for (size_t idx = 0; idx < keyframe_count; ++idx)
    {
        const bfs::path file_path = buffer_dir / (std::to_string(idx) + ".pcd");
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if (buffer_available && bfs::exists(file_path))
        {
            if (pcl::io::loadPCDFile(file_path.string(), *cloud) != 0)
            {
                ++failed_files;
                cloud->clear();
            }
        }
        else
        {
            ++missing_files;
        }
        surf_cloud_keyframes.push_back(cloud);
        laser_cloud_raw_keyframes.push_back(cloud);
    }

    if (missing_files > 0)
    {
        ROS_WARN_STREAM("Missing " << missing_files << " keyframe PCD files under " << buffer_dir.string());
    }
    if (failed_files > 0)
    {
        ROS_WARN_STREAM("Failed to read " << failed_files << " keyframe PCD files under " << buffer_dir.string());
    }

    return !surf_cloud_keyframes.empty();
}

void load_raw_map(const boost::filesystem::path &map_dir)
{
    namespace bfs = boost::filesystem;
    const bfs::path raw_path = map_dir / "raw_map.pcd";

    if (!preloaded_raw_map)
    {
        preloaded_raw_map.reset(new pcl::PointCloud<PointType>());
    }

    if (!bfs::exists(raw_path) || !bfs::is_regular_file(raw_path))
    {
        ROS_WARN_STREAM("Raw map file not found at " << raw_path.string());
        preloaded_raw_map->clear();
        raw_map_needs_publish = false;
        return;
    }

    if (pcl::io::loadPCDFile(raw_path.string(), *preloaded_raw_map) != 0)
    {
        ROS_WARN_STREAM("Failed to load raw map from " << raw_path.string());
        preloaded_raw_map->clear();
        raw_map_needs_publish = false;
        return;
    }

    raw_map_needs_publish = !preloaded_raw_map->empty();
    ROS_INFO_STREAM("Loaded raw_map.pcd with " << preloaded_raw_map->size() << " points.");
}

bool load_origin_file(const boost::filesystem::path &map_dir)
{
    namespace bfs = boost::filesystem;
    const bfs::path origin_path = map_dir / "origin.txt";
    if (!bfs::exists(origin_path) || !bfs::is_regular_file(origin_path))
    {
        ROS_WARN_STREAM("origin.txt not found at " << origin_path.string());
        return false;
    }

    std::ifstream ifs(origin_path.string());
    if (!ifs.is_open())
    {
        ROS_WARN_STREAM("Failed to open origin.txt at " << origin_path.string());
        return false;
    }

    std::string token;
    double lat = 0.0, lon = 0.0, alt = 0.0;
    bool parsed = false;
    while (ifs >> token)
    {
        if (token == "LLA:" || token == "LLA")
        {
            if (ifs >> lat >> lon >> alt)
            {
                parsed = true;
                break;
            }
        }
    }

    if (!parsed)
    {
        ROS_WARN_STREAM("Failed to parse origin.txt at " << origin_path.string());
        return false;
    }

    originLLA = Eigen::Vector3d(lat, lon, alt);
    geo_converter.Reset(lat, lon, alt);
    origin_loaded_from_map = true;
    ROS_INFO_STREAM("Loaded origin LLA from origin.txt: " << originLLA.transpose());
    return true;
}
} // namespace

inline bool PublishPreloadedRawMap()
{
    if (!raw_map_needs_publish)
        return false;

    if (!preloaded_raw_map || preloaded_raw_map->empty())
    {
        raw_map_needs_publish = false;
        return false;
    }

    if (pubLaserCloudSurround.getNumSubscribers() == 0)
    {
        return false;
    }

    publishCloud(pubLaserCloudSurround, preloaded_raw_map, ros::Time::now(), map_frame);
    raw_map_needs_publish = false;
    return true;
}

bool LoadMap_gtsam(const std::string &map_path)
{
    namespace bfs = boost::filesystem;

    raw_map_needs_publish = false;

    const bfs::path map_dir(map_path);
    if (!validate_map_directory(map_dir))
    {
        return false;
    }

    const bfs::path pose_graph_path = resolve_pose_graph_path(map_dir);
    ROS_INFO("Loading map from directory: %s", pose_graph_path.string().c_str());
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

    const size_t loaded_keyframe_count = cloud_key_poses_3D->size();
    if (!load_keyframe_pointclouds(map_dir, loaded_keyframe_count))
    {
        ROS_WARN("Keyframe clouds were not loaded from pcd_buffer.");
    }
    else
    {
        ROS_INFO_STREAM("[LOADED FINISHED!!!]: Loaded " << surf_cloud_keyframes.size() << " keyframe point clouds from pcd_buffer.");
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

    load_raw_map(map_dir);
    if (!load_origin_file(map_dir))
    {
        origin_loaded_from_map = false;
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

    loop_Index_container.clear();
    loop_Index_queue.clear();
    loop_pose_queue.clear();
    loop_noise_queue.clear();
    for (size_t i = 0; i < old_map_keyframe_count; ++i)
    {
        loop_Index_container[static_cast<int>(i)] = static_cast<int>(i);
    }
    for (const auto &factor : graph)
    {
        const auto between = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
        if (!between)
            continue;
        const gtsam::Key key1 = between->key1();
        const gtsam::Key key2 = between->key2();
        if (key1 >= old_map_keyframe_count || key2 >= old_map_keyframe_count)
            continue;
        const auto separation = std::llabs(static_cast<long long>(key1) - static_cast<long long>(key2));
        if (separation <= 1)
            continue;
        const int newer = static_cast<int>(std::max(key1, key2));
        const int older = static_cast<int>(std::min(key1, key2));
        loop_Index_container[newer] = older;
    }
    ROS_INFO("Established %zu loop index entries from loaded map.", loop_Index_container.size());
    ROS_INFO("Loaded %zu keyframes from previous map.", old_map_keyframe_count);
    ROS_INFO("Loaded factor graph with %zu factors.", graph.size());

    gtsam_graph.resize(0);
    initial_estimate.clear();

    return true;
}
