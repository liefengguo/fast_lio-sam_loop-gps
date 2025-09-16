/**
 * 更新里程计轨迹
 */
void update_global_path(const PointTypePose &pose_in)
{
    string odometry_frame = map_frame;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometry_frame;
    pose_stamped.pose.position.x =  pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z =  pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    global_path.poses.push_back(pose_stamped);
}

// 这里直接使用Tmi的增量进行判断即可
bool need_new_keyframe()
{
    if(first_keyframe)
    {	
        first_keyframe = false;
        return true;
    }
    else
    {
        // 计算时间差，并判断
        double time_diff = std::fabs(lidar_end_time-last_timestamp_keyframe);
        if(time_diff<keyframe_timestamp_th)
            return false;

        // 计算位姿增量差，并判断
        Eigen::Affine3f last_Toi = stateIkfomToAffine3f(last_odom_kf_state);
        Eigen::Affine3f cur_Toi = stateIkfomToAffine3f(state_point);

        Eigen::Affine3f transBetween = last_Toi.inverse() * cur_Toi;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        // std::cout << "delta_euler: " << roll << " " << pitch << " " << yaw << std::endl;
        // std::cout << "delta_t: " << x << " " << y << " " << z << std::endl << std::endl;

        if(sqrt(x*x + y*y + z*z) > keyframe_trans_th || fabs(roll) > keyframe_rot_th ||
                            fabs(pitch) > keyframe_rot_th || fabs(yaw) > keyframe_rot_th)
            return true;
        else
            return false;
    }
}

void keyframe_detection(const ros::Publisher & pubLidarOdom, const ros::Publisher & pubLaserCloudFull_lidar)
{
    if(keyframe_pub_en)
    {
        if(need_new_keyframe())
        {
            publish_lidar_odometry(pubLidarOdom);
            publish_frame_lidar(pubLaserCloudFull_lidar);
            ROS_INFO("\033[1;32m----> Create new keyframe by LIO.\033[0m");

            // ! add-mtc-20250329, 添加位姿图后端（ref: lio-sam）
            std::lock_guard<std::mutex> lock(mtx);

            if(loop_closure_enable_flag)
            {
                if(correct_fe_flag)
                {
                    save_keyframes_and_factor();
                    correct_poses();
                }
                else
                {
                    save_keyframes_and_factor_wt_update_ikf();
                    correct_poses_wt_rebuild_ikd();
                }        
            }

            // ! 更新时间戳和状态量, 不能放在前面
            last_timestamp_keyframe = lidar_end_time;
            last_odom_kf_state = state_point;
        }

        static tf::TransformBroadcaster br3;
        tf::Transform                   transform;
        tf::Quaternion                  qmo;
        Eigen::Vector3d Correct_tmo = correct_Tmo.translation();
        Eigen::Quaterniond Correct_qmo = correct_Tmo.rotation().toQuaternion();
        transform.setOrigin(tf::Vector3(Correct_tmo(0), \
                                        Correct_tmo(1), \
                                        Correct_tmo(2)));
        qmo.setW(Correct_qmo.w());
        qmo.setX(Correct_qmo.x());
        qmo.setY(Correct_qmo.y());
        qmo.setZ(Correct_qmo.z());
        transform.setRotation( qmo );
        br3.sendTransform( tf::StampedTransform( transform, lidarOdom.header.stamp, map_frame, odometry_frame ) );      
    }
    else
    {
        if (scan_pub_en && scan_lidar_pub_en) 
        {
            publish_frame_lidar(pubLaserCloudFull_lidar);
        }   
    }
}

void loop_closure_thread()
{
    if (loop_closure_enable_flag == false)
        return;

    ros::Rate rate(loop_closure_frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        // std::cout << "run loop closure!" << std::endl;
        perform_loop_closure();
        visualize_loop_closure();
    }
}

bool detect_loop_closure_distance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloud_key_poses_3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loop_Index_container.find(loopKeyCur);
    if (it != loop_Index_container.end())
      return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtree_history_key_poses->setInputCloud(copy_cloud_key_poses_3D);
    kdtree_history_key_poses->radiusSearch(copy_cloud_key_poses_3D->back(), history_keyframe_search_radius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
      int id = pointSearchIndLoop[i];
      if (abs(copy_cloud_key_poses_6D->points[id].time - lidar_end_time) > history_keyframe_search_time_diff)
      {
        loopKeyPre = id;
        break;
      }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
      return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void perform_loop_closure()
{
    if (cloud_key_poses_3D->points.empty() == true)
        return;
    
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳

    mtx.lock();
    *copy_cloud_key_poses_3D = *cloud_key_poses_3D;
    *copy_cloud_key_poses_6D = *cloud_key_poses_6D;
    mtx.unlock();

    // find keys
    int loopKeyCur;
    int loopKeyPre;

    if (detect_loop_closure_distance(&loopKeyCur, &loopKeyPre) == false)
        return;
    
    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    mtx.lock();
    loop_find_near_keyframes(cureKeyframeCloud, loopKeyCur, 0);
    loop_find_near_keyframes(prevKeyframeCloud, loopKeyPre, history_keyframe_search_num);
    mtx.unlock();

    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        return;

    if (pubHistoryKeyFrames.getNumSubscribers() != 0)
        publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, map_frame);

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(history_keyframe_search_radius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > history_keyframe_fitness_score)
        return;
    else
        ROS_INFO("\033[1;32m----> Loop ICP Check Pass!!.\033[0m");

    // publish corrected cloud
    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
      pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
      pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
      publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, map_frame);
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloud_key_poses_6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloud_key_poses_6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    mtx.lock();
    loop_Index_queue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loop_pose_queue.push_back(poseFrom.between(poseTo));
    loop_noise_queue.push_back(constraintNoise);
    mtx.unlock();

    // add loop constriant
    loop_Index_container[loopKeyCur] = loopKeyPre;
}

bool detect_loop_closure_multi_cond(int *latestID, int *closestID)
{

    return false;
}

void visualize_loop_closure()
{
    if (loop_Index_container.empty())
      return;
    
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometry_frame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometry_frame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loop_Index_container.begin(); it != loop_Index_container.end(); ++it)
    {
      int key_cur = it->first;
      int key_pre = it->second;
      geometry_msgs::Point p;
      p.x = copy_cloud_key_poses_6D->points[key_cur].x;
      p.y = copy_cloud_key_poses_6D->points[key_cur].y;
      p.z = copy_cloud_key_poses_6D->points[key_cur].z;
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
      p.x = copy_cloud_key_poses_6D->points[key_pre].x;
      p.y = copy_cloud_key_poses_6D->points[key_pre].y;
      p.z = copy_cloud_key_poses_6D->points[key_pre].z;
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

void loop_find_near_keyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloud_key_poses_6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
      int keyNear = key + i;
      if (keyNear < 0 || keyNear >= cloudSize )
        continue;
        
      *nearKeyframes += *transformPointCloud(surf_cloud_keyframes[keyNear],   &copy_cloud_key_poses_6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
      return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

// 里程计线程
void get_current_pose(state_ikfom cur_state)     // fast_lio_sam
{
    //  欧拉角是没有群的性质，所以从SO3还是一般的rotation matrix 转换过来的结果一样
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2,1,0);        //  yaw pitch roll  单位：弧度
    
    transformTobeMapped[0] = eulerAngle(2);             //  roll  使用 eulerAngles(2,1,0) 方法时，顺序是 ypr
    transformTobeMapped[1] = eulerAngle(1);             //  pitch
    transformTobeMapped[2] = eulerAngle(0);             //  yaw
    transformTobeMapped[3] = cur_state.pos(0);          //  x
    transformTobeMapped[4] = cur_state.pos(1);          //   y
    transformTobeMapped[5] = cur_state.pos(2);          // z
}

void save_keyframes_and_factor()
{
    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    add_odom_factor();

    if(use_gps)
        add_gps_factor();

    // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    add_loop_factor();

    gtsam::Pose3 latestRawEstimate = stateIkfomTogtsamPose3(state_point); // ! 记录优化前的位姿

    // update iSAM
    // std::cout << "gtsam_graph.size(): " << gtsam_graph.size() << " initial_estimate.size(): " << initial_estimate.size() << std::endl;
    // gtsam_graph.print();
    // initial_estimate.print();
    isam->update(gtsam_graph, initial_estimate);
    isam->update();     // 这一步很重要，如果后端发生了回环优化，则这里可以把前端修过来

    if (aloop_Is_closed == true)
    {
        isam->update();
        isam->update();
        isam->update();  
        isam->update();
        isam->update();
    }

    gtsam_graph.resize(0);
    initial_estimate.clear();

    // save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    isam_current_estimate = isam->calculateEstimate();
    latestEstimate = isam_current_estimate.at<gtsam::Pose3>(isam_current_estimate.size()-1);
    correct_Tmo = latestEstimate * latestRawEstimate.inverse();        // ! 更新矫正矩阵
    
    // static tf::TransformBroadcaster br3;
    // tf::Transform                   transform;
    // tf::Quaternion                  qmo;
    // Eigen::Vector3d Correct_tmo = correct_Tmo.translation();
    // Eigen::Quaterniond Correct_qmo = correct_Tmo.rotation().toQuaternion();
    // transform.setOrigin(tf::Vector3(Correct_tmo(0), \
    //                                 Correct_tmo(1), \
    //                                 Correct_tmo(2)));
    // qmo.setW(Correct_qmo.w());
    // qmo.setX(Correct_qmo.x());
    // qmo.setY(Correct_qmo.y());
    // qmo.setZ(Correct_qmo.z());
    // transform.setRotation( qmo );
    // br3.sendTransform( tf::StampedTransform( transform, lidarOdom.header.stamp, map_frame, odometry_frame ) );

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloud_key_poses_3D->size(); // this can be used as index
    cloud_key_poses_3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloud_key_poses_6D->push_back(thisPose6D);

    // ESKF状态和方差  更新
    state_ikfom state_updated = kf.get_x(); //  获取cur_pose (还没修正)
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    //  更新状态量
    state_updated.pos = pos;
    state_updated.rot =  q;
    state_point = state_updated; // 对state_point进行更新，state_point可视化用到
    kf.change_x(state_updated);  // 对cur_pose 进行isam2优化后的修正

    // 更新协方差
    auto P_updated = kf.get_P();  // ikf中前6个维度是先旋转后平移
    correct_pose_covariance = isam->marginalCovariance(isam_current_estimate.size()-1);  // 6*6 先旋转后平移
    // std::cout << "P_update: ";
    for (int k = 0; k < 6; k++)
    {
        // std::cout << P_updated(k, k) << " ";
        P_updated(k, 0) = correct_pose_covariance(k, 0);
        P_updated(k, 1) = correct_pose_covariance(k, 1);
        P_updated(k, 2) = correct_pose_covariance(k, 2);
        P_updated(k, 3) = correct_pose_covariance(k, 3);
        P_updated(k, 4) = correct_pose_covariance(k, 4);
        P_updated(k, 5) = correct_pose_covariance(k, 5);
        // std::cout << correct_pose_covariance(k, k) << " ";
    }
    kf.change_P(P_updated);
    // std::cout << std::endl<< std::endl;
    
    // 存储降采样点云和原始（降采样）点云
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_down_body, *thisSurfKeyFrame);
    if(sparse_raw_point_cloud_flag)
        pcl::copyPointCloud(*feats_down_body, *thislaserCloudRawKeyFrame);
    else
        pcl::copyPointCloud(*feats_undistort, *thislaserCloudRawKeyFrame);
    surf_cloud_keyframes.push_back(thisSurfKeyFrame);
    laser_cloud_raw_keyframes.push_back(thislaserCloudRawKeyFrame);

    // save path for visualization
    update_global_path(thisPose6D);
}

void save_keyframes_and_factor_wt_update_ikf()
{
    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    add_odom_factor_fastlio();

    if(use_gps)
        add_gps_factor();

    // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    add_loop_factor();

    gtsam::Pose3 latestRawEstimate = stateIkfomTogtsamPose3(state_point); // ! 记录优化前的位姿

    // update iSAM
    isam->update(gtsam_graph, initial_estimate);
    isam->update();     // 这一步很重要，如果后端发生了回环优化，则这里可以把前端修过来

    if (aloop_Is_closed == true)
    {
        isam->update();
        isam->update();
        isam->update();  
        isam->update();
        isam->update();
    }

    gtsam_graph.resize(0);
    initial_estimate.clear();

    // save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    isam_current_estimate = isam->calculateEstimate();
    latestEstimate = isam_current_estimate.at<gtsam::Pose3>(isam_current_estimate.size()-1);
    correct_Tmo = latestEstimate * latestRawEstimate.inverse();        // ! 更新矫正矩阵

    // static tf::TransformBroadcaster br3;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // Eigen::Vector3d Correct_tmo = correct_Tmo.translation();
    // Eigen::Quaterniond Correct_qmo = correct_Tmo.rotation().toQuaternion();
    // transform.setOrigin(tf::Vector3(Correct_tmo(0), \
    //                                 Correct_tmo(1), \
    //                                 Correct_tmo(2)));
    // q.setW(Correct_qmo.w());
    // q.setX(Correct_qmo.x());
    // q.setY(Correct_qmo.y());
    // q.setZ(Correct_qmo.z());
    // transform.setRotation( q );
    // br3.sendTransform( tf::StampedTransform( transform, lidarOdom.header.stamp, map_frame, odometry_frame ) );

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloud_key_poses_3D->size(); // this can be used as index
    cloud_key_poses_3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloud_key_poses_6D->push_back(thisPose6D);

    correct_pose_covariance = isam->marginalCovariance(isam_current_estimate.size()-1);  // 6*6 先旋转后平移

    // 存储降采样点云和原始（降采样）点云
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_down_body, *thisSurfKeyFrame);
    if(sparse_raw_point_cloud_flag)
        pcl::copyPointCloud(*feats_down_body, *thislaserCloudRawKeyFrame);
    else
        pcl::copyPointCloud(*feats_undistort, *thislaserCloudRawKeyFrame);
    surf_cloud_keyframes.push_back(thisSurfKeyFrame);
    laser_cloud_raw_keyframes.push_back(thislaserCloudRawKeyFrame);

    // save path for visualization
    update_global_path(thisPose6D);
}

void add_odom_factor()
{
    if (cloud_key_poses_3D->points.empty())
    {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter
        gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initial_estimate.insert(0, trans2gtsamPose(transformTobeMapped));
        // std::cout << "PriorFactor:" << trans2gtsamPose(transformTobeMapped) << std::endl;
        // std::cout << "add-prior factor!" << std::endl;
    }else{
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloud_key_poses_6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloud_key_poses_3D->size()-1, cloud_key_poses_3D->size(), poseFrom.between(poseTo), odometryNoise));
        initial_estimate.insert(cloud_key_poses_3D->size(), poseTo);
    }
}

void add_odom_factor_fastlio()
{
    if (cloud_key_poses_3D->points.empty())
    {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter
        gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initial_estimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }else{
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = stateIkfomTogtsamPose3(last_odom_kf_state);  // diff from above function
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);        // 这里是里程计估计的位姿
        gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloud_key_poses_3D->size()-1, cloud_key_poses_3D->size(), poseFrom.between(poseTo), odometryNoise));
        // 进行预测矫正, diff from above function
        poseTo = correct_Tmo * poseTo;
        initial_estimate.insert(cloud_key_poses_3D->size(), poseTo);             // diff from above function
    }
}

void add_loop_factor()
{
    if (loop_Index_queue.empty())
      return;

    for (int i = 0; i < (int)loop_Index_queue.size(); ++i)
    {
      int indexFrom = loop_Index_queue[i].first;
      int indexTo = loop_Index_queue[i].second;
      gtsam::Pose3 poseBetween = loop_pose_queue[i];
      gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loop_noise_queue[i];
      gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loop_Index_queue.clear();
    loop_pose_queue.clear();
    loop_noise_queue.clear();
    aloop_Is_closed = true;
}

void correct_poses()  // 回环成功的话，更新轨迹。并且if correct_fe_flag == true, 重建Ikd-tree地图
{
    if (cloud_key_poses_3D->points.empty())
        return;

    if (aloop_Is_closed == true)
    {
        // 清空里程计轨迹
        global_path.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isam_current_estimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloud_key_poses_3D->points[i].x = isam_current_estimate.at<gtsam::Pose3>(i).translation().x();
            cloud_key_poses_3D->points[i].y = isam_current_estimate.at<gtsam::Pose3>(i).translation().y();
            cloud_key_poses_3D->points[i].z = isam_current_estimate.at<gtsam::Pose3>(i).translation().z();

            cloud_key_poses_6D->points[i].x = cloud_key_poses_3D->points[i].x;
            cloud_key_poses_6D->points[i].y = cloud_key_poses_3D->points[i].y;
            cloud_key_poses_6D->points[i].z = cloud_key_poses_3D->points[i].z;
            cloud_key_poses_6D->points[i].roll = isam_current_estimate.at<gtsam::Pose3>(i).rotation().roll();
            cloud_key_poses_6D->points[i].pitch = isam_current_estimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloud_key_poses_6D->points[i].yaw = isam_current_estimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            update_global_path(cloud_key_poses_6D->points[i]);
        }

        // 清空局部map， reconstruct  ikdtree submap
        recontruct_ikd_tree();
    
        ROS_INFO("\033[1;32m----> ISAM2 Big Update after loop.\033[0m");
        aloop_Is_closed = false;
    }
}

void correct_poses_wt_rebuild_ikd()
{
    if (cloud_key_poses_3D->points.empty())
        return;

    if (aloop_Is_closed == true)
    {
        // 清空里程计轨迹
        global_path.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isam_current_estimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloud_key_poses_3D->points[i].x = isam_current_estimate.at<gtsam::Pose3>(i).translation().x();
            cloud_key_poses_3D->points[i].y = isam_current_estimate.at<gtsam::Pose3>(i).translation().y();
            cloud_key_poses_3D->points[i].z = isam_current_estimate.at<gtsam::Pose3>(i).translation().z();

            cloud_key_poses_6D->points[i].x = cloud_key_poses_3D->points[i].x;
            cloud_key_poses_6D->points[i].y = cloud_key_poses_3D->points[i].y;
            cloud_key_poses_6D->points[i].z = cloud_key_poses_3D->points[i].z;
            cloud_key_poses_6D->points[i].roll = isam_current_estimate.at<gtsam::Pose3>(i).rotation().roll();
            cloud_key_poses_6D->points[i].pitch = isam_current_estimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloud_key_poses_6D->points[i].yaw = isam_current_estimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            update_global_path(cloud_key_poses_6D->points[i]);
        }

        ROS_INFO("ISAM2 Update");
        aloop_Is_closed = false;
    }
}

void recontruct_ikd_tree()    // ikd-tree地图重建，使用lio-sam中的局部地图搜索方案	
{
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(cloud_key_poses_3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloud_key_poses_3D->back(), (double)surrounding_keyframe_search_radius, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
      int id = pointSearchInd[i];
      surroundingKeyPoses->push_back(cloud_key_poses_3D->points[id]);
    }

    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
    downSizeFilterSurroundingKeyPoses.setLeafSize(surrounding_keyframe_density, surrounding_keyframe_density, surrounding_keyframe_density);
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for(auto& pt : surroundingKeyPosesDS->points)
    {
      kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
      pt.intensity = cloud_key_poses_3D->points[pointSearchInd[0]].intensity;
    }

    extract_cloud(surroundingKeyPosesDS);
}

void extract_cloud(pcl::PointCloud<PointType>::Ptr cloud_to_extract)
{
    pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());
        
    for (int i = 0; i < (int)cloud_to_extract->size(); ++i)
    {
        if (pointDistance(cloud_to_extract->points[i], cloud_key_poses_3D->back()) > surrounding_keyframe_search_radius)
            continue;

        int thisKeyInd = (int)cloud_to_extract->points[i].intensity;
        pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surf_cloud_keyframes[thisKeyInd], &cloud_key_poses_6D->points[thisKeyInd]);
        *subMapKeyFrames += laserCloudSurfTemp;
    }

    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyFrames;      // for global map visualization
    downSizeFilterSubMapKeyFrames.setLeafSize(mapping_surf_leaf_size, mapping_surf_leaf_size, mapping_surf_leaf_size);
    downSizeFilterSubMapKeyFrames.setInputCloud(subMapKeyFrames);
    downSizeFilterSubMapKeyFrames.filter(*subMapKeyFramesDS);

    std::cout << "subMapKeyFramesDS->points.size(): " << subMapKeyFramesDS->points.size() << std::endl;

    ROS_INFO("\033[1;32m----> Reconstructed ikd-tree Map.\033[0m");
    ikdtree.reconstruct(subMapKeyFramesDS->points);
    int featsFromMapNum = ikdtree.validnum();
    int kdtree_size_st = ikdtree.size();
    std::cout << "featsFromMapNum  =  "   << featsFromMapNum   <<  "\t" << " kdtree_size_st   =  "  <<  kdtree_size_st  << std::endl;
}

// 发布全局地图
void visualize_global_map_thread()
{
    ros::Rate rate(0.2);
    while (ros::ok()){
      rate.sleep();
      publish_global_map();
    }
}

void publish_global_map()
{   
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);

    if (pubLaserCloudSurround.getNumSubscribers() == 0)
      return;

    if (cloud_key_poses_3D->points.empty() == true)
      return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloud_key_poses_3D);
    kdtreeGlobalMap->radiusSearch(cloud_key_poses_3D->back(), global_map_visualization_search_radius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
      globalMapKeyPoses->push_back(cloud_key_poses_3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(global_map_visualization_pose_density, global_map_visualization_pose_density, global_map_visualization_pose_density); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for(auto& pt : globalMapKeyPosesDS->points)
    {
      kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
      pt.intensity = cloud_key_poses_3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
        if (pointDistance(globalMapKeyPosesDS->points[i], cloud_key_poses_3D->back()) > global_map_visualization_search_radius)
        continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *transformPointCloud(surf_cloud_keyframes[thisKeyInd],    &cloud_key_poses_6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(global_map_visualization_leaf_size, global_map_visualization_leaf_size, global_map_visualization_leaf_size); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, map_frame);
}

// 存储地图
bool save_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (cloud_key_poses_6D->size() < 1) {
			ROS_INFO("NO ENCOUGH POSE!");
			return false;
		}
		pcl::PointCloud<PointType>::Ptr globalRawCloud(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr globalRawCloudDS(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

        mtx.lock();
		for (int i = 0; i < (int) cloud_key_poses_3D->size(); i++) {
			*globalRawCloud += *transformPointCloud(laser_cloud_raw_keyframes[i], &cloud_key_poses_6D->points[i]);
			cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloud_key_poses_6D->size()<< " ...";
		}
        pcl::VoxelGrid<PointType> downSizeFilterSurf2;
        downSizeFilterSurf2.setLeafSize(global_map_server_leaf_size, global_map_server_leaf_size, global_map_server_leaf_size);
		downSizeFilterSurf2.setInputCloud(globalRawCloud);	
		downSizeFilterSurf2.filter(*globalRawCloudDS);
        mtx.unlock();

		*globalMapCloud += *globalRawCloudDS;
		std::cout << "map size: " << globalMapCloud->size() << std::endl;

		if (globalMapCloud->empty()) {
			std::cout << "empty global map cloud!" << std::endl;
			return false;
		}
		pcl::io::savePCDFileASCII(save_directory + "globalmap_lidar_feature.pcd", *globalMapCloud);
		cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed!" << endl;

		return true;
}

void gps_handler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
{
    if(use_gps) {
        mtxGpsInfo.lock();
        gpsQueue.push_back(*gpsMsg);
        mtxGpsInfo.unlock();
    }
}

bool sync_gps(std::deque<nav_msgs::Odometry> &gpsBuf, nav_msgs::Odometry &aligedGps, double timestamp,
double eps_cam)
{
    bool hasGPS = false;

    // if(gpsQueue.empty())
    //     std::cout << "empty?" << std::endl;

    while (!gpsQueue.empty()) {
        mtxGpsInfo.lock();
        if (gpsQueue.front().header.stamp.toSec() < timestamp - eps_cam) {
            // message too old
            std::cout << "message too old: " << timestamp-gpsQueue.front().header.stamp.toSec() << std::endl;
            gpsQueue.pop_front();
            mtxGpsInfo.unlock();
        } else if (gpsQueue.front().header.stamp.toSec() > timestamp + eps_cam) {
            // message too new
            std::cout << "message too new: " << gpsQueue.front().header.stamp.toSec()-timestamp << std::endl;
            mtxGpsInfo.unlock();
            break;
        } else {
            hasGPS = true;
            aligedGps = gpsQueue.front();
            gpsQueue.pop_front();
            mtxGpsInfo.unlock();
        }
    }

    if (hasGPS)
        return true;
    else
        return false;
}


// 初始化gps，并且无视Kf检测，创建第一个gps因子
void update_initial_guess()
{
    if (cloud_key_poses_3D->points.empty()) {
        system_initialized = false;
        if (use_gps) 
        {
            ROS_INFO("\033[1;32m----> GPS use to init pose.\033[0m");

            nav_msgs::Odometry alignedGPS;
            if (sync_gps(gpsQueue, alignedGPS, lidar_end_time, 1.0 / gps_frequence)) {
                double roll, pitch, yaw;
                tf::Matrix3x3(tf::Quaternion(alignedGPS.pose.pose.orientation.x,
                                             alignedGPS.pose.pose.orientation.y,
                                             alignedGPS.pose.pose.orientation.z,
                                             alignedGPS.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
                if (!std::isnan(yaw)) 
                {
                    if(!manual_gps_init)
                    {
                        Eigen::Quaterniond q = EulerToQuat(roll, pitch, yaw);
                        state_ikfom state_updated = kf.get_x();  // ! 使用这个yaw初始化ikf估计器
                        state_updated.rot = q;
                        kf.change_x(state_updated);
                    }
                    else
                    {
                        Eigen::Quaterniond q = EulerToQuat(roll, pitch, manual_init_yaw);
                        state_ikfom state_updated = kf.get_x();  // ! 使用这个yaw初始化ikf估计器
                        state_updated.rot = q;
                        kf.change_x(state_updated); 
                    }
                }
                else
                {
                    ROS_ERROR("GPS NAN, waiting for a better yaw");
                    return;
                }

                /** we store the origin wgs84 coordinate points in covariance[1]-[3] */
                originLLA.setIdentity();
                originLLA = Eigen::Vector3d(alignedGPS.pose.covariance[1], // ! 是lla，不是cov
                                            alignedGPS.pose.covariance[2],
                                            alignedGPS.pose.covariance[3]);
                /** set your map origin points */
                geo_converter.Reset(originLLA[0], originLLA[1], originLLA[2]);
                // WGS84->ENU, must be (0,0,0)
                Eigen::Vector3d enu;
                geo_converter.Forward(originLLA[0], originLLA[1], originLLA[2], enu[0], enu[1], enu[2]);

                if(1)
                {
                    double roll, pitch, yaw;
                    tf::Matrix3x3(tf::Quaternion(alignedGPS.pose.pose.orientation.x,
                                                    alignedGPS.pose.pose.orientation.y,
                                                    alignedGPS.pose.pose.orientation.z,
                                                    alignedGPS.pose.pose.orientation.w))
                            .getRPY(roll, pitch, yaw);
                    std::cout << "initial gps yaw: " << yaw << std::endl;
                    std::cout << "GPS Position: " << enu.transpose() << std::endl;
                    std::cout << "GPS LLA: " << originLLA.transpose() << std::endl;
                }
                

                /** add the first factor, we need this origin GPS point for prior map based localization,
                 * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                PointType gnssPoint;
                gnssPoint.x = enu[0],
                gnssPoint.y = enu[1],
                gnssPoint.z = enu[2];
                float noise_x = alignedGPS.pose.covariance[0];
                float noise_y = alignedGPS.pose.covariance[7];
                float noise_z = alignedGPS.pose.covariance[14];

                // std::cout << "first gps measurement: " << gnssPoint.x << " " << gnssPoint.y << " " << gnssPoint.z << std::endl;

                /** if we get reliable origin point, we adjust the weight of this gps factor to fix the map origin */
                //if (!updateOrigin) {
                noise_x *= 1e-4;
                noise_y *= 1e-4;
                noise_z *= 1e-4;
                // }
                gtsam::Vector Vector3(3);
                Vector3 << noise_x, noise_y, noise_z;
                gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                        gtsam::noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(0, gtsam::Point3(gnssPoint.x, gnssPoint.y, gnssPoint.z),
                                            gps_noise);
                keyframeGPSfactor.push_back(gps_factor);
                cloudKeyGPSPoses3D->points.push_back(gnssPoint);

                system_initialized = true;

                state_ikfom state_updated = kf.get_x();
                get_current_pose(state_updated);
                keyframe_detection(pubLidarOdom, pubLaserCloudFull_lidar);

                ROS_WARN("GPS init success");
                
            }
        }
    }

    if (!system_initialized) {
        ROS_ERROR("sysyem need to be initialized");
        return;
    }
}

void add_gps_factor()
{
    if (gpsQueue.empty()) 
        return;
    
    if (cloud_key_poses_3D->points.empty() || cloud_key_poses_3D->points.size() == 1)
            return;
    
    // last gps position
    static PointType lastGPSPoint;
    nav_msgs::Odometry thisGPS;
    if (sync_gps(gpsQueue, thisGPS, lidar_end_time, 1.0 / gps_frequence)) {

        // GPS too noisy, skip
        float noise_x = thisGPS.pose.covariance[0];
        float noise_y = thisGPS.pose.covariance[7];
        float noise_z = thisGPS.pose.covariance[14];

        // make sure the gps data is stable encough
        if (abs(noise_x) > gps_cov_threshold || abs(noise_y) > gps_cov_threshold)
        {
            ROS_INFO("\033[1;31m----> not stable gps data !\033[0m");
            // std::cout << "noise_x: " << noise_x << " noise_y: " << noise_y << std::endl;
            return;
        }
            
        double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
        Eigen::Vector3d LLA(thisGPS.pose.covariance[1], thisGPS.pose.covariance[2], thisGPS.pose.covariance[3]);
        geo_converter.Forward(LLA[0], LLA[1], LLA[2], gps_x, gps_y, gps_z);

        if (1) {
            ROS_INFO("curr LLA : %f, %f , %f", LLA[0], LLA[1], LLA[2]);
            ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
            ROS_INFO("curr gps cov: %f, %f , %f", thisGPS.pose.covariance[0],
                        thisGPS.pose.covariance[7], thisGPS.pose.covariance[14]);
        }

        // if (!use_gps_elevation) {
        //     gps_z = transformTobeMapped[5];
        //     noise_z = 0.01;
        // }

        // GPS not properly initialized (0,0,0)
        if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) 
        {
            ROS_INFO("\033[1;31m----> GPS not properly initialized \033[0m");
            return;
        }

        // Add GPS every a few meters
        PointType curGPSPoint;
        curGPSPoint.x = gps_x;
        curGPSPoint.y = gps_y;
        curGPSPoint.z = gps_z;
        if (pointDistance(curGPSPoint, lastGPSPoint) < gps_distance)
            return; 
        else
            lastGPSPoint = curGPSPoint;

        gtsam::Vector Vector3(3);
        Vector3 << noise_x, noise_y, noise_z;
        // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                gtsam::noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(cloud_key_poses_3D->size(),
                                    gtsam::Point3(gps_x, gps_y, gps_z),
                                    gps_noise);
        keyframeGPSfactor.push_back(gps_factor);
        cloudKeyGPSPoses3D->points.push_back(curGPSPoint);

        // only a trick!
        // we need to accumulate some accurate gps points to initialize the
        // transform between gps coordinate system and LIO coordinate system and
        // then we can add gps points one by one into the pose graph or the whole
        // pose graph will crashed if giving some respectively bad gps points at
        // first.
        if (keyframeGPSfactor.size() < gpc_factor_init_num) {
            ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
            return;
        }

        if (!gpsTransfromInit) {
            ROS_INFO("Initialize GNSS transform!");
            for (int i = 0; i < keyframeGPSfactor.size(); ++i) {
                gtsam::GPSFactor gpsFactor = keyframeGPSfactor.at(i);
                gtsam_graph.add(gpsFactor);
                gps_index_container[gpsFactor.key()] = i;
            }
            gpsTransfromInit = true;
        } else {
            gtsam_graph.add(gps_factor);
            gps_index_container[cloud_key_poses_3D->size()] =
                    cloudKeyGPSPoses3D->size() - 1;
        }
        aloop_Is_closed = true;
    }
}

// 发布pubGPSOdometry(natfix)+global_path
void publish_global_path()
{
    if(!cloud_key_poses_6D->empty())
    {
        // if (pubGPSOdometry.getNumSubscribers() != 0) {
            /** we transform the  ENU point to LLA point for visualization with rviz_satellite*/
            Eigen::Vector3d curr_point(cloud_key_poses_6D->back().x,
                                        cloud_key_poses_6D->back().y,
                                        cloud_key_poses_6D->back().z);
            Eigen::Vector3d curr_lla;
            // ENU->LLA
            geo_converter.Reverse(curr_point[0], curr_point[1], curr_point[2], curr_lla[0], curr_lla[1],
                                    curr_lla[2]);
            //                std::cout << std::setprecision(9)
            //                          << "CURR LLA: " << originLLA.transpose() << std::endl;
            //                std::cout << std::setprecision(9)
            //                          << "update LLA: " << curr_lla.transpose() << std::endl;
            sensor_msgs::NavSatFix fix_msgs;
            fix_msgs.header.stamp = ros::Time().fromSec(lidar_end_time);
            fix_msgs.header.frame_id = map_frame;
            fix_msgs.latitude = curr_lla[0];
            fix_msgs.longitude = curr_lla[1];
            fix_msgs.altitude = curr_lla[2];
            pubGPSOdometry.publish(fix_msgs);
        // }
    }

    // publish path
    if (pub_map_path.getNumSubscribers() != 0) {
        global_path.header.stamp = ros::Time().fromSec(lidar_end_time);
        global_path.header.frame_id = map_frame;
        pub_map_path.publish(global_path);
    }
} 
