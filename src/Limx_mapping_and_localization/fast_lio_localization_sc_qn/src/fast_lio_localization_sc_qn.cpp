#include "fast_lio_localization_sc_qn.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
FastLioLocalizationScQn::FastLioLocalizationScQn(const ros::NodeHandle &n_private):
    nh_(n_private)
{
    ////// ROS params
    // temp vars, only used in constructor
    std::string saved_map_path;
    double map_match_hz;
    MapMatcherConfig mm_config;
    auto &gc = mm_config.gicp_config_;
    auto &qc = mm_config.quatro_config_;
    // get params
    /* basic */
    nh_.param<std::string>("/basic/map_frame", map_frame_, "map");
    nh_.param<std::string>("/basic/saved_map", saved_map_path, "/home/mason/kitti.bag");
    nh_.param<double>("/basic/map_match_hz", map_match_hz, 1.0);
    nh_.param<double>("/basic/visualize_voxel_size", voxel_res_, 1.0);
    /* keyframe */
    nh_.param<double>("/keyframe/keyframe_threshold", keyframe_dist_thr_, 1.0);
    nh_.param<int>("/keyframe/num_submap_keyframes", mm_config.num_submap_keyframes_, 5);
    /* match */
    nh_.param<double>("/match/scancontext_max_correspondence_distance",
                      mm_config.scancontext_max_correspondence_distance_,
                      100.0);
    nh_.param<double>("/match/quatro_nano_gicp_voxel_resolution", mm_config.voxel_res_, 0.3);
    /* nano */
    nh_.param<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
    nh_.param<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
    nh_.param<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    nh_.param<double>("/nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 0.01);
    nh_.param<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
    nh_.param<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    nh_.param<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
    /* quatro */
    nh_.param<bool>("/quatro/enable", mm_config.enable_quatro_, false);
    nh_.param<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
    nh_.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
    nh_.param<int>("/quatro/max_correspondences", qc.quatro_max_num_corres_, 200);
    nh_.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.02);
    nh_.param<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.04);
    nh_.param<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
    nh_.param<double>("/quatro/noise_bound", qc.noise_bound_, 0.25);
    nh_.param<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 0.25);
    nh_.param<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.25);
    nh_.param<int>("/quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);
    temp_keyframe_dist_thr_ = keyframe_dist_thr_ ;
    keyframe_dist_thr_ = 0.1;
    ////// Matching init
    map_matcher_ = std::make_shared<MapMatcher>(mm_config);

    ////// Load map
    loadMap(saved_map_path);

    ////// ROS things
    raw_odom_path_.header.frame_id = map_frame_;
    corrected_odom_path_.header.frame_id = map_frame_;
    // publishers
    odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ori_path", 10, true);
    corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
    map_match_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_match", 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
    pubMapOdom = nh_.advertise<nav_msgs::Odometry> ("/odom", 10);
    saved_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/saved_map", 10, true);
    debug_src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10);
    debug_dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10);
    debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10);
    // subscribers
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "/Odometry", 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "/cloud_registered", 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&FastLioLocalizationScQn::odomPcdCallback, this, _1, _2));
    // Timers at the end
    match_timer_ = nh_.createTimer(ros::Duration(1 / map_match_hz), &FastLioLocalizationScQn::matchingTimerFunc, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/lidar_filter1", 1); 
    ROS_WARN("Main class, starting node...");

    {
        transform_base_pose = Eigen::Matrix4d::Identity();
        transform_base_pose << 1,0,0,0,
                               0,-1,0,0,
                               0,0,-1,0,
                               0,0,0,1;
    }
}

void FastLioLocalizationScQn::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
    PosePcd current_frame = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    auto temp_pose_corrected_eig_ = transform_base_pose * current_frame.pose_corrected_eig_ * transform_base_pose.inverse();
    geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(temp_pose_corrected_eig_, map_frame_);
    
    realtime_pose_pub_.publish(current_pose_stamped_);
    

    //if(first_relocation_success)
    {
        keyframe_dist_thr_ = temp_keyframe_dist_thr_;
        Eigen::Quaterniond quat(temp_pose_corrected_eig_.block<3, 3>(0, 0));
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(temp_pose_corrected_eig_(0, 3), temp_pose_corrected_eig_(1, 3), 0));
        transform.setRotation(tf::Quaternion(0, 0,quat.z(), quat.w()));

        broadcaster_.sendTransform(tf::StampedTransform(transform,
                                                    odom_msg->header.stamp,
                                                    map_frame_,
                                                    "body"));

        odomAftMapped.pose.pose.position = current_pose_stamped_.pose.position;
        odomAftMapped.pose.pose.orientation = current_pose_stamped_.pose.orientation;
        odomAftMapped.pose.pose.position.z = 0;
        odomAftMapped.pose.pose.orientation.x = 0.00;
        odomAftMapped.pose.pose.orientation.y = 0.00;
        // odomAftMapped.pose.pose.orientation.z = odom.pose.pose.orientation.z;
        // odomAftMapped.pose.pose.orientation.w = odom.pose.pose.orientation.w;
        odomAftMapped.header.stamp = odom_msg->header.stamp;
        odomAftMapped.header.frame_id = map_frame_;
        odomAftMapped.child_frame_id = "body";
        pubMapOdom.publish(odomAftMapped);



        
        // 创建过滤后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = current_frame.pcd_.makeShared();

        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud_ptr, output_cloud);
        // output_cloud.header = input_cloud->header;
        Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
        auto msg = pclToPclRos(transformPcd(*cloud_ptr, transform_base_pose), "body");
        msg.header.stamp = odom_msg->header.stamp;
        msg.header.frame_id ="body";
        pub_.publish(msg);
    }
    
    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame.pcd_, transform_base_pose * current_frame.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {
        // 1. save first keyframe
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 2. vis
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            updateOdomsAndPaths(current_frame);
        }
        is_initialized_ = true;
    }
    else
    {
        //// 1. check if keyframe
        if (checkIfKeyframe(current_frame, last_keyframe_))
        {
            // 2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                last_keyframe_ = current_frame;
            }
            current_keyframe_idx_++;
            //// 3. vis
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame);
            }
        }
    }
    return;
}

void FastLioLocalizationScQn::matchingTimerFunc(const ros::TimerEvent &event)
{
    if (!is_initialized_)
    {
        return;
    }
	
	
    //// 1. copy not processed keyframes
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        last_keyframe_copy = last_keyframe_;
        last_keyframe_.processed_ = true;
    }
    if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_)
    {
        return; // already processed or initial keyframe
    }

    static int count_failure = 0;
    RegistrationOutput reg_output;
    if(!first_relocation_success){
            //// 2. detect match and calculate TF
        // from last_keyframe_copy keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
        int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(last_keyframe_copy, saved_map_from_bag_);
        if (closest_keyframe_idx < 0)
        {
            return; // if no matched candidate
        }
        // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
        reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
                                                    saved_map_from_bag_,
                                                   closest_keyframe_idx);
            
    }else{
        
        pcl::PointXYZI search_pt;
        search_pt.x = last_keyframe_copy.pose_corrected_eig_(0,3);
        search_pt.y = last_keyframe_copy.pose_corrected_eig_(1,3);
        search_pt.z = last_keyframe_copy.pose_corrected_eig_(2,3);
        search_pt.intensity = 0.0;
        std::vector<int> nearest_indices;
        std::vector<float> nearest_sqr_dists;
        kdtree.nearestKSearch(search_pt, 1, nearest_indices, nearest_sqr_dists);
        const auto &[src_cloud, dst_cloud] =  map_matcher_->setSrcAndDstCloud(last_keyframe_copy,
                                                               saved_map_from_bag_,
                                                               *nearest_indices.begin(),
                                                                map_matcher_->config_.num_submap_keyframes_,
                                                                map_matcher_->config_.voxel_res_,
                                                                map_matcher_->config_.enable_quatro_);
        reg_output =  map_matcher_->icpAlignment(src_cloud, dst_cloud);
        // reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
        //                                             saved_map_from_bag_,
        //                                            *nearest_indices.begin());
        std::cerr<<"-------------------submap matches------------------"<<std::endl;
    }
    

    //// 3. handle corrected results
    if (reg_output.is_valid_) // TF the pose with the result of match
    {
        count_failure = 0;
        ROS_INFO("\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        
        if(!first_relocation_success){
            tf_pose_vec.push_back(reg_output.pose_between_eig_.block<3,1>(0,3));
        
            if(tf_pose_vec.size() >= 2 ){
                if( 0.2 > (tf_pose_vec[1] - tf_pose_vec[0]).norm()){
                    ROS_INFO("\033[loopclouse success]");
                    relocate_success_flag = true;
                    tf_pose_vec.clear();
                    tf_pose_vec.resize(0);
                    
                }else{
                    ROS_INFO("\033[reject loopclouse]");
                    std::cout<<"first = "<< tf_pose_vec[0]<<" second = "<<tf_pose_vec[1]<<std::endl;
                    tf_pose_vec.erase(tf_pose_vec.begin());
                }
            }
        }
        
        // if( reg_output.score_ > 0.5)

        if(relocate_success_flag)
        {
            first_relocation_success = true;
            last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
            Eigen::Matrix4d TFed_pose = reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
            // correct poses in vis data
            {
                auto temp_pose_eig_ = transform_base_pose * TFed_pose * transform_base_pose.inverse();
                //auto temp_pose_corrected_eig_ = transform_base_pose * pose_pcd_in.pose_corrected_eig_ * transform_base_pose.inverse();
                std::lock_guard<std::mutex> lock(vis_mutex_);
                corrected_odoms_.points[last_keyframe_copy.idx_] = pcl::PointXYZ(temp_pose_eig_(0, 3), temp_pose_eig_(1, 3), temp_pose_eig_(2, 3));
                corrected_odom_path_.poses[last_keyframe_copy.idx_] = poseEigToPoseStamped(temp_pose_eig_, map_frame_);
            }   
            // map matches
            //matched_pairs_xyz_.push_back({corrected_odoms_.points[last_keyframe_copy.idx_], raw_odoms_.points[last_keyframe_copy.idx_]}); // for vis
            // map_match_pub_.publish(getMatchMarker(matched_pairs_xyz_));
        }
        
    }else{
        if(count_failure >= 40){
            relocate_success_flag =false;
            first_relocation_success =  false;
        }
        count_failure++;
    }
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();

    debug_src_pub_.publish(pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
    debug_dst_pub_.publish(pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
    debug_coarse_aligned_pub_.publish(pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));

    // publish odoms and paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_odom_path_);
    }
    odom_pub_.publish(pclToPclRos(raw_odoms_, map_frame_));
    path_pub_.publish(raw_odom_path_);
    // publish saved map
    if (saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() > 0)
    {
        saved_map_pub_.publish(pclToPclRos(transformPcd(saved_map_pcd_,transform_base_pose), map_frame_));
        saved_map_vis_switch_ = false;
    }
    if (!saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() == 0)
    {
        saved_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    ROS_INFO("Matching: %.1fms, vis: %.1fms",
             duration_cast<microseconds>(t2_ - t1_).count() / 1e3,
             duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
    return;
}

void FastLioLocalizationScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    auto temp_pose_eig_ = transform_base_pose * pose_pcd_in.pose_eig_ * transform_base_pose.inverse();
    auto temp_pose_corrected_eig_ = transform_base_pose * pose_pcd_in.pose_corrected_eig_ * transform_base_pose.inverse();
    raw_odoms_.points.emplace_back(temp_pose_eig_(0, 3),
                               temp_pose_eig_(1, 3),
                               temp_pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(temp_pose_corrected_eig_(0, 3),
                                         temp_pose_corrected_eig_(1, 3),
                                         temp_pose_corrected_eig_(2, 3));
    raw_odom_path_.poses.emplace_back(poseEigToPoseStamped(temp_pose_eig_, map_frame_));
    corrected_odom_path_.poses.emplace_back(poseEigToPoseStamped(temp_pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::Marker FastLioLocalizationScQn::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs)
{
    visualization_msgs::Marker edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = map_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
        geometry_msgs::Point p_, p2_;
        p_.x = match_xyz_pairs[i].first.x;
        p_.y = match_xyz_pairs[i].first.y;
        p_.z = match_xyz_pairs[i].first.z;
        p2_.x = match_xyz_pairs[i].second.x;
        p2_.y = match_xyz_pairs[i].second.y;
        p2_.z = match_xyz_pairs[i].second.z;
        edges_.points.push_back(p_);
        edges_.points.push_back(p2_);
    }
    return edges_;
}

bool FastLioLocalizationScQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_dist_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}

void FastLioLocalizationScQn::loadMap(const std::string &saved_map_path)
{
    rosbag::Bag bag;

    bag.open(saved_map_path, rosbag::bagmode::Read);

    rosbag::View view1(bag, rosbag::TopicQuery("/keyframe_pcd"));
    rosbag::View view2(bag, rosbag::TopicQuery("/keyframe_pose"));
    std::vector<sensor_msgs::PointCloud2> load_pcd_vec;
    std::vector<geometry_msgs::PoseStamped> load_pose_vec;
    for (const rosbag::MessageInstance &pcd_msg : view1)
    {
        sensor_msgs::PointCloud2::ConstPtr pcd_msg_ptr = pcd_msg.instantiate<sensor_msgs::PointCloud2>();
        if (pcd_msg_ptr != nullptr)
        {
            load_pcd_vec.push_back(*pcd_msg_ptr);
        }
    }
    for (const rosbag::MessageInstance &pose_msg : view2)
    {
        geometry_msgs::PoseStamped::ConstPtr pose_msg_ptr = pose_msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg_ptr != nullptr)
        {
            load_pose_vec.push_back(*pose_msg_ptr);
        }
    }
    if (load_pcd_vec.size() != load_pose_vec.size())
    {
        ROS_ERROR("WRONG BAG FILE!!!!!");
    }
    for (size_t i = 0; i < load_pose_vec.size(); ++i)
    {
        saved_map_from_bag_.push_back(PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
        saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_, saved_map_from_bag_[i].pose_eig_);
        map_matcher_->updateScancontext(saved_map_from_bag_[i].pcd_); // note: update scan context for loop candidate detection
    }

    pose_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>) ;
    for (const auto& item : saved_map_from_bag_) {
        pcl::PointXYZI pt;
        pt.x = item.pose_eig_(0,3);
        pt.y = item.pose_eig_(1,3);
        pt.z = item.pose_eig_(2,3);
        pt.intensity = static_cast<float>(pose_cloud->size());  // 用 intensity 存 index
        pose_cloud->push_back(pt);
    }
    kdtree.setInputCloud(pose_cloud);

    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);
    bag.close();
    return;
}
