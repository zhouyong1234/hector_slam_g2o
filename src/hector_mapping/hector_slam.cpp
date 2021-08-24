
/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hector_slam_g2o/hector_mapping/hector_slam.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>


// 构造函数
HectorMappingRos::HectorMappingRos()
    : private_node_("~"), lastGetMapUpdateIndex(-100), tfB_(0), map_publish_thread_(0), anchor_node(nullptr), anchor_edge(nullptr)
{
    ROS_INFO_STREAM("\033[1;32m----> Hector SLAM started.\033[0m");

    // 参数初始化
    InitParams();

    g2o_graph_optimizer = std::make_shared<G2oGraphOptimizer>();
    keyframe_updater = std::make_shared<KeyFrameUpdater>();
    loop_detector = std::make_shared<LoopDetector>(private_node_);
    inf_calculator = std::make_shared<InformationMatrixCalculator>();
    lidar_motion_calibrator = std::make_shared<LidarMotionCalibrator>(&tf_);

    laser_scan_subscriber_ = node_handle_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this); // 雷达数据处理

    imu_sub_ = node_handle_.subscribe("/IMU_data", 10, &HectorMappingRos::imuCallback, this);

    graph_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("pose_graph",1,true);

    calib_scan_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("calib_scan", 1, true);

    if (p_pub_odometry_)
    {
        odometryPublisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_hector", 50);
    }

    tfB_ = new tf::TransformBroadcaster();

    slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_),
                                                        p_map_size_, p_map_size_,
                                                        Eigen::Vector2f(p_map_start_x_, p_map_start_y_),
                                                        p_map_multi_res_levels_);

    slamProcessor->setUpdateFactorFree(p_update_factor_free_);                // 0.4
    slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);        // 0.9
    slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_); // 0.4
    slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);   // 0.9

    // 多层地图的初始化
    int mapLevels = slamProcessor->getMapLevels();
    mapLevels = 1; // 这里设置成只发布最高精度的地图，如果有其他需求，如进行路径规划等等需要多层地图时，注释本行。

    std::string mapTopic_ = "map";
    for (int i = 0; i < mapLevels; ++i)
    {
        mapPubContainer.push_back(MapPublisherContainer());
        slamProcessor->addMapMutex(i, new HectorMapMutex());

        std::string mapTopicStr(mapTopic_);

        if (i != 0)
        {
            mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
        }

        std::string mapMetaTopicStr(mapTopicStr);
        mapMetaTopicStr.append("_metadata");

        MapPublisherContainer &tmp = mapPubContainer[i];
        tmp.mapPublisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
        tmp.mapMetadataPublisher_ = node_handle_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

        setMapInfo(tmp.map_, slamProcessor->getGridMap(i)); // 设置地图服务

        if (i == 0)
        {
            mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
        }
    }

    // 新建一个线程用来发布地图
    map_publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));
    map_to_odom_.setIdentity();

    // 创建定时器，定时执行优化
    // double graph_optimize_interval = 10.0;
    optimization_timer = node_handle_.createWallTimer(ros::WallDuration(graph_optimize_interval_), &HectorMappingRos::optimizationCallback, this);


    // 查找 base_link -> front_laser_link 的tf，循环5次以确保其能找到
    int count = 5;
    ros::Duration dur(0.5);
    while (count-- != 0)
    {
        if (tf_.waitForTransform(p_base_frame_, p_scan_frame_, ros::Time(0), dur))
        {
            tf_.lookupTransform(p_base_frame_, p_scan_frame_, ros::Time(0), laserTransform_);
            break;
        }
        else
        {
            ROS_WARN("lookupTransform laser frame into base_link timed out.");
        }
    }

}

HectorMappingRos::~HectorMappingRos()
{
    delete slamProcessor;

    if (tfB_)
        delete tfB_;

    if (map_publish_thread_)
        delete map_publish_thread_;

    if(anchor_node)
        delete anchor_node;

    if(anchor_edge)
        delete anchor_edge;
}

// ros的参数初始化
void HectorMappingRos::InitParams()
{
    private_node_.param("pub_map_baselink_tf", pub_map_to_baselink_tf_, true);
    private_node_.param("pub_map_odom_tf", p_pub_map_odom_transform_, true);
    private_node_.param("pub_odometry_topic", p_pub_odometry_, true);
    private_node_.param("tracking_frame", p_tf_map_scanmatch_transform_frame_name_, std::string("base_link"));

    private_node_.param("scan_topic", p_scan_topic_, std::string("laser_scan"));
    private_node_.param("scan_frame", p_scan_frame_, std::string("front_laser_link"));
    private_node_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);
    private_node_.param("use_max_scan_range", p_use_max_scan_range_, 20.0);

    private_node_.param("map_frame", p_map_frame_, std::string("map"));
    private_node_.param("odom_frame", p_odom_frame_, std::string("odom"));
    private_node_.param("base_frame", p_base_frame_, std::string("base_link"));

    private_node_.param("output_timing", p_timing_output_, false);
    private_node_.param("map_pub_period", p_map_pub_period_, 2.0);
    private_node_.param("graph_optimize_interval", graph_optimize_interval_, 10.0);

    private_node_.param("map_resolution", p_map_resolution_, 0.05);
    private_node_.param("map_size", p_map_size_, 2048);
    private_node_.param("map_start_x", p_map_start_x_, 0.5);
    private_node_.param("map_start_y", p_map_start_y_, 0.5);
    private_node_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

    private_node_.param("update_factor_free", p_update_factor_free_, 0.4);
    private_node_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

    private_node_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
    private_node_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

    double tmp = 0.0;
    private_node_.param("laser_min_dist", tmp, 0.2);
    p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);

    private_node_.param("laser_max_dist", tmp, 30.0);
    p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);

    private_node_.param("laser_z_min_value", tmp, -1.0);
    p_laser_z_min_value_ = static_cast<float>(tmp);

    private_node_.param("laser_z_max_value", tmp, 1.0);
    p_laser_z_max_value_ = static_cast<float>(tmp);

    private_node_.param("use_loop_close", use_loop_close_, false);
    private_node_.param("use_keyframe_constraint", use_keyframe_constraint_, false);
    private_node_.param("use_lidar_calibration", use_lidar_calibration_, false);
    private_node_.param("use_graph_optimize", use_graph_optimize_, false);

    private_node_.param("enable_imu_orientation", enable_imu_orientation_, false);

    private_node_.param("enable_imu_acceleration", enable_imu_acceleration_, false);

    max_keyframes_per_update = 10;
}

// 对ROS地图进行数据初始化与分配内存
void HectorMappingRos::setMapInfo(nav_msgs::GetMap::Response &map_, const hectorslam::GridMap &gridMap)
{
    Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
    mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

    // std::cout << "mapOrigin.x: " << mapOrigin.x() << " mapOrigin.y: " << mapOrigin.y() << std::endl;
    // std::cout << "resolution: " << gridMap.getCellLength() << std::endl;
    // std::cout << "width: " << gridMap.getSizeX() << std::endl;
    // std::cout << "height: " << gridMap.getSizeY() << std::endl;

    map_.map.info.origin.position.x = mapOrigin.x();
    map_.map.info.origin.position.y = mapOrigin.y();
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.resolution = gridMap.getCellLength();
    map_.map.info.width = gridMap.getSizeX();
    map_.map.info.height = gridMap.getSizeY();

    map_.map.header.frame_id = p_map_frame_;
    // 分配内存空间
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/**
 * 激光数据处理回调函数，将ros数据格式转换为算法中的格式，并转换成地图尺度，交由slamProcessor处理。
 * 算法中所有的计算都是在地图尺度下进行。  
 */
void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan &scan)
{
    start_time_ = std::chrono::steady_clock::now();

    ros::WallTime startTime = ros::WallTime::now();


    if(use_lidar_calibration_)
    {
        // 激光雷达运动畸变去除
        sensor_msgs::LaserScan scan_out;
        lidar_motion_calibrator->lidar_calibration(scan, scan_out);
        // 运动畸变去除后的激光数据
        calib_scan_pub_.publish(scan_out);
    }
    
    // 将 scan 转换成 点云格式
    projector_.projectLaser(scan, laser_point_cloud_, 30.0);

    Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

    // 将雷达数据的点云格式 更改成 hector 内部的数据格式
    if (rosPointCloudToDataContainer(laser_point_cloud_, laserTransform_, laserScanContainer, slamProcessor->getScaleToMap()))
    {
        // 首先获取上一帧的位姿，作为初值
        startEstimate = slamProcessor->getLastScanMatchPose();
        // std::cout << "startEstimate: " << startEstimate << std::endl;

        // 进入扫描匹配与地图更新
        slamProcessor->update(laserScanContainer, startEstimate);
    }
        
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "数据转换与扫描匹配用时: " << time_used_.count() << " 秒。" << std::endl;

    if (p_timing_output_)
    {
        ros::WallDuration duration = ros::WallTime::now() - startTime;
        ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec() * 1000.0f);
    }

    // 更新存储的位姿, 这里的位姿是 base_link 在 map 下的位姿
    poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

    
    if (pub_map_to_baselink_tf_)
    {
        // pub map -> odom -> base_link tf
        if (p_pub_map_odom_transform_)
        {
            tfB_->sendTransform(tf::StampedTransform(map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
            tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_odom_frame_, p_tf_map_scanmatch_transform_frame_name_));
            // tfB_->sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(), p_map_frame_, p_odom_frame_));
            // tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), ros::Time::now(), p_odom_frame_, p_tf_map_scanmatch_transform_frame_name_));
        }
        // pub map -> base_link tf
        else
        {
            tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
        }
    }

    // 发布 odom topic
    if (p_pub_odometry_)
    {
        nav_msgs::Odometry tmp;
        tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;
        tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
        // tmp.header.stamp = ros::Time::now();
        tmp.child_frame_id = p_base_frame_;
        odometryPublisher_.publish(tmp);
    }
    
    // 存储关键帧
    const ros::Time& stamp = laser_point_cloud_.header.stamp;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::convertPointCloudToPointCloud2(laser_point_cloud_, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 判断是否为关键帧
    Eigen::Isometry2d odom = odom2isometry(slamProcessor->getLastScanMatchPose());

    // std::cout << "odom: " << std::endl << odom.matrix() << std::endl;

    if(!keyframe_updater->update(odom))
    {
        // std::cout << "keyframe not updated..." << std::endl;
        return;
    }

    double accum_d = keyframe_updater->get_accum_distance();

    // std::cout << "accum_d: " << accum_d << std::endl;
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));
    keyframe_queue.push_back(keyframe);
    
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);

    
    // std::cout << "执行一次回调用时: " << time_used_.count() << " 秒。" << std::endl;
}

void HectorMappingRos::imuCallback(const sensor_msgs::ImuConstPtr& imu)
{
    if(!enable_imu_acceleration_ && !enable_imu_orientation_)
    {
        return;
    }
    imu_queue.push_back(imu);
}


// 发布地图的线程
void HectorMappingRos::publishMapLoop(double map_pub_period)
{
    ros::Rate r(1.0 / map_pub_period);
    while (ros::ok())
    {
        ros::Time mapTime(ros::Time::now());

        //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
        //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
        publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

        r.sleep();
    }
}

// 发布ROS地图
void HectorMappingRos::publishMap(MapPublisherContainer &mapPublisher,
                                  const hectorslam::GridMap &gridMap,
                                  ros::Time timestamp, MapLockerInterface *mapMutex)
{
    nav_msgs::GetMap::Response &map_(mapPublisher.map_);

    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
    {
        int sizeX = gridMap.getSizeX();
        int sizeY = gridMap.getSizeY();

        int size = sizeX * sizeY;

        std::vector<int8_t> &data = map_.map.data;

        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);

        if (mapMutex)
        {
            mapMutex->lockMap();
        }

        for (int i = 0; i < size; ++i)
        {
            if (gridMap.isFree(i))
            {
                data[i] = 0;
            }
            else if (gridMap.isOccupied(i))
            {
                data[i] = 100;
            }
        }

        lastGetMapUpdateIndex = gridMap.getUpdateIndex();

        if (mapMutex)
        {
            mapMutex->unlockMap();
        }
    }

    map_.map.header.stamp = timestamp;

    mapPublisher.mapPublisher_.publish(map_.map);
}

// 将点云数据转换成Hector中雷达数据的格式
bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud &pointCloud, const tf::StampedTransform &laserTransform, hectorslam::DataContainer &dataContainer, float scaleToMap)
{
    size_t size = pointCloud.points.size();
    dataContainer.clear();

    tf::Vector3 laserPos(laserTransform.getOrigin());

    // dataContainer.setOrigo(Eigen::Vector2f::Zero());
    // 将base_link到雷达坐标系的坐标转换 乘以地图分辨率 当成这帧数据的 origo
    dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y()) * scaleToMap);

    for (size_t i = 0; i < size; ++i)
    {
        const geometry_msgs::Point32 &currPoint(pointCloud.points[i]);

        float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;
        if ((dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_))
        {
            if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f))
            {
                continue;
            }

            // 距离太远的点跳动太大，如果距离大于使用距离(20m)，就跳过
            if (dist_sqr > p_use_max_scan_range_ * p_use_max_scan_range_)
                continue;
            
            // 点的坐标左乘base_link->laser_link的tf 将得到该点在base_link下的 xy 坐标, 但是z坐标是不正确
            tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));
            
            // 通过再减去 base_link->laser_link的tf的z的值，得到该点在base_link下正确的 z 坐标
            float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

            if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
            {
                // 将雷达数据的 x y 都乘地图的分辨率 0.05 再放入dataContainer中
                dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(), pointPosBaseFrame.y()) * scaleToMap);
            }
        }
    }

    return true;
}

void HectorMappingRos::optimizationCallback(const ros::WallTimerEvent& event)
{
    /**
     * 1、关键帧更新
     * 2、向图中插入节点：关键帧位姿
     * 3、固定第一个节点
     * 4、向图中插入边：相邻两帧的相对位姿、回环检测到的相对位姿
     * 5、执行优化
     **/

    // std::cout << "执行一次优化" << std::endl;

    bool keyframe_updated = updateKeyframeQueue();
    if(!keyframe_updated & !updateImuQueue())
    {
        return;
    }

    if(use_loop_close_)
    {
        // std::cout << "loop detection..." << std::endl;

        tf::Transform transform;

        std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes);

        for(const auto& loop : loops)
        {
            matrixAsTransform(loop->relative_pose, transform);

            double rel_x = transform.getOrigin().x();
            double rel_y = transform.getOrigin().y();
            double rel_theta = transform.getRotation().getAngle();

            SE2 rel_pose = SE2(rel_x, rel_y, rel_theta);

            Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
            information.topLeftCorner(2,2).array() /= 0.1;
            information.bottomRightCorner(1,1).array() /= 0.1;

            Eigen::Isometry2d relpose = Eigen::Isometry2d::Identity();
            relpose.linear() << cos(rel_theta), -sin(rel_theta),
                                sin(rel_theta),  cos(rel_theta);
            relpose.translation() = Eigen::Vector2d(rel_x, rel_y);

            // Eigen::Matrix3d information = inf_calculator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);

            // std::cout << "loop information matrix: " << std::endl << information << std::endl;

            std::cout << "loop pose: " << std::endl;
            std::cout << "rel_x: " << rel_x << " rel_y: " << rel_y << " rel_theta: " << rel_theta << std::endl;

            // 回环约束加入图
            if(use_loop_close_)
            {
                auto edge = g2o_graph_optimizer->add_se2_edge(loop->key1->node, loop->key2->node, rel_pose, information);
                g2o_graph_optimizer->add_robust_kernel(edge, "Huber", 1.0);
                // graph_edges.push_back(edge);
                loop_edges.push_back(edge);
            }
        }
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // fix first node adaptive
    if(anchor_node)
    {
        VertexSE2* v = static_cast<VertexSE2*>(anchor_edge->vertices()[1]);
        double x = v->estimate().translation()[0];
        double y = v->estimate().translation()[1];
        double theta = v->estimate().rotation().angle();
        SE2 pose = SE2(x, y, theta);
        anchor_node->setEstimate(pose);
        
    }

    // std::cout << "begin to optimize..." << std::endl;

    start_time_ = std::chrono::steady_clock::now();

    // publishGraphForVisulization(&graph_pub_, graph_vertexs, graph_edges, 1);

    if(use_graph_optimize_)
    {
        int num_iterations = 20;
        g2o_graph_optimizer->optimize(num_iterations);
    }

    // g2o_graph_optimizer->optimizer.save("/home/touchair/ekf_ws/src/Creating-2D-laser-slam-from-scratch/lesson4/data/pose_graph.g2o");

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "执行一次优化用时: " << time_used_.count() << " 秒。" << std::endl;

    // std::cout << "optimize finished..." << std::endl;

    // 图可视化
    publishGraphForVisulization(&graph_pub_, graph_vertexs, graph_edges, 0);

    // 回环可视化
    publishGraphForVisulization(&graph_pub_, loop_edges, 2);

    // IMU约束可视化 imu_quat_edges
    publishGraphForVisulization(&graph_pub_, imu_quat_edges, 3);




    const auto& keyframe = keyframes.back();
    // map -> base_link
    Eigen::Isometry2d trans_estimate = Eigen::Isometry2d::Identity();
    trans_estimate.linear() = keyframe->node->estimate().rotation().toRotationMatrix();
    trans_estimate.translation() = keyframe->node->estimate().translation();

    // std::cout << "trans_estimate: " << std::endl << trans_estimate.matrix() << std::endl;

    // std::cout << "keyframe_odom: " << std::endl << keyframe->odom.matrix() << std::endl;

    // publish tf map->base_link x (odom->base_link).inverse() = map->odom
    Eigen::Matrix3d trans  = trans_estimate.matrix() * keyframe->odom.matrix().inverse();

    // map->odom
    // std::cout << "trans: " << std::endl << trans << std::endl;
    
    
}

bool HectorMappingRos::updateKeyframeQueue()
{
    /**
     * 1、关键帧位姿加入图
     * 2、固定第一帧
     * 3、相邻节点位姿加入图
     */
    if(keyframe_queue.empty())
    {
        return false;
    }

    Eigen::Quaterniond quat;
    quat.w() = map_to_odom_.getRotation().getW();
    quat.x() = map_to_odom_.getRotation().getX();
    quat.y() = map_to_odom_.getRotation().getY();
    quat.z() = map_to_odom_.getRotation().getZ();

    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2,1,0);

    Eigen::Matrix3d trans_map2odom;
    trans_map2odom << cos(euler[0]), -sin(euler[0]), map_to_odom_.getOrigin().getX(),
                      sin(euler[0]),  cos(euler[0]), map_to_odom_.getOrigin().getY(),
                      0,              0,             1;   

    
    // std::cout << "trans_map2odom: " << std::endl << trans_map2odom << std::endl;
    // Eigen::Isometry2d map2odom(Eigen::Matrix3d::Identity());
    Eigen::Isometry2d map2odom(trans_map2odom);

    int num_processed = 0;
    for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++)
    {
        num_processed = i;
        const auto& keyframe = keyframe_queue[i];
        new_keyframes.push_back(keyframe);

        // add pose node
        Eigen::Isometry2d pose = map2odom * keyframe->odom;

        double x = pose.translation()[0];
        double y = pose.translation()[1];
        double theta = Eigen::Rotation2Dd(pose.linear()).angle();

        SE2 t = SE2(x, y, theta);
        keyframe->node = g2o_graph_optimizer->add_se2_node(t);
        graph_vertexs.push_back(keyframe->node);

        // fix the first node
        if(keyframes.empty() && new_keyframes.size() == 1)
        {
            Eigen::Matrix3d inf = Eigen::Matrix3d::Identity();
            inf.topLeftCorner(2,2).array() /= 0.1;
            anchor_node = g2o_graph_optimizer->add_se2_node(SE2(0,0,0));
            graph_vertexs.push_back(anchor_node);
            anchor_node->setFixed(true);
            anchor_edge = g2o_graph_optimizer->add_se2_edge(anchor_node, keyframe->node, SE2(0,0,0), inf);
            graph_edges.push_back(anchor_edge);
        }

        if(i == 0 && keyframes.empty())
        {
            continue;
        }

        // add edge between consecutive keyframes
        const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i-1];

        // relative_pose between two frames
        Eigen::Isometry2d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;

        double rel_x = relative_pose.translation()[0];
        double rel_y = relative_pose.translation()[1];
        double rel_theta = Eigen::Rotation2Dd(relative_pose.linear()).angle();

        SE2 rel_pose = SE2(rel_x, rel_y, rel_theta);

        // 相邻两帧位姿间的信息矩阵
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
        information.topLeftCorner(2,2).array() /= 0.01;
        information.bottomRightCorner(1,1).array() /= 0.01;

        // Eigen::Isometry2d relpose = Eigen::Isometry2d::Identity();
        // relpose.linear() << cos(rel_theta), -sin(rel_theta),
        //                     sin(rel_theta),  cos(rel_theta);
        // relpose.translation() = Eigen::Vector2d(rel_x, rel_y);

        // Eigen::Matrix3d information = inf_calculator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);

        // std::cout << "keyframe information matrix: " << std::endl << information << std::endl;

        // std::cout << "consecutive keyframe pose: " << std::endl;
        // std::cout << "rel_x: " << rel_x << " rel_y: " << rel_y << " rel_theta: " << rel_theta << std::endl;

        if(use_keyframe_constraint_)
        {
            // 相邻两帧的位姿约束加入图
            auto edge = g2o_graph_optimizer->add_se2_edge(keyframe->node, prev_keyframe->node, rel_pose, information);
            g2o_graph_optimizer->add_robust_kernel(edge, "Huber", 1.0);
            // 可视化
            graph_edges.push_back(edge);
        }

    }

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);

    return true;

}


bool HectorMappingRos::updateImuQueue()
{
    if(keyframes.empty() || imu_queue.empty())
    {
        return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes)
    {
        if(keyframe->stamp > imu_queue.back()->header.stamp)
        {
            break;
        }

        if(keyframe->stamp < (*imu_cursor)->header.stamp)
        {
            continue;
        }

        auto closest_imu = imu_cursor;
        for(auto imu = imu_cursor; imu != imu_queue.end(); imu++)
        {
            auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
            auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
            if(std::abs(dt) < std::abs(dt2))
            {
                break;
            }
            closest_imu = imu;
        }

        imu_cursor = closest_imu;
        if(std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec()) > 0.2)
        {
            continue;
        }

        const auto& imu_ori = (*closest_imu)->orientation;
        const auto& imu_acc = (*closest_imu)->linear_acceleration;

        geometry_msgs::Vector3Stamped acc_imu;
        geometry_msgs::Vector3Stamped acc_base;
        geometry_msgs::QuaternionStamped quat_imu;
        geometry_msgs::QuaternionStamped quat_base;

        quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;

        quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
        acc_imu.vector = (*closest_imu)->linear_acceleration;
        quat_imu.quaternion = (*closest_imu)->orientation;

        try
        {
            tf_.transformVector("base_link", acc_imu, acc_base);
            tf_.transformQuaternion("base_link", quat_imu, quat_base);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }

        // std::cout << "quat_base: " << std::endl << quat_base << std::endl;

        keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
        keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);

        if(keyframe->orientation->w() < 0.0)
        {
            keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
        }

        if(enable_imu_orientation_)
        {
            Eigen::Matrix3d inf = Eigen::Matrix3d::Identity() / 6000.0;
            auto edge = g2o_graph_optimizer->add_se2_prior_quat_edge(keyframe->node, *keyframe->orientation, inf);

            g2o_graph_optimizer->add_robust_kernel(edge, "Huber", 1.0);
            imu_quat_edges.push_back(edge);

        }

        if(enable_imu_acceleration_)
        {
            Eigen::Matrix3d inf = Eigen::Matrix3d::Identity() / 3.0;

            auto edge = g2o_graph_optimizer->add_se2_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, inf);

            g2o_graph_optimizer->add_robust_kernel(edge, "Huber", 1.0);
            // loop_edges.push_back(edge);
        }

        updated = true; 
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) { return stamp < imu->header.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
}


void HectorMappingRos::publishGraphForVisulization(ros::Publisher* pub, std::vector<VertexSE2*>& vertexs, std::vector<EdgeSE2*>& edges, int color = 0)
{
    if(vertexs.empty() || edges.empty())
    {
        std::cout << "No edges..." << std::endl;
        return;
    }

    visualization_msgs::MarkerArray marray;

    // point red
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "hector-slam";
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    if(color == 0)
    {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
    }
    else
    {
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
    }

    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);

    // line blue
    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "hector-slam";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;

    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    else
    {
        edge.color.r = 1.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }

    edge.color.a = 1.0;

    m.action = visualization_msgs::Marker::ADD;
    uint id = 0;

    // add poses
    for(std::vector<VertexSE2*>::const_iterator pose_iter = vertexs.begin(); pose_iter != vertexs.end(); ++pose_iter)
    {
        auto vertex = *pose_iter;
        m.id = id;
        m.pose.position.x = vertex->estimate().translation()[0];
        m.pose.position.y = vertex->estimate().translation()[1];
        marray.markers.push_back(visualization_msgs::Marker(m));
        id++;
    }


    // add edges
    for(std::vector<EdgeSE2*>::const_iterator edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter)
    {
        auto constraint = *edge_iter;
        edge.points.clear();

        VertexSE2* v1 = dynamic_cast<VertexSE2*>(constraint->vertices()[0]);
        VertexSE2* v2 = dynamic_cast<VertexSE2*>(constraint->vertices()[1]);

        geometry_msgs::Point p;
        p.x = v1->estimate().translation()[0];
        p.y = v1->estimate().translation()[1];
        edge.points.push_back(p);

        p.x = v2->estimate().translation()[0];
        p.y = v2->estimate().translation()[1];
        edge.points.push_back(p);
        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));
        id++;

    }

    pub->publish(marray);

}


void HectorMappingRos::publishGraphForVisulization(ros::Publisher* pub, std::vector<EdgeSE2*>& edges, int color = 2)
{
    if(edges.empty())
    {
        // std::cout << "No loop edges..." << std::endl;
        return;
    }

    visualization_msgs::MarkerArray marray;

    
    // line green
    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "loop";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;

    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 1.0;
        edge.color.b = 1.0;
    }
    else if(color == 2)
    {
        edge.color.r = 0.0;
        edge.color.g = 1.0;
        edge.color.b = 0.0;
    }

    edge.color.a = 1.0;

    uint id = 0;

    // add edges
    for(std::vector<EdgeSE2*>::const_iterator edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter)
    {
        auto constraint = *edge_iter;
        edge.points.clear();

        VertexSE2* v1 = dynamic_cast<VertexSE2*>(constraint->vertices()[0]);
        VertexSE2* v2 = dynamic_cast<VertexSE2*>(constraint->vertices()[1]);

        geometry_msgs::Point p;
        p.x = v1->estimate().translation()[0];
        p.y = v1->estimate().translation()[1];
        edge.points.push_back(p);

        p.x = v2->estimate().translation()[0];
        p.y = v2->estimate().translation()[1];
        edge.points.push_back(p);
        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));
        id++;

    }

    pub->publish(marray);

}


void HectorMappingRos::publishGraphForVisulization(ros::Publisher* pub, std::vector<EdgeSE2PriorQuat*>& edges, int color = 3)
{
    if(edges.empty())
    {
        // std::cout << "No loop edges..." << std::endl;
        return;
    }


    visualization_msgs::MarkerArray marray;

    
    // line 
    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "imu";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;

    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 1.0;
        edge.color.b = 1.0;
    }
    else if(color == 3)
    {
        edge.color.r = 1.0;
        edge.color.g = 1.0;
        edge.color.b = 0.0;
    }

    edge.color.a = 1.0;

    uint id = 0;

    // add edges
    for(std::vector<EdgeSE2PriorQuat*>::const_iterator edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter)
    {
        auto constraint = *edge_iter;
        edge.points.clear();

        VertexSE2* v1 = dynamic_cast<VertexSE2*>(constraint->vertices()[0]);

        geometry_msgs::Point p;
        p.x = v1->estimate().translation()[0];
        p.y = v1->estimate().translation()[1];
        edge.points.push_back(p);

        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));
        id++;

    }

    pub->publish(marray);

}



inline void HectorMappingRos::matrixAsTransform(const Eigen::Matrix4f& out_mat, tf::Transform& bt)
{
    double mv[12];
    mv[0] = out_mat(0,0);
    mv[4] = out_mat(0,1);
    mv[8] = out_mat(0,2);
    mv[1] = out_mat(1,0);
    mv[5] = out_mat(1,1);
    mv[9] = out_mat(1,2);
    mv[2] = out_mat(2,0);
    mv[6] = out_mat(2,1);
    mv[10] = out_mat(2,2);
    tf::Matrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    tf::Vector3 origin(out_mat(0,3), out_mat(1,3), out_mat(2,3));
    bt = tf::Transform(basis, origin);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_hector_slam");

    HectorMappingRos hector_slam;

    ros::spin();

    return (0);
}