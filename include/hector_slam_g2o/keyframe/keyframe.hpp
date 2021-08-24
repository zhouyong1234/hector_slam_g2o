#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "hector_slam_g2o/graph_optimizer/se2.h"
#include "hector_slam_g2o/graph_optimizer/vertex_se2.hpp"
#include "hector_slam_g2o/graph_optimizer/edge_se2.hpp"

#include <boost/optional.hpp>

class KeyFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using PointT = pcl::PointXYZ;
    using Ptr = std::shared_ptr<KeyFrame>;

    KeyFrame(const ros::Time& stamp, const Eigen::Isometry2d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);
    ~KeyFrame();

    long id() const;
    SE2 estimate() const;
public:
    /* data */
    ros::Time stamp;
    Eigen::Isometry2d odom;
    double accum_distance;
    pcl::PointCloud<PointT>::ConstPtr cloud;
    VertexSE2* node;

    boost::optional<Eigen::Vector3d> acceleration;
    boost::optional<Eigen::Quaterniond> orientation;
};

KeyFrame::KeyFrame(const ros::Time& stamp, const Eigen::Isometry2d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud) : stamp(stamp), odom(odom), accum_distance(accum_distance), cloud(cloud), node(nullptr)
{

}

KeyFrame::~KeyFrame()
{
}

long KeyFrame::id() const 
{
    return node->id();
}

SE2 KeyFrame::estimate() const
{
    return node->estimate();
}


#endif
