#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>
#include <Eigen/Core>

static Eigen::Isometry2d odom2isometry(const Eigen::Vector3f& odom_msg)
{
    Eigen::Isometry2d isometry = Eigen::Isometry2d::Identity();
    isometry.linear() << cos(odom_msg(2)), -sin(odom_msg(2)),
                         sin(odom_msg(2)),  cos(odom_msg(2));
    isometry.translation() = Eigen::Vector2d(odom_msg(0), odom_msg(1));
    return isometry;
}

#endif