#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>


class KeyFrameUpdater
{
private:
    double keyframe_delta_trans;
    double keyframe_delta_angle;
    bool is_first;
    double accum_distance;
    Eigen::Isometry2d prev_keypose;
public:
    KeyFrameUpdater() : is_first(true), prev_keypose(Eigen::Isometry2d::Identity())
    {
        keyframe_delta_trans = 3.0;
        keyframe_delta_angle = 1.0;
        accum_distance = 0.0;
    }

    ~KeyFrameUpdater() {}

    bool update(const Eigen::Isometry2d& pose)
    {
        if(is_first)
        {
            is_first = false;
            prev_keypose = pose;
            return true;
        }

        Eigen::Isometry2d delta = prev_keypose.inverse() * pose;
        double dx = delta.translation().norm();
        double da = Eigen::Rotation2Dd(delta.linear()).angle();

        if(dx < keyframe_delta_trans && abs(da) < keyframe_delta_angle)
        {
            return false;
        }
        accum_distance += dx;
        prev_keypose = pose;
        return true;

    }

    double get_accum_distance() const
    {
        return accum_distance;
    }


};



#endif