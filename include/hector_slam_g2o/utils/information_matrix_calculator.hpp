#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>

class InformationMatrixCalculator
{
public:
    using PointT = pcl::PointXYZ;
    InformationMatrixCalculator(/* args */);
    ~InformationMatrixCalculator();

    static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry2d& relpose, double max_range = std::numeric_limits<double>::max());

    Eigen::Matrix3d calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry2d& relpose) const;

private:
    double weight(double a, double max_x, double min_y, double max_y, double x) const {
        double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));

        return min_y + (max_y - min_y) * y;
    }

private:
    double const_stddev_x;
    double const_stddev_q;

    double var_gain_a;
    double min_stddev_x;
    double max_stddev_x;
    double min_stddev_q;
    double max_stddev_q;
    double fitness_score_thresh;
};

InformationMatrixCalculator::InformationMatrixCalculator(/* args */)
{
    const_stddev_x = 0.5;
    const_stddev_q = 0.1;
    var_gain_a = 20.0;
    min_stddev_x = 0.1;
    max_stddev_x = 5.0;
    min_stddev_q = 0.05;
    max_stddev_q = 0.2;
    fitness_score_thresh = 0.5;
}

InformationMatrixCalculator::~InformationMatrixCalculator()
{
}

double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry2d& relpose, double max_range)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    tree->setInputCloud(cloud1);

    double fitness_score = 0.0;
    pcl::PointCloud<PointT> input_transformed;
    
    double rel_x = relpose.translation()[0];
    double rel_y = relpose.translation()[1];
    double rel_theta = Eigen::Rotation2Dd(relpose.linear()).angle();
    Eigen::AngleAxisf rel_rotation(rel_theta, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f rel_translation(rel_x, rel_y, 0);
    Eigen::Matrix4f rel_pose = (rel_translation * rel_rotation).matrix();

    pcl::transformPointCloud(*cloud2 , input_transformed, rel_pose);

    std::vector<int> indices(1);
    std::vector<float> dists(1);

    int nr = 0;
    for(size_t i = 0; i < input_transformed.points.size(); ++i)
    {
        tree->nearestKSearch(input_transformed.points[i], 1, indices, dists);
        if(dists[0] <= max_range)
        {
            fitness_score += dists[0];
            nr++;
        }
    }

    if(nr > 0)
    {
        return fitness_score / nr;
    }
    else
    {
        return std::numeric_limits<double>::max();
    }

}


Eigen::Matrix3d InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry2d& relpose) const
{
    double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

    double min_var_x = std::pow(min_stddev_x, 2);
    double max_var_x = std::pow(max_stddev_x, 2);
    double min_var_q = std::pow(min_stddev_q, 2);
    double max_var_q = std::pow(max_stddev_q, 2);

    float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);

    float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

    Eigen::Matrix3d inf = Eigen::Matrix3d::Identity();
    inf.topLeftCorner(2,2).array() /= w_x;
    inf.bottomRightCorner(1,1).array() /= w_q;
    return inf;    
}



#endif