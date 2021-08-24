#ifndef REGISTRATIONS_HPP
#define REGISTRATIONS_HPP

#include <iostream>
#include <ros/ros.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>



pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr select_registration_method(ros::NodeHandle& pnh);


pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr select_registration_method(ros::NodeHandle& pnh)
{
    using PointT = pcl::PointXYZ;
    std::string registration_method = pnh.param<std::string>("registration_method", "ICP");

    if(registration_method == "ICP")
    {
        std::cout << "registration: ICP" << std::endl;
        pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
        icp->setTransformationEpsilon(1e-4);
        icp->setMaximumIterations(100);
        icp->setMaxCorrespondenceDistance(2.5);
        return icp;
    }
    else if(registration_method == "NDT")
    {
        std::cout << "registration: NDT" << std::endl;
        pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
        ndt->setTransformationEpsilon(1e-4);
        ndt->setMaximumIterations(100);
        ndt->setResolution(0.5);
        return ndt;
    }
    else if(registration_method == "GICP")
    {
        std::cout << "registration: GICP" << std::endl;
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
        gicp->setTransformationEpsilon(1e-4);
        gicp->setMaximumIterations(100);
        gicp->setMaxCorrespondenceDistance(2.5);
        return gicp;
    }
    else if(registration_method == "FAST_GICP")
    {
        std::cout << "registration: FAST_GICP" << std::endl;
        fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(new fast_gicp::FastGICP<PointT, PointT>());
        gicp->setNumThreads(0);
        gicp->setTransformationEpsilon(1e-4);
        gicp->setMaximumIterations(100);
        gicp->setMaxCorrespondenceDistance(2.5);
        gicp->setCorrespondenceRandomness(20);
        return gicp;
    }
    else if(registration_method == "FAST_VGICP")
    {
        std::cout << "registration: FAST_VGICP" << std::endl;
        fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
        vgicp->setNumThreads(0);
        vgicp->setResolution(1.0);
        vgicp->setTransformationEpsilon(1e-4);
        vgicp->setMaximumIterations(100);
        vgicp->setCorrespondenceRandomness(20);
        return vgicp;
    }


    return nullptr;    
}

#endif