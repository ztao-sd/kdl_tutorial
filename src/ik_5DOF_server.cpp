#include "ros/ros.h"
#include "ik_5DOF.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_5DOF_server");
    ros::NodeHandle n;

    // Create class instances
    ik_service ik_service;
    ik_5DOF ik_5DOF;

    // Boost bind
    const auto ik_vel_service_bind = boost::bind(&ik_service::ik_vel_service, &ik_service, _1, _2, ik_5DOF.ik_solver_vel);
    const auto ik_pos_service_bind = boost::bind(&ik_service::ik_pos_service, &ik_service, _1, _2, ik_5DOF.ik_solver_pos);

    ros::ServiceServer ik_vel_server = n.advertiseService<kdl_tutorial::ik_5DOF_vel::Request, kdl_tutorial::ik_5DOF_vel::Response>("ik_5DOF", ik_vel_service_bind);
    ROS_INFO_STREAM("Ready to calculate inverse kinematics of 5DOF robot arm!" << std::endl);
    ros::spin();

    return 0;
}