#include "ros/ros.h"
#include "ik_5DOF.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_5DOF_client");
    ros::NodeHandle n;

    // Test
    // std::vector<double> jnt_pos1(5);
    //n.getParam("/joint_states/velocity", jnt_pos1);

    ROS_INFO_STREAM("Args= "<<argv[1]<<std::endl);
    // sensor_msgs::JointState jnt_state_test;
    // ROS_INFO_STREAM("Current position :"<<jnt_pos1[1]<<std::endl);

    ros::ServiceClient client=n.serviceClient<kdl_tutorial::ik_5DOF_vel>("ik_5DOF");

    // Get input from parameter server    
    kdl_tutorial::ik_5DOF_vel ik_5DOF_vel;

    n.getParam("/ik_5DOF_client/twist/linear/x",ik_5DOF_vel.request.cart_vel.linear.x);
    n.getParam("/ik_5DOF_client/twist/linear/y",ik_5DOF_vel.request.cart_vel.linear.y);
    n.getParam("/ik_5DOF_client/twist/linear/z",ik_5DOF_vel.request.cart_vel.linear.z);
    n.getParam("/ik_5DOF_client/twist/angular/x",ik_5DOF_vel.request.cart_vel.angular.x);
    n.getParam("/ik_5DOF_client/twist/angular/y",ik_5DOF_vel.request.cart_vel.angular.y);
    n.getParam("/ik_5DOF_client/twist/angular/z",ik_5DOF_vel.request.cart_vel.angular.z);

    ik_5DOF_vel.request.jnt_pos_cur.position.resize(5);
    n.getParam("/ik_5DOF_client/joint_states/position", ik_5DOF_vel.request.jnt_pos_cur.position);

    const int nj = ik_5DOF_vel.request.jnt_pos_cur.position.size();

    if (client.call(ik_5DOF_vel))
    {
        for (unsigned int i = 0; i < nj; i++)
        {
            ROS_INFO_STREAM("Joint" << i + 1 << " velocity: " << ik_5DOF_vel.response.jnt_vel.velocity[i] << std::endl);
        }
    }
    else
    {
        ROS_ERROR("Failed!");
        return 1;
    }

    return 0;
}