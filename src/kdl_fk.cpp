#include "ros/ros.h" // header for ROS system
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include <kdl/chain.hpp> // Orocos KDL package
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>



#include <stdio.h>
#include <iostream>

using namespace KDL;

// Service node that computes the forward kinematics of a robot
// Input: joint state , and robot description file
// Output: cartesian coordiantes

int main (int argc, char** argv)
{

    // ROS Initiation
    ros::init(argc, argv, "kdl_fk");
    ros::NodeHandle n;

    // URDF Parser
    if (argc != 2)
    {
        ROS_ERROR("Need a urdf file as argument");
        return -1;
    }
    std::string urdf_file = argv[1];
    urdf::Model arm_model;
    if (!arm_model.initFile(urdf_file))
    {
        ROS_ERROR("Failed to parse urdf file:");
        return -1;
    }
    ROS_INFO("Successfully parsed urdf file");

    // KDL Parser
    KDL::Tree tree1;
    if (!kdl_parser::treeFromUrdfModel(arm_model, tree1))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return -1;
    }
    ROS_INFO("Successfully created kdl tree from urdf file");

    // KDL Kinematic Chain
    KDL::Tree tree;
    KDL::Chain chain;
    std::string name0, name1, name2, name3, name4, name5;
    KDL::Vector v0, v1, v2, v3, v4, v5;
    KDL::Rotation rot0, rot1, rot2, rot3, rot4, rot5;
    KDL::Joint j1, j2, j3, j4, j5;
    KDL::Frame f0, f1, f2, f3, f4, f5;
    KDL::Segment s0,s1,s2,s3,s4,s5;
    name0="base_link";
    name1="link_1";
    name2="link_2";
    name3="link_3";
    name4="link_4";
    name5="link_5";
    v0 = Vector(0.0, 0.0, 91.0);
    v1 = Vector(0.0, 0.0, 128.4);
    v2 = Vector(0.0, 412.77, 0.0);
    v3 = Vector(406.40, 0.0, 0.0);
    v4 = Vector(107.80, 0.0, 0.0);
    v5 = Vector(0.0, 0.0, 50.0);
    rot0 = Rotation::RPY(0.0, 0.0, 0.0);
    rot1 = Rotation::RPY(PI / 2.0, 0.0, 0.0);
    rot2 = Rotation::RPY(0.0, 0.0, 0.0);
    rot3 = Rotation::RPY(0.0, 0.0, 0.0);
    rot4 = Rotation::RPY(-PI / 2.0, 0.0, -PI / 2.0);
    rot5 = Rotation::RPY(0, 0.0, 0.0);
    j1 = Joint(Joint::RotZ);
    j2 = Joint(Joint::RotZ);
    j3 = Joint(Joint::RotZ);
    j4 = Joint(Joint::RotZ);
    j5 = Joint(Joint::RotZ);
    f0 = Frame(rot0, v0); // F_tip
    f1 = Frame(rot1, v1);
    f2 = Frame(rot2, v2);
    f3 = Frame(rot3, v3);
    f4 = Frame(rot4, v4);
    f5 = Frame(rot5, v5);
    s0=Segment(Joint(Joint::None),f0);
    s1=Segment(j1,f1);
    s2=Segment(j2,f2);
    s3=Segment(j3,f3);
    s4=Segment(j4,f4);
    s5=Segment(j5,f5);
    chain.addSegment(s0);
    chain.addSegment(s1);
    chain.addSegment(s2);
    chain.addSegment(s3);
    chain.addSegment(s4);
    chain.addSegment(s5);

    // Debug chain
    // unsigned int n1=chain.getNrOfJoints();
    // unsigned int n2=chain.getNrOfSegments();
    // std::cout << n1 << ", " << n2 << std::endl;

    // KDL FK solver on a kinematic chain
    ChainFkSolverPos_recursive fk_solver = ChainFkSolverPos_recursive(chain);

    // Joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray joint_pos = JntArray(nj);
    for (unsigned int i = 0; i < nj; i++)
    {
        joint_pos(i) = 0.0;
    }

    joint_pos(0) = 0.8;
    joint_pos(1) = -0.2;
    joint_pos(2) = -0.3;
    joint_pos(3) = 1.1;
    joint_pos(4) = -0.1;

    // Frame to store answer
    KDL::Frame cart_pos;

    // Compute FK
    if (fk_solver.JntToCart(joint_pos, cart_pos))
    {
        ROS_ERROR("Error: could not calculate forward kinematics");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("Foward kinematics result"<<cart_pos<<std::endl);
        ROS_INFO("Success!");
    }

    // Inverse Kinematic Solver
    ChainIkSolverVel_pinv ik_solver_vel(chain);
    ROS_INFO("IK VEL SOLVER");
    ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, 100, 1e-6);
    JntArray q_out = JntArray(nj);

    // Compute inverse kinematics
    if (ik_solver.CartToJnt(joint_pos, cart_pos, q_out))
    {
        ROS_ERROR("Error: could not calculate forward kinematics");
        return -1;
    }
    else
    {
        ROS_INFO("Foward kinematics result");
        for (unsigned int i = 0; i < nj; i++)
        {
            std::cout << q_out(i) << std::endl;
        }

        ROS_INFO("Sucess!");
    }
    return 0;
}
