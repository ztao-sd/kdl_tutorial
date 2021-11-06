#include "ros/ros.h" // header for ROS system
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <urdf/model.h>
#include "std_msgs/String.h"

#include <kdl/chain.hpp> // Orocos KDL package
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <stdio.h>
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>


using namespace KDL;

class ik_subscribe
{
public:
    sensor_msgs::JointState cur_jnt_state;
    geometry_msgs::Transform p1;
    geometry_msgs::Twist v1;
    Frame p;
    JntArray cur_JntArray;

    void jnt_state_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg)
    {
        cur_jnt_state = *jnt_msg;
        ROS_INFO_STREAM("Joint state:" << cur_jnt_state << std::endl);
    }

    template <class ik>
    void ik_callback(const geometry_msgs::Transform::ConstPtr &transform_msg, ik ik_solver)
    {
        geometry_msgs::Transform T = *transform_msg;
        Vector v = Vector(T.translation.x, T.translation.y, T.translation.z);
        Rotation R = Rotation::Quaternion(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w);
        Frame kdl_frame = Frame(R, v);

        ROS_INFO_STREAM("KDL frame:" << kdl_frame << std::endl);

        // Compute inverse kinematics
        // if (ik_solver.CartToJnt(joint_pos, cart_pose, q_out))
        // {
        //     ROS_ERROR("Error: could not calculate forward kinematics");
        // }
        // else
        // {
        //     ROS_INFO("Foward kinematics result");
        //     for (unsigned int i = 0; i < nj; i++)
        //     {
        //         std::cout << q_out(i) << std::endl;
        //     }

        //     ROS_INFO("Sucess!");
        // }

        // ROS_INFO_STREAM("");
    }

    JntArray getJntArray(sensor_msgs::JointState jnt_state_input)
    {
        int nj = jnt_state_input.position.size();
        JntArray cur_JntArray = JntArray(nj);
        for (unsigned int i = 0; i < nj; i++)
        {
            cur_JntArray(i) = jnt_state_input.position[i];
            // ROS_INFO_STREAM("Joint " << i + 1 << ":" << cur_JntArray(i) << std::endl);
        }
        return cur_JntArray;
    }
};

// Prototype
template <class ik>
void ik_callback(const sensor_msgs::JointState::ConstPtr &cur_joint, ik ik_solver);

void chatterCallback(const std_msgs::String::ConstPtr &msg, ChainIkSolverPos_NR  ik_solver)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


// Service node that computes the forward kinematics of a robot
// Input: joint state , and robot description file
// Output: cartesian coordiantes

int main (int argc, char** argv)
{
    //--------------------//
    // KDL IK Solver
    //--------------------//
    // KDL Kinematic Chain
    Tree tree;
    Chain chain;
    std::string name0, name1, name2, name3, name4, name5;
    Vector v0, v1, v2, v3, v4, v5;
    Rotation rot0, rot1, rot2, rot3, rot4, rot5;
    Joint j1, j2, j3, j4, j5;
    Frame f0, f1, f2, f3, f4, f5;
    Segment s0,s1,s2,s3,s4,s5;
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

    // KDL FK solver on a kinematic chain
    ChainFkSolverPos_recursive fk_solver = ChainFkSolverPos_recursive(chain);

    // Inverse Kinematic Solver
    ChainIkSolverVel_pinv ik_solver_vel(chain);
    ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, 100, 1e-6);

    //----------------- ROS PART ------------------------------------------------//

    // ROS Initiation
    ros::init(argc, argv, "sub_ik");
    ros::NodeHandle n;

    // Bind ik_callback
    const auto ik_bind = boost::bind(ik_callback<ChainIkSolverPos_NR>, _1, ik_solver);
    

    // ROS Subscriber
    ik_subscribe ik_sub;
    const auto ik_joint_bind = boost::bind(&ik_subscribe::jnt_state_callback, &ik_sub, _1);
    const auto ik_cart_bind = boost::bind(&ik_subscribe::ik_callback<ChainIkSolverPos_NR >, &ik_sub, _1, ik_solver);
    ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>("joint_state", 10, ik_joint_bind);
    ros::Subscriber cart_state_sub = n.subscribe<geometry_msgs::Transform>("cart_state", 10, ik_cart_bind);

    // ROS Publisher
    geometry_msgs::Transform ee_tf;
    geometry_msgs::Twist ee_twist;
    ros::Publisher cart_pose_pub = n.advertise<geometry_msgs::Transform>("cartesian_pose", 1);
    ros::Publisher cart_vel_pub = n.advertise<geometry_msgs::Twist>("cartesian_velocity", 1);

    // Spin
    ros::spin();

    return 0;
}

// Functions


template <class ik>
void ik_callback(const sensor_msgs::JointState::ConstPtr &cur_joint, ik ik_solver)
    {
        // int nj = cur_joint.position.size();
        // JntArray cur_JntArray;
        // Frame cart_pose;
        // JntArray q_out;

        //     // Compute inverse kinematics
        //     if (ik_solver.CartToJnt(cur_JntArray, cart_pose, q_out))
        // {
        //     ROS_ERROR("Error: could not calculate forward kinematics");
        // }
        // else
        // {
        //     ROS_INFO("Foward kinematics result");
        //     for (unsigned int i = 0; i < nj; i++)
        //     {
        //         std::cout << q_out(i) << std::endl;
        //     }

        //     ROS_INFO("Sucess!");
        // }

        // ROS_INFO_STREAM("");
    }
 
