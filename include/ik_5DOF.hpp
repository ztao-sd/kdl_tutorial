#ifndef IK_5DOF_H // head guards
#define IK_5DOF_H

// Orocos KDL package
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_kdl/tf2_kdl.h>

#include "kdl_tutorial/ik_5DOF_pos.h"
#include "kdl_tutorial/ik_5DOF_vel.h"

class ik_5DOF
{
private:
    /* data */
public:
    KDL::Tree tree;
    KDL::Chain chain;
    std::string name0, name1, name2, name3, name4, name5;
    KDL::Vector v0, v1, v2, v3, v4, v5;
    KDL::Rotation rot0, rot1, rot2, rot3, rot4, rot5;
    KDL::Joint j1, j2, j3, j4, j5;
    KDL::Frame f0, f1, f2, f3, f4, f5;
    KDL::Segment s0, s1, s2, s3, s4, s5;
    KDL::ChainFkSolverPos_recursive fk_solver;
    KDL::ChainIkSolverVel_pinv ik_solver_vel;
    KDL::ChainIkSolverPos_NR ik_solver_pos;

    ik_5DOF(/* args */); // Constructor (create ik solvers)
    ~ik_5DOF();          // Destructor
};

class ik_subscriber // callback functions
{
public:

    ik_subscriber();
    ~ik_subscriber();

    void jnt_state_sub_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg);

    template <class ik>
    void cart_state_sub_callback(const geometry_msgs::Transform::ConstPtr &transform_msg, ik ik_solver);
};

class ik_service //
{
public:
    ik_service();
    ~ik_service();

    bool ik_vel_service(kdl_tutorial::ik_5DOF_vel::Request &req,
                        kdl_tutorial::ik_5DOF_vel::Response &res,
                        KDL::ChainIkSolverVel_pinv ik_solver_vel);


    bool ik_pos_service(kdl_tutorial::ik_5DOF_pos::Request &req,
                        kdl_tutorial::ik_5DOF_pos::Response &res,
                        KDL::ChainIkSolverPos_NR ik_solver_pos);

    KDL::JntArray JointState2JntArray(sensor_msgs::JointState jnt_state_input);

};

#endif