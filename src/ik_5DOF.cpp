#include "ik_5DOF.hpp"
#include "ros/ros.h" // header for ROS system

using namespace KDL;

//================== ik_5DOF ==========================/

ik_5DOF::ik_5DOF(/* args */) // Class constructor
    // Define a serial kinematic chain
    : fk_solver(chain), ik_solver_vel(chain), ik_solver_pos(chain, fk_solver, ik_solver_vel, 100, 1e-6)
{
    name0 = "base_link";
    name1 = "link_1";
    name2 = "link_2";
    name3 = "link_3";
    name4 = "link_4";
    name5 = "link_5";
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
    s0 = Segment(Joint(Joint::None), f0);
    s1 = Segment(j1, f1);
    s2 = Segment(j2, f2);
    s3 = Segment(j3, f3);
    s4 = Segment(j4, f4);
    s5 = Segment(j5, f5);
    chain.addSegment(s0);
    chain.addSegment(s1);
    chain.addSegment(s2);
    chain.addSegment(s3);
    chain.addSegment(s4);
    chain.addSegment(s5);

    // Update kinematic solvers
    fk_solver.updateInternalDataStructures();
    ik_solver_vel.updateInternalDataStructures();
    ik_solver_pos.updateInternalDataStructures();
}

ik_5DOF::~ik_5DOF()
{
    // ik_solver_pos.~ChainIkSolverPos_NR();
    // ik_solver_vel.~ChainIkSolverVel_pinv();
    // fk_solver.~ChainFkSolverPos_recursive();
}

//================== ik_suscriber ==========================/

//================== ik_service ==========================/

bool ik_service::ik_vel_service(kdl_tutorial::ik_5DOF_vel::Request &req,
                                kdl_tutorial::ik_5DOF_vel::Response &res,
                                ChainIkSolverVel_pinv ik_solver_vel)
{
    Twist cart_vel = Twist(Vector(req.cart_vel.linear.x, req.cart_vel.linear.y, req.cart_vel.linear.z),
                           Vector(req.cart_vel.angular.x, req.cart_vel.angular.y, req.cart_vel.angular.z));
    JntArray jnt_pos_cur = ik_service::JointState2JntArray(req.jnt_pos_cur);
    const int nj = jnt_pos_cur.rows();
    JntArray jnt_vel=JntArray(nj);
    res.jnt_vel.velocity.resize(nj);

    ROS_INFO_STREAM("nj :" << nj << std::endl);
    // for (unsigned int i = 0; i < 5; i++)
    // {
    //     ROS_INFO_STREAM("Cartesian Velocity :" << req.cart_vel << std::endl);
    // }

    ik_solver_vel.CartToJnt(jnt_pos_cur, cart_vel, jnt_vel);

    ROS_INFO_STREAM("nj :" << nj << std::endl);
    for (unsigned int i = 0; i < nj; i++)
    {
        //ROS_INFO_STREAM("J"<<i+1 << jnt_vel(i) << std::endl);
        res.jnt_vel.velocity[i] = jnt_vel(i);
    }
    //    for (unsigned int i = 0; i < nj-1; i++)
    // {
    //     res.jnt_vel.velocity[i] = jnt_vel(i);
    // }

    ROS_INFO_STREAM("Cartesian Velocity :" << req.cart_vel << std::endl);
    ROS_INFO_STREAM("Current Joint Position :" << req.jnt_pos_cur << std::endl);
    ROS_INFO_STREAM("Joint Velocity :" << res.jnt_vel << std::endl);


    return true;
}

bool ik_service::ik_pos_service(kdl_tutorial::ik_5DOF_pos::Request &req,
                                kdl_tutorial::ik_5DOF_pos::Response &res,
                                ChainIkSolverPos_NR ik_solver_pos)
{
    Frame cart_pose = Frame(Rotation::Quaternion(req.cart_pose.rotation.x, req.cart_pose.rotation.y, req.cart_pose.rotation.z, req.cart_pose.rotation.w),
                            Vector(req.cart_pose.translation.x, req.cart_pose.translation.y, req.cart_pose.translation.z));
    JntArray jnt_pos_init = ik_service::JointState2JntArray(req.jnt_pos_init);
    JntArray jnt_pos;

    ik_solver_pos.CartToJnt(jnt_pos_init, cart_pose, jnt_pos);

    int nj = jnt_pos.rows();
    for (unsigned int i = 0; i < nj; i++)
    {
        res.jnt_pos.position[i] = jnt_pos(i);
    }

    ROS_INFO_STREAM("Joint position :" << res.jnt_pos << std::endl);
    ROS_INFO_STREAM("Cartesian pose :" << req.cart_pose << std::endl);

    return true;
}

JntArray ik_service::JointState2JntArray(sensor_msgs::JointState jnt_state_input)
{
    int nj = jnt_state_input.position.size();
    JntArray cur_JntArray = JntArray(nj);
    for (unsigned int i = 0; i < nj; i++)
    {
        cur_JntArray(i) = jnt_state_input.position[i];
    }
    return cur_JntArray;
}

ik_service::ik_service()
{

}

ik_service::~ik_service()
{

}