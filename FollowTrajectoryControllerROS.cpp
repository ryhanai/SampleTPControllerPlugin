/**
   @author Ryo Hanai
*/

#include <sstream>
#include "FollowTrajectoryControllerROS.h"

#include <cnoid/RootItem>
#include <cnoid/BodyItem>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace cnoid;

namespace teaching
{

  FollowTrajectoryController* FollowTrajectoryController::instance()
  {
    static FollowTrajectoryController* controller = new FollowTrajectoryController();
    return controller;
  }

  FollowTrajectoryController::FollowTrajectoryController()
  {
    registerCommands();
    setToolLink(0, "LARM_JOINT5");
    setToolLink(1, "RARM_JOINT5");
    name_ = "teaching_plugin";
    topic_name_ = "joint_trajectory";

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv;
      ros::init(argc, argv, name_);
    }

    node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(topic_name_, 1);
    spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner_->start();
  }


  bool FollowTrajectoryController::MoveArmCommand::operator()(std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    try {
      // transform xyz,rpy to WASIT-based
      // convert the rotation to matrix, then quaternion
      BodyPtr body = c_->getRobotBody();
      Link* waistLink = body->link("WAIST");

      Position waistToWristT; // Eigen::Transform
      Position wristT;
      wristT.linear() = rotFromRpy(rpy);
      wristT.translation() = xyz;
      waistToWristT = waistLink->T().inverse() * wristT;
      std::cout << waistToWristT.linear() << std::endl;
      std::cout << waistToWristT.translation() << std::endl;
      Eigen::Quaterniond quat(waistToWristT.linear());
      geometry_msgs::Pose target_pose1;
      target_pose1.orientation.x = quat.x();
      target_pose1.orientation.y = quat.y();
      target_pose1.orientation.z = quat.z();
      target_pose1.orientation.w = quat.w();
      target_pose1.position.x = waistToWristT.translation().x();
      target_pose1.position.y = waistToWristT.translation().y();
      target_pose1.position.z = waistToWristT.translation().z();

      // now send (quat, waistToWristT.translation())

      static const std::string RARM_GROUP = "right_arm_torso";
      static const std::string LARM_GROUP = "left_arm_torso";

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      namespace rvt = rviz_visual_tools;
      moveit_visual_tools::MoveItVisualTools visual_tools("RARM_JOINT5_Link");
      visual_tools.deleteAllMarkers();

      if (armID == 0) {
        moveit::planning_interface::MoveGroupInterface larm_group(LARM_GROUP);
        const robot_state::JointModelGroup* larm_joint_model_group =
          larm_group.getCurrentState()->getJointModelGroup(LARM_GROUP);
        larm_group.setPoseTarget(target_pose1);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (larm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        printLog("plan for target_pose1: ", success ? "SUCCEEDED" : "FAILED");

        visual_tools.publishAxisLabeled(target_pose1, "pose1");
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, larm_joint_model_group);
        visual_tools.trigger(); // Batch publishing

        // my_plan.trajectory_ : moveit_msgs::RobotTrajectory.msg
        trajectory_msgs::JointTrajectory& jt = my_plan.trajectory_.joint_trajectory;
        std::cout << "Number of joints: " << jt.joint_names.size() << std::endl;
        for (const auto& name : jt.joint_names) {
          std::cout << name << " ";
        }
        std::cout << std::endl;

        std::cout << "Number of points: " << jt.points.size() << std::endl;
        int np = 0;
        for (const auto& p : jt.points) {
          std::cout << "Point: " << np++ << "," << "Time: " << p.time_from_start << std::endl;
          for (const auto& x : p.positions) {
            std::cout << x << " ";
          }
          std::cout << std::endl;
        }

        larm_group.move();

        // ros::shutdown()
      } else {
        // moveit::planning_interface::MoveGroupInterface rarm_group(RARM_GROUP);
        // const robot_state::JointModelGroup* rarm_joint_model_group =
        //   rarm_group.getCurrentState()->getJointModelGroup(RARM_GROUP);
        // rarm_group.setPoseTarget(target_pose1);
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (rarm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      }

      // pack to ROS message
      // send it to move_group
      // receive the plan
      // and play it in Choreonoid
      // subscribe to /joint_state and update the local model


#if 0 // move the robot model explicitly in the simulator
      Link* base = body->rootLink();
      Link* wrist = body->link(c_->getToolLinkName(armID));

      JointPathPtr jointPath = getCustomJointPath(body, base, wrist);
      jointPath->calcForwardKinematics();

      c_->ci.clear();
      c_->ci.appendSample(0, wrist->p(), wrist->attitude());
      c_->ci.appendSample(duration, xyz, rotFromRpy(rpy));
      c_->ci.update();
      return c_->executeCartesianMotion(wrist, jointPath);
#endif
    } catch (...) {
      printLog("unknown armID: ", armID);
      return false;
    }
  }

  bool FollowTrajectoryController::sendTrajectory()
  {
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();

    BodyPtr body = getRobotBody();
    int n = body->numJoints();
    int num_points = 1;

    for (int i = 0; i < n; i++)
    {
      //std::cout << body->joint(i)->name() << std::endl;
      traj.joint_names.push_back(body->joint(i)->name());
    }

    traj.points.resize(num_points);
    for (int ind = 0; ind < num_points; ind++)
    {
      traj.points[ind].positions.resize(n);
      traj.points[ind].velocities.resize(n);
      for (int i = 0; i < n; i++)
      {
        traj.points[ind].positions[i] = body->joint(i)->q();
        traj.points[ind].velocities[i] = 0.0;
      }
      traj.points[ind].time_from_start = ros::Duration(1.0);
    }

    traj_pub_.publish(traj);

    // add time stamp and other required info
    // controllerをmoveitにする
    // sim => plan => 結果を受け取って可視化
    // real => execute （ボタン）
    // ハンド関節を除外して送る

#if 0
    // joint->name().c_str();
    //const trajectory_msgs::JointTrajectoryPoint* traj;
    //trajectory_msgs::JointTrajectoryPoint traj;
    trajectory_msgs::JointTrajectory traj;
    //traj.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj.header.stamp = ros::Time::now();

    traj.joint_names.push_back("rarm1_joint");
    traj.joint_names.push_back("rarm2_joint");
    traj.joint_names.push_back("rarm3_joint");

    traj.points.resize(2);
    int ind = 0;
    traj.points[ind].positions.resize(3);
    traj.points[ind].positions[0] = 0.0;
    traj.points[ind].positions[1] = 0.1;
    traj.points[ind].positions[2] = 0.2;
    traj.points[ind].velocities.resize(3);
    for (size_t j = 0; j < 3; ++j)
    {
      traj.points[ind].velocities[j] = 0.0;
    }
    traj.points[ind].time_from_start = ros::Duration(1.0);

    ind++;
    traj.points[ind].positions.resize(3);
    traj.points[ind].positions[0] = 1.0;
    traj.points[ind].positions[1] = 1.1;
    traj.points[ind].positions[2] = 1.2;
    traj.points[ind].velocities.resize(3);
    for (size_t j = 0; j < 3; ++j)
    {
      traj.points[ind].velocities[j] = 0.0;
    }
    traj.points[ind].time_from_start = ros::Duration(2.0);
    traj_pub_.publish(traj);
#endif

    // wait the completion of the execution
    // update the robot status in Scene View on the completion

  }

  void FollowTrajectoryController::registerCommands()
  {
    registerCommand("moveArm", "Arm", "boolean",
                    {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)},
                    new MoveArmCommand(this)); // 0=left, 1=right
  }

}
