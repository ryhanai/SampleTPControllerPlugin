/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <map>

namespace teaching
{

  class FollowTrajectoryControllerUR3Dual : public Controller
  {
  public:
    static FollowTrajectoryControllerUR3Dual* instance();

    bool sendTrajectory();
    void syncWithReal();
    void updateState(const sensor_msgs::JointState::ConstPtr& jointstate);

  private:
    class MoveArmCommand : public Command
  {
    public:
      MoveArmCommand(FollowTrajectoryControllerUR3Dual* c) { c_ = c; }
      FollowTrajectoryControllerUR3Dual* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
      bool doMove(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group,
                  geometry_msgs::Pose& target_pose,
                  const std::string& arm_group_name);

    };

  class MoveGripperCommand : public Command
    {
    public:
      MoveGripperCommand(FollowTrajectoryControllerUR3Dual* c) { c_ = c; }
      FollowTrajectoryControllerUR3Dual* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

  class GoInitialCommand : public Command
    {
    public:
      GoInitialCommand(FollowTrajectoryControllerUR3Dual* c) { c_ = c; }
      FollowTrajectoryControllerUR3Dual* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

  private:
    FollowTrajectoryControllerUR3Dual();
    void registerCommands ();

    static constexpr const char* RARM_GROUP = "right_arm";
    static constexpr const char* LARM_GROUP = "left_arm";
    static constexpr const char* RHAND_GROUP = "right_hand";
    static constexpr const char* LHAND_GROUP = "left_hand";

  protected:
    std::string name_;
    std::string topic_name_;
    boost::shared_ptr<ros::NodeHandle> node_;
    ros::Publisher traj_pub_;
    ros::Subscriber js_sub_;
    boost::shared_ptr<ros::AsyncSpinner> spinner_;
    boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> rarm_group_;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> larm_group_;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> rhand_group_;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> lhand_group_;
    std::map<std::string, double> joint_state_;
  };

}
