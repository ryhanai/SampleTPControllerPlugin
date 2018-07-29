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

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace teaching
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
  typedef std::shared_ptr<TrajClient> TrajClientPtr;

  class FollowTrajectoryControllerUR3Dual : public Controller
  {
  public:
    static FollowTrajectoryControllerUR3Dual* instance();

    bool sendTrajectory();
    void syncWithReal();
    void updateState(const sensor_msgs::JointState::ConstPtr& jointstate);
    bool interpolateJ(JointPathPtr jointPath, trajectory_msgs::JointTrajectory& traj);
    bool interpolate(Link* wrist, JointPathPtr jointPath, trajectory_msgs::JointTrajectory& traj);

  private:
    class MoveArmCommand : public Command
  {
    public:
      MoveArmCommand(FollowTrajectoryControllerUR3Dual* c) { c_ = c; }
      FollowTrajectoryControllerUR3Dual* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
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
    std::string rarm_topic_name_;
    std::string larm_topic_name_;
    std::string rhand_topic_name_;
    std::string lhand_topic_name_;
    boost::shared_ptr<ros::NodeHandle> node_;
    ros::Publisher rarm_traj_pub_;
    ros::Publisher larm_traj_pub_;
    ros::Publisher rhand_traj_pub_;
    ros::Publisher lhand_traj_pub_;

    TrajClientPtr rarm_traj_client_;
    TrajClientPtr larm_traj_client_;
    TrajClientPtr rhand_traj_client_;
    TrajClientPtr lhand_traj_client_;
    
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
