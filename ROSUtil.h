/**
   @author Ryo Hanai
*/

#pragma once

#include <map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include "TPUtil.h"

namespace teaching
{

  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
  typedef std::shared_ptr<TrajClient> TrajClientPtr;

  class ROSInterface
  {
  public:
    static ROSInterface& instance ();

    void addTrajectoryClient (int id, std::string action_name);
    void setJointStateTopic (std::string topic_name);

    bool followTrajectory (const Trajectory& traj);
    bool followTrajectory (int toolNumber, const Trajectory& traj);
    void updateState (const sensor_msgs::JointState::ConstPtr& jointstate);
    void syncWithReal ();
    void cancelFollowTrajectory ();

  private:
    ROSInterface ();
    std::string node_name_;
    boost::shared_ptr<ros::NodeHandle> node_;
    ros::Subscriber js_sub_;
    boost::shared_ptr<ros::AsyncSpinner> spinner_;
    std::map<std::string, double> joint_state_;
    std::map<int, TrajClientPtr> clientTable_;
  };

}
