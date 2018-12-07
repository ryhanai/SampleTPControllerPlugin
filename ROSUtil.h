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

    void sendGoal (TrajClientPtr client, const JointTrajectory& traj);
    bool wait (TrajClientPtr client);
    void updateState (const sensor_msgs::JointState::ConstPtr& jointstate);
    void syncWithReal ();
    void cancelFollowTrajectory ();

    boost::shared_ptr<ros::NodeHandle> getNodeHandle () { return node_; }

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
