/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerManager.h"

#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace teaching
{

  class FollowTrajectoryController : public ControllerBase
  {
  public:
    static FollowTrajectoryController* instance();
    FollowTrajectoryController();
    bool sendTrajectory();

  protected:
    std::string name_;
    std::string topic_name_;
    boost::shared_ptr<ros::NodeHandle> node_;
    ros::Publisher traj_pub_;

  };

}
