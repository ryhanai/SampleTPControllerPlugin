/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace teaching
{

  class FollowTrajectoryController : public Controller
  {
  public:
    static FollowTrajectoryController* instance();

    bool sendTrajectory();
  private:
    class MoveArmCommand : public Command
    {
    public:
      MoveArmCommand(FollowTrajectoryController* c) { c_ = c; }
      FollowTrajectoryController* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };


  private:
    FollowTrajectoryController();
    void registerCommands ();
    /* bool executeDualArmMotion(); */
    /* bool executeGripperMotion (const std::vector<std::string>& gripperLinks, double width); */
    std::string RARM_GROUP;
    std::string LARM_GROUP;

  protected:
    std::string name_;
    std::string topic_name_;
    boost::shared_ptr<ros::NodeHandle> node_;
    ros::Publisher traj_pub_;
    boost::shared_ptr<ros::AsyncSpinner> spinner_;
  };

}
