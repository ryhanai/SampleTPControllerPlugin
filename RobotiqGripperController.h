/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

#ifdef ROS_ON
#include "ROSUtil.h"
#endif

namespace teaching
{

  class RobotiqGripperController
  {
  public:
    bool moveGripper (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    void setGripperJoints (std::vector<std::string> joints) { gripperJoints_ = joints; }
    void setGripperDriverJoint (std::string joint) { driverJoint_ = joint; }

  private:
    std::vector<std::string> gripperJoints_;
    std::string driverJoint_;

#ifdef ROS_ON
  public:
    void setTrajectoryActionClient (const std::string& actionName);
    void setJointStateListener (const std::string& topicName);

  private:
    void jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate);
    void updateRobotModel ();
    bool waitAndUpdateRobotModel ();
    TrajClientPtr client_;
    std::map<std::string, double> joint_state_;
#endif
  };

}
