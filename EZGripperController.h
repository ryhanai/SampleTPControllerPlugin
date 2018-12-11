/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

#ifdef ROS_ON
#include <control_msgs/GripperCommandAction.h>
#include "ROSUtil.h"
#endif

namespace teaching
{

#ifdef ROS_ON
  typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperActionClient;
  typedef std::shared_ptr<GripperActionClient> GripperActionClientPtr;
#endif

  class EZGripperController
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
    void setGripperActionClient (const std::string& actionName);
    void setJointStateListener (const std::string& topicName);

  private:
    void jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate);
    void updateRobotModel ();
    bool waitAndUpdateRobotModel ();
    GripperActionClientPtr client_;
    std::map<std::string, double> joint_state_;
#endif
  };

}
