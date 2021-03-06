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

  class SingleArmController
  {
  public:
    bool moveL (std::vector<CompositeParamType>& params, bool isReal);
    bool moveJ (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    void setJointPathName (std::string jointPathName) { jointPathName_ = jointPathName; }

  private:
    std::string jointPathName_;

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
