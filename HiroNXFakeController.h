/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class HiroNXFakeController
  {
  public:
    bool moveTorso (std::vector<CompositeParamType>& params);
    bool moveHead (std::vector<CompositeParamType>& params);
    bool moveBothArms (std::vector<CompositeParamType>& params);
    bool moveGripper (std::vector<CompositeParamType>& params);

    void setJointPath (JointPathPtr jointPath) { jointPath_ = jointPath; }

  private:
    cnoid::JointPathPtr jointPath_;
  };

}
