/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class HiroNXGripperController
  {
  public:
    bool moveGripper (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    void setGripperJoints (std::vector<std::string> joints) { gripperJoints_ = joints; }

  private:
    std::vector<std::string> gripperJoints_;

  };

}
