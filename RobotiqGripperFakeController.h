/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class RobotiqGripperFakeController
  {
  public:
    bool moveGripper (std::vector<CompositeParamType>& params);
    bool goInitial (std::vector<CompositeParamType>& params);
  };

}
