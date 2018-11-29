/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class SingleArmROSController
  {
  public:
    SingleArmROSController ();
    bool moveArm (std::vector<CompositeParamType>& params);
    bool goInitial (std::vector<CompositeParamType>& params);
  };

}
