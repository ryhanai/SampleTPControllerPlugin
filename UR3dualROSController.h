/**
   @author Ryo Hanai
*/

#pragma once

#include "UR3dualControllerIF.h"

namespace teaching
{

  class UR3dualROSController : public UR3dualControllerIF
  {
  public:
    static UR3dualROSController* instance();

    bool moveArm (std::vector<CompositeParamType>& params) override;
    bool moveGripper (std::vector<CompositeParamType>& params) override;
    bool goInitial (std::vector<CompositeParamType>& params) override;

  private:
    UR3dualROSController();
  };

}
