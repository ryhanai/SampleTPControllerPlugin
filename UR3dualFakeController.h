/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "UR3dualControllerIF.h"

namespace teaching
{

  class UR3dualFakeController : public Controller, public UR3dualControllerIF
  {
  public:
    static UR3dualFakeController* instance();

    bool moveArm (std::vector<CompositeParamType>& params) override;
    bool moveGripper (std::vector<CompositeParamType>& params) override;
    bool goInitial (std::vector<CompositeParamType>& params) override;

  private:
    UR3dualFakeController();

    SingleArmController rarmc_;
    SingleArmController larmc_;
    RobotiqGripper rgripperc_;
    RobotiqGripper lgripperc_;
  };

}
