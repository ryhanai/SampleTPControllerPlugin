/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "SingleArmWithGripperCommandSet.h"
#include "SingleArmController.h"
#include "RobotiqGripperController.h"

namespace teaching
{

  class UR3dualController : public Controller
  {
  public:
    static UR3dualController* instance ();
    void initialize ();

  private:
    bool moveArm (std::vector<CompositeParamType>& params, bool isReal);
    bool moveGripper (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    SingleArmController rarmc_;
    SingleArmController larmc_;
    RobotiqGripperController rgripperc_;
    RobotiqGripperController lgripperc_;

  };

}
