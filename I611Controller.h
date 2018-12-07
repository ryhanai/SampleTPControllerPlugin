/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "SingleArmWithGripperCommandSet.h"
#include "SingleArmController.h"
#include "EZGripperController.h"

namespace teaching
{

  class I611Controller : public Controller
  {
  public:
    static I611Controller* instance ();
    void initialize ();

  private:
    bool moveArm (std::vector<CompositeParamType>& params, bool isReal);
    bool moveGripper (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    SingleArmController armc_;
    EZGripperController gripperc_;

  };

}
