/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "HiroNXCommandSet.h"
#include "SingleArmController.h"
#include "HiroNXGripperController.h"
#include "ObjectPoseSensor.h"

namespace teaching
{

  class HiroNXController : public Controller
  {
  public:
    static HiroNXController* instance ();
    void initialize ();

  private:
    bool moveArm (std::vector<CompositeParamType>& params, bool isReal);
    bool moveGripper (std::vector<CompositeParamType>& params, bool isReal);
    bool goInitial (std::vector<CompositeParamType>& params, bool isReal);

    // extension for HiroNX
    bool moveTorso (std::vector<CompositeParamType>& params, bool isReal);
    bool moveHead (std::vector<CompositeParamType>& params, bool isReal);
    bool moveBothArms (std::vector<CompositeParamType>& params, bool isReal);

    bool recognize (std::vector<CompositeParamType>& params, bool isReal);
    bool recognize_double (std::vector<CompositeParamType>& params, bool isReal);
    bool recognize_int (std::vector<CompositeParamType>& params, bool isReal);

    SingleArmController rarmc_;
    SingleArmController larmc_;
    SingleArmController rarm_no_waistc_;
    SingleArmController larm_no_waistc_;
    HiroNXGripperController rgripperc_;
    HiroNXGripperController lgripperc_;
    ObjectPoseSensor sensor_;

  };

}
