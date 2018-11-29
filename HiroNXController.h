/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "HiroNXCommandSet.h"
#include "SingleArmFakeController.h"
#include "HiroNXFakeController.h"

#ifdef ROS_ON
#include "SingleArmROSController.h"
#endif

namespace teaching
{

  class HiroNXController : public Controller
  {
  public:
    static HiroNXController* instance ();
    void initialize ();

  private:
    SingleArmFakeController fake_armc_;
    HiroNXFakeController fake_nxc_;
    // RobotiqGripperFakeController fake_gripperc_;

#ifdef ROS_ON
    SingleArmROSController ros_armc_;
#endif

  };

}
