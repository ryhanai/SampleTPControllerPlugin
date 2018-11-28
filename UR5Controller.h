/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "SingleArmWithGripperCommandSet.h"
#include "SingleArmFakeController.h"
#include "RobotiqGripperFakeController.h"

namespace teaching
{

  class UR5Controller : public Controller
  {
  public:
    static UR5Controller* instance();
    void initialize ();

  private:
    SingleArmFakeController fake_armc_;
    RobotiqGripperFakeController fake_gripperc_;

#ifdef ROS_ON
    SingleArmROSController ros_armc_;
    RobotiqGripperFakeController fake_gripperc_;
#endif

  };

}
