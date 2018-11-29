/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "SingleArmWithGripperCommandSet.h"
#include "SingleArmFakeController.h"
#include "EZGripperFakeController.h"

#ifdef ROS_ON
#include "SingleArmROSController.h"
#endif

namespace teaching
{

  class I611Controller : public Controller
  {
  public:
    static I611Controller* instance ();
    void initialize ();

  private:
    SingleArmFakeController fake_armc_;
    EZGripperFakeController fake_gripperc_;

#ifdef ROS_ON
    SingleArmROSController ros_armc_;
#endif

  };

}
