/**
   @author Ryo Hanai
*/

#include <functional>
using namespace std::placeholders;

#include "UR5Controller.h"

namespace teaching
{

  UR5Controller* UR5Controller::instance ()
  {
    static UR5Controller* c = new UR5Controller();
    return c;
  }

  void UR5Controller::initialize ()
  {
    TPInterface& tpif = TPInterface::instance();
    tpif.setRobotName("main_withHands");
    tpif.setToolLink(0, "wrist_3_joint");

    setCommandSet(new SingleArmWithGripperCommandSet);

    bindCommandFunction("moveArm", false, std::bind(&SingleArmFakeController::moveArm, fake_armc_, _1));
    bindCommandFunction("moveGripper", false, std::bind(&RobotiqGripperFakeController::moveGripper, fake_gripperc_, _1));
    bindCommandFunction("goInitial", false, std::bind(&SingleArmFakeController::goInitial, fake_armc_, _1));

#ifdef ROS_ON
    bindCommandFunction("moveArm", true, std::bind(&SingleArmROSController::moveArm, ros_armc_, _1));
    bindCommandFunction("moveGripper", true, std::bind(&RobotiqGripperROSController::moveGripper, ros_gripperc_, _1));
    bindCommandFunction("goInitial", true, std::bind(&SingleArmROSController::goInitial, ros_armc_, _1));
#endif
  }

}
