/**
   @author Ryo Hanai
*/

#include <functional>
using namespace std::placeholders;

#include "HiroNXController.h"

namespace teaching
{

  HiroNXController* HiroNXController::instance ()
  {
    static HiroNXController* c = new HiroNXController();
    return c;
  }

  void HiroNXController::initialize ()
  {
    TPInterface& tpif = TPInterface::instance();
    tpif.setToolLink(0, "LARM_JOINT5");
    tpif.setToolLink(1, "RARM_JOINT5");
    tpif.setRobotName("main_withHands");

    setCommandSet(new HiroNXCommandSet);

    bindCommandFunction("moveArm", false, std::bind(&SingleArmFakeController::moveArm, fake_armc_, _1));

    bindCommandFunction("moveTorso", false, std::bind(&HiroNXFakeController::moveTorso, fake_nxc_, _1));
    bindCommandFunction("moveHead", false, std::bind(&HiroNXFakeController::moveHead, fake_nxc_, _1));
    bindCommandFunction("moveBothArms", false, std::bind(&HiroNXFakeController::moveBothArms, fake_nxc_, _1));

    bindCommandFunction("moveGripper", false, std::bind(&HiroNXFakeController::moveGripper, fake_nxc_, _1));
    bindCommandFunction("goInitial", false, std::bind(&SingleArmFakeController::goInitial, fake_armc_, _1));

#ifdef ROS_ON
    bindCommandFunction("moveArm", true, std::bind(&SingleArmROSController::moveArm, ros_armc_, _1));
    bindCommandFunction("moveGripper", false, std::bind(&RobotiqGripperROSController::moveGripper, ros_gripperc_, _1));
    // bindCommandFunction("goInitial", true, std::bind(&SingleArmROSController::goInitial, rosc_, _1));
#endif

  }

}
