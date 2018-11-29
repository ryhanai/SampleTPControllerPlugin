/**
   @author Ryo Hanai
*/

#include <functional>
using namespace std::placeholders;

#include "UR3dualController.h"

namespace teaching
{

  UR3dualController* UR3dualController::instance ()
  {
    static UR3dualController* c = new UR3dualController();
    return c;
  }

  void UR3dualController::initialize ()
  {
    TPInterface& tpif = TPInterface::instance();
    tpif.setToolLink(0, "larm_wrist_3_joint");
    tpif.setToolLink(1, "rarm_wrist_3_joint");
    tpif.setRobotName("main_withHands");

    setCommandSet(new SingleArmWithGripperCommandSet);

    bindCommandFunction("moveArm", false, std::bind(&SingleArmFakeController::moveArm, fake_armc_, _1));
    bindCommandFunction("moveGripper", false, std::bind(&RobotiqGripperFakeController::moveGripper, fake_gripperc_, _1));

    // モデルが統合されているので、fake実行の場合はどのコントローラのgoInitialを呼んでも同じ
    bindCommandFunction("goInitial", false, std::bind(&SingleArmFakeController::goInitial, fake_armc_, _1));

#ifdef ROS_ON
    bindCommandFunction("moveArm", true, std::bind(&SingleArmROSController::moveArm, ros_armc_, _1));
    // bindCommandFunction("moveGripper", false, std::bind(&RobotiqGripperROSController::moveGripper, ros_gripperc_, _1));
    // bindCommandFunction("goInitial", true, std::bind(&SingleArmROSController::goInitial, rosc_, _1));
#endif

  }

}
