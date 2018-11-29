/**
   @author Ryo Hanai
*/

#include <functional>
using namespace std::placeholders;

#include "I611Controller.h"

namespace teaching
{

  I611Controller* I611Controller::instance ()
  {
    static I611Controller* c = new I611Controller();
    return c;
  }

  void I611Controller::initialize ()
  {
    TPInterface& tpif = TPInterface::instance();
    // i611のロボットモデル、アームモデル依存情報の設定
    // yamlかBodyから抜き出せるかもしれない
    tpif.setToolLink(0, "arm1/Link6");
    tpif.setRobotName("main_withHands");
    tpif.setTimeStep(0.1); // i611の経由点は100ms or 200msステップとする

    // teachingPluginのコントローラがサポートするコマンドセットを定義
    // realとsimulatorで同じコマンドセットでなければならない
    setCommandSet(new SingleArmWithGripperCommandSet);

    // teachingPlugin上でのfake実行用のコントローラを設定する
    bindCommandFunction("moveArm", false, std::bind(&SingleArmFakeController::moveArm, fake_armc_, _1));
    bindCommandFunction("moveGripper", false, std::bind(&EZGripperFakeController::moveGripper, fake_gripperc_, _1));
    // armとgripperのgoInitialを呼ぶ関数を設定する
    // bindCommandFunction("goInitial", false, std::bind(&SingleArmFakeController::goInitial, fake_armc_, _1));

    // 外部コントローラを設定する
#ifdef ROS_ON
    bindCommandFunction("moveArm", true, std::bind(&SingleArmROSController::moveArm, ros_armc_, _1));
    // bindCommandFunction("moveGripper", false, std::bind(&EZGripperROSController::moveGripper, ros_gripperc_, _1));
    // bindCommandFunction("goInitial", true, std::bind(&SingleArmROSController::goInitial, rosc_, _1));
#endif

  }

}
