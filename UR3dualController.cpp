/**
   @author Ryo Hanai
*/

#include <functional>
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
    tpif.setRobotName("main_withHands");
    // attach/detach対象の登録は冗長だが、
    // teachingPlugin側からattach/detachを呼ぶ仕様になっているため
    // jointPath指定とは別になっている。
    tpif.setToolLink(0, "larm_wrist_3_joint");
    tpif.setToolLink(1, "rarm_wrist_3_joint");

    setCommandSet(new SingleArmWithGripperCommandSet);

    // setup low-level controllers
    larmc_.setJointPathName("larm_wrist_3_joint");
    rarmc_.setJointPathName("rarm_wrist_3_joint");
    lgripperc_.setGripperJoints(
      {"lgripper_finger1_finger_tip_joint",
          "lgripper_finger1_inner_knuckle_joint",
          "lgripper_finger1_joint",
          "lgripper_finger2_finger_tip_joint",
          "lgripper_finger2_inner_knuckle_joint",
          "lgripper_finger2_joint"}
      );
    lgripperc_.setGripperDriverJoint("lhand_right_driver_joint");
    rgripperc_.setGripperJoints(
      {"rgripper_finger1_finger_tip_joint",
          "rgripper_finger1_inner_knuckle_joint",
          "rgripper_finger1_joint",
          "rgripper_finger2_finger_tip_joint",
          "rgripper_finger2_inner_knuckle_joint",
          "rgripper_finger2_joint"}
      );
    rgripperc_.setGripperDriverJoint("rhand_right_driver_joint");

    // コマンドの実装関数の登録
    bindCommandFunction("moveArm", std::bind(&UR3dualController::moveArm, this,
                                             std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveGripper", std::bind(&UR3dualController::moveGripper, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("goInitial", std::bind(&UR3dualController::goInitial, this,
                                               std::placeholders::_1, std::placeholders::_2));

#ifdef ROS_ON
    ROSInterface& rosif = ROSInterface::instance(); // ros::init() etc.
    larmc_.setTrajectoryActionClient("/left_arm/follow_joint_trajectory");
    rarmc_.setTrajectoryActionClient("/right_arm/follow_joint_trajectory");
    lgripperc_.setTrajectoryActionClient("/left_hand/joint_trajectory_controller/follow_joint_trajectory");
    rgripperc_.setTrajectoryActionClient("/right_hand/joint_trajectory_controller/follow_joint_trajectory");
    larmc_.setJointStateListener("/left_arm/joint_states");
    rarmc_.setJointStateListener("/right_arm/joint_states");
    lgripperc_.setJointStateListener("/left_hand/joint_states");
    rgripperc_.setJointStateListener("/right_hand/joint_states");
#endif

  }

  bool UR3dualController::moveArm (std::vector<CompositeParamType>& params, bool isReal)
  {
    int armID = boost::get<int>(params[3]);

    // タスク記述層のパラメータとlow-level controllerの対応付けを記述
    switch (armID) {
    case 0:
      return larmc_.moveL(params, isReal);
      break;
    case 1:
      return rarmc_.moveL(params, isReal);
      break;
    default:
      printLog("undefined armID");
      return false;
    }
  }

  bool UR3dualController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
  {
    int gripperID = boost::get<int>(params[2]);

    switch (gripperID) {
    case 0:
      return lgripperc_.moveGripper(params, isReal);
      break;
    case 1:
      return rgripperc_.moveGripper(params, isReal);
      break;
    default:
      printLog("undefined gripperID");
      return false;
    }
  }

  bool UR3dualController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    // yamlから取得したjoint anglesを分割し、
    // 各low-level controllerのmoveJointsを呼ぶ

    return false;
  }

}
