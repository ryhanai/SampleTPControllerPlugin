/**
   @author Ryo Hanai
*/

#include <functional>
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
    tpif.setRobotName("main_withHands");
    tpif.setToolLink(0, "arm1/Link6");

    tpif.setTimeStep(0.1); // i611の経由点は100ms or 200msステップとする

    setCommandSet(new SingleArmWithGripperCommandSet);

    armc_.setJointPathName("arm1/Link6"); // base->arm1/Link6のjoint pathを割当てる

    gripperc_.setGripperJoints(
      {"arm1_ezgripper_finger_L1_1",
          "arm1_ezgripper_finger_L2_1",
          "arm1_ezgripper_finger_L1_2",
          "arm1_ezgripper_finger_L2_2"}
      );
    gripperc_.setGripperDriverJoint("arm1_ezgripper_finger_L1_1");

    // アーム、グリッパともに1つしか持っていないので
    // そのまま下位Controllerに割当てる
    bindCommandFunction("moveArm", std::bind(&SingleArmController::moveL, armc_,
                                             std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveGripper", std::bind(&EZGripperController::moveGripper, gripperc_,
                                                 std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("goInitial", std::bind(&I611Controller::goInitial, this,
                                               std::placeholders::_1, std::placeholders::_2));

#ifdef ROS_ON
    ROSInterface& rosif = ROSInterface::instance(); // ros::init() etc.
    armc_.setTrajectoryActionClient("/joint_trajectory/action");
    //gripperc_.setTrajectoryActionClient("/left_hand/joint_trajectory_controller/follow_joint_trajectory");
    // グリッパはtrajectoryでなく、GripperCommandAction
    armc_.setJointStateListener("/joint_states");
    // gripperc_.setJointStateListener("/gripper/joint_states");
    // グリッパも/joint_statesは出ている？アームと同じトピック？
#endif
  }

  bool I611Controller::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    // Fake実行の場合は、
    // armc_のgoInitialがモデルファイルで定義された初期姿勢を
    // 実現するためgripperc_のgoInitialを呼ぶ必要がない
    armc_.goInitial(params, isReal);
    // gripperc_.goInitial(params, isReal);
    return false;
  }
  
}
