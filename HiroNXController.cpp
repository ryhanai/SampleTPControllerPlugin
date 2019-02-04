/**
   @author Ryo Hanai
*/

#include <functional>
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
    tpif.setRobotName("main_withHands");
    tpif.setToolLink(0, "LARM_JOINT5");
    tpif.setToolLink(1, "RARM_JOINT5");

    setCommandSet(new HiroNXCommandSet);

    // setup low-level controllers
    larmc_.setJointPathName("LARM_JOINT5");
    rarmc_.setJointPathName("RARM_JOINT5");
    lgripperc_.setGripperJoints({"LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3"});
    rgripperc_.setGripperJoints({"RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3"});

    // コマンドの実装関数の登録
    bindCommandFunction("moveArm", std::bind(&HiroNXController::moveArm, this,
                                             std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveGripper", std::bind(&HiroNXController::moveGripper, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("goInitial", std::bind(&HiroNXController::goInitial, this,
                                               std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveTorso", std::bind(&HiroNXController::moveTorso, this,
                                               std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveHead", std::bind(&HiroNXController::moveHead, this,
                                              std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("moveBothArms", std::bind(&HiroNXController::moveBothArms, this,
                                                  std::placeholders::_1, std::placeholders::_2));

    bindCommandFunction("recognize", std::bind(&ObjectPoseSensor::recognize, sensor_,
                                               std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("recognize_double", std::bind(&ObjectPoseSensor::recognize_double, sensor_,
                                                      std::placeholders::_1, std::placeholders::_2));
    bindCommandFunction("recognize_int", std::bind(&ObjectPoseSensor::recognize_int, sensor_,
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

  bool HiroNXController::moveArm (std::vector<CompositeParamType>& params, bool isReal)
  {
    int armID = boost::get<int>(params[3]);

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

  bool HiroNXController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
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

  bool HiroNXController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    return larmc_.goInitial(params, isReal);
  }

  bool HiroNXController::moveTorso (std::vector<CompositeParamType>& params, bool isReal)
  {
    return false;
  }

  bool HiroNXController::moveHead (std::vector<CompositeParamType>& params, bool isReal)
  {
    return false;
  }

  bool HiroNXController::moveBothArms (std::vector<CompositeParamType>& params, bool isReal)
  {
    return false;
  }


}
