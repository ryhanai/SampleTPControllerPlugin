/**
   @author Ryo Hanai
*/

#include "UR3dualFakeController.h"

bool UR3dualFakeController::UR3dualFakeController()
{
  // initialize sub-controllers
  rarmc_.setRobotItem();
  larmc_.setRobotItem();
  rgripperc_.setRobotItem();
  lgripperc_.setRobotItem();
}

bool UR3dualFakeController::moveArm (std::vector<CompositeParamType>& params)
{
  // unbox armID
  int armID = boost::get<int>(params[3]);

  // delegate to sub-controllers
  switch (armID) {
  case 0:
    return larmc_.moveArm(params)
    break;
  case 1:
    return rarmc_.moveArm(params)
    break;
  default:
    printLog("unknown armID");
    return false; // コマンド実行結果のfalse特別した方がよい
  }
}

bool UR3dualFakeController::moveGripper (std::vector<CompositeParamType>& params)
{
  int gripperID = boost::get<int>(params[2]);

  switch (gripperID) {
  case 0:
    return lgripper_.moveGripper(params);
  case 1:
    return lgripper_.moveGripper(params);
  default:
    printLog("unknown gripperID");
    return false; // コマンド実行結果のfalse特別した方がよい
  }
}

bool UR3dualFakeController::goInitial (std::vector<CompositeParamType>& params)
{
  // split joint vector into arm-arts and gripper parts
  // sub-controllersでの実行は非同期にした方が良いか？
  // ただし、非同期にすると実行タイミングの再現性が無くなる
  return rarmc_.goInitial() && larmc_.goInitial()
    && rgripperc_.goInitial() && lgrippermc_.goInitial();
}

void UR3dualFakeController::registerCommands ()
{
  registerCommand("moveArm", "Arm", "boolean",
                  {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)}, moveArm); // 0=left, 1=right
  registerCommand("moveGripper", "Gripper", "boolean",
                  {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)},moveGripper); // 0=left, 1=right
  registerCommand("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)}, goInitial);
}
