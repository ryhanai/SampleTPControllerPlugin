/**
   @author Ryo Hanai
*/

#include "SingleArmFakeController.h"

#include <functional>
using namespace std::placeholders;

namespace teaching
{

  SingleArmFakeController* SingleArmFakeController::instance()
  {
    static SingleArmFakeController* controller = new SingleArmFakeController();
    return controller;
  }

  SingleArmFakeController::SingleArmFakeController()
  {
    registerCommand("moveArm", "Arm", "boolean",
                    {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)},
                    std::bind(&SingleArmFakeController::moveArm, this, _1)); // 0=left, 1=right
    registerCommand("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)},
                    std::bind(&SingleArmFakeController::goInitial, this, _1));

    //setToolLink(0, "larm_wrist_3_joint"); // YAMLファイルから取得する方が良い？
    setToolLink(0, "LARM_JOINT5"); // YAMLファイルから取得する方が良い？
  }

  bool SingleArmFakeController::moveArm (std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    Trajectory traj;
    if (interpolate(armID, xyz, rpy, duration, traj)) {
      return followTrajectory(armID, traj);
    }

    return false;
  }

  bool SingleArmFakeController::goInitial (std::vector<CompositeParamType>& params)
  {
    return false;
  }

}
