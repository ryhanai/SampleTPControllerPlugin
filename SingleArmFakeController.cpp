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
    registerCommandFunction("moveArm", std::bind(&SingleArmFakeController::moveArm, this, _1));
    registerCommandFunction("goInitial", std::bind(&SingleArmFakeController::goInitial, this, _1));
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
    if (tpif_->interpolate(armID, xyz, rpy, duration, traj)) {
      return tpif_->followTrajectory(armID, traj);
    }

    return false;
  }

  bool SingleArmFakeController::goInitial (std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);
    printLog("goInitial(", duration, ")");

    VectorXd qGoal = tpif_->getStandardPose();
    Trajectory traj;
    if (tpif_->interpolate(qGoal, duration, traj)) {
      return tpif_->followTrajectory(traj);
    }

    return false;
  }

}
