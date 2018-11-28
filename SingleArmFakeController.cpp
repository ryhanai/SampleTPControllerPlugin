/**
   @author Ryo Hanai
*/

#include "SingleArmFakeController.h"

namespace teaching
{

  bool SingleArmFakeController::moveArm (std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    TPInterface& tpif = TPInterface::instance();
    Trajectory traj;
    if (tpif.interpolate(armID, xyz, rpy, duration, traj)) {
      return tpif.followTrajectory(armID, traj);
    }

    return false;
  }

  bool SingleArmFakeController::goInitial (std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);
    printLog("goInitial(", duration, ")");

    TPInterface& tpif = TPInterface::instance();
    VectorXd qGoal = tpif.getStandardPose();
    Trajectory traj;
    if (tpif.interpolate(qGoal, duration, traj)) {
      return tpif.followTrajectory(traj);
    }

    return false;
  }

}
