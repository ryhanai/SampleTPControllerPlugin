/**
   @author Ryo Hanai
*/

#include "HiroNXFakeController.h"
#include "TPUtil.h"

namespace teaching
{

  bool HiroNXFakeController::moveTorso (std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    TPInterface& tpif = TPInterface::instance();
    JointTrajectory traj;
    if (tpif.interpolate(jointPath_, xyz, rpy, duration, traj)) {
      return tpif.followTrajectory(armID, traj);
    }

    return false;
  }

  bool HiroNXFakeController::moveHead (std::vector<CompositeParamType>& params)
  {

    return true;
  }

  bool HiroNXFakeController::moveBothArms (std::vector<CompositeParamType>& params)
  {

    return true;
  }

  bool HiroNXFakeController::moveGripper (std::vector<CompositeParamType>& params)
  {

    return true;
  }

}
