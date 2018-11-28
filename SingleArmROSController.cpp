/**
   @author Ryo Hanai
*/

#include "SingleArmROSController.h"
#include "TPUtil.h"

namespace teaching
{

  bool SingleArmROSController::moveArm (std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    printLog("not yet implemented");
    return false;
  }

  bool SingleArmROSController::goInitial (std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);
    printLog("goInitial(", duration, ")");

    TPInterface& tpif = TPInterface::instance();
    VectorXd qGoal = tpif.getStandardPose();
    Trajectory traj;

    printLog("not yet implemented");
    return false;
  }

}
