/**
   @author Ryo Hanai
*/

#include "SingleArmROSController.h"
#include "TPUtil.h"
#include "ROSUtil.h"

namespace teaching
{
  SingleArmROSController::SingleArmROSController ()
  {
    ROSInterface& rosif = ROSInterface::instance();
    rosif.addTrajectoryClient(0, "/left_arm/follow_joint_trajectory");
    rosif.addTrajectoryClient(1, "/right_arm/follow_joint_trajectory");
    rosif.addTrajectoryClient(2, "/left_hand/joint_trajectory_controller/follow_joint_trajectory");
    rosif.addTrajectoryClient(3, "/right_hand/joint_trajectory_controller/follow_joint_trajectory");

    // addListener("/joint_states", &FollowTrajectoryControllerUR3Dual::updateState);
  }

  bool SingleArmROSController::moveArm (std::vector<CompositeParamType>& params)
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
      ROSInterface::instance().followTrajectory(armID, traj);
    } else {
      return false;
    }
  }

  bool SingleArmROSController::goInitial (std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);
    printLog("goInitial(", duration, ")");

    TPInterface& tpif = TPInterface::instance();
    VectorXd qGoal = tpif.getStandardPose();

    printLog("not yet implemented");
    return false;
  }

}
