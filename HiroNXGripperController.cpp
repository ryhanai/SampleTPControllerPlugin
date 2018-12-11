/**
   @author Ryo Hanai
*/

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif
#include <QCoreApplication>

#include "HiroNXGripperController.h"

namespace teaching
{

  bool HiroNXGripperController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ",", gripperID, ")");

    TPInterface& tpif = TPInterface::instance();
    BodyPtr body = tpif.getRobotBody();

    // IK
    const double th = asin(((width/2.0) - 0.015) / 0.042);

    JointTrajectory traj;
    VectorXd qGoal(4);
    qGoal[0] = th;
    qGoal[1] = -th;
    qGoal[2] = -th;
    qGoal[3] = th;
    if (tpif.interpolate(gripperJoints_, qGoal, duration, traj)) {
      if (isReal) {
#ifdef ROS_ON
        ROSInterface::instance().sendGoal(client_, traj);
        return waitAndUpdateRobotModel();
#endif
      } else {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }

  bool HiroNXGripperController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    printLog("not yet implemented");
    return true;
  }

}
