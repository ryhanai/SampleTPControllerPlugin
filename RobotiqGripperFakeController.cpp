/**
   @author Ryo Hanai
*/

#include "RobotiqGripperFakeController.h"
#include "TPUtil.h"

namespace teaching
{

  bool RobotiqGripperFakeController::moveGripper (std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    printLog("moveGripper(", width, ", ", duration, ")");

    printLog("not yet implemented");

#if 0
    BodyPtr body = tpif_->getRobotBody();
    VectorXd qCur = tpif_->getCurrentJointAngles();

    cnoid::Interpolator<cnoid::VectorXd> ji;
    ji.appendSample(0, qCur);

    // width -> theta (適当)
    const double th = 111.2 - width * 10.0;

    for (auto gj : gjoints_) {
      qCur[body->link(std::get<0>(gj))->jointId()] = std::get<1>(gj)(th);
    }

    ji.appendSample(duration, qCur);
    ji.update();
#endif

    return true;

  }

  bool RobotiqGripperFakeController::goInitial (std::vector<CompositeParamType>& params)
  {
    printLog("not yet implemented");
    return true;
  }

#if 0
  RobotiqGripperFakeController::RobotiqGripperFakeController()
  {
    registerCommandFunction("moveGrppper", std::bind(&SimpleGripperFakeController::moveGripper, this, _1));

  }
#endif


}
