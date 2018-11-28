/**
   @author Ryo Hanai
*/

#include "EZGripperFakeController.h"
#include "TPUtil.h"

namespace teaching
{

  bool EZGripperFakeController::moveGripper (std::vector<CompositeParamType>& params)
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

  bool EZGripperFakeController::goInitial (std::vector<CompositeParamType>& params)
  {
    printLog("not yet implemented");
    return true;
  }

#if 0
  EZGripperFakeController::EZGripperFakeController()
  {
    registerCommandFunction("moveGrppper", std::bind(&SimpleGripperFakeController::moveGripper, this, _1));

    gjoints_.push_back(std::make_tuple("arm1_ezgripper_finger_L1_1", [](double x){return x;}));
    gjoints_.push_back(std::make_tuple("arm1_ezgripper_finger_L2_1", [](double x){return 111.2-x;}));
    gjoints_.push_back(std::make_tuple("arm1_ezgripper_finger_L1_2", [](double x){return x;}));
    gjoints_.push_back(std::make_tuple("arm1_ezgripper_finger_L2_2", [](double x){return 111.2-x;}));

    // gjoints_.push_back(std::make_tuple("lgripper_finger1_finger_tip_joint", [](x){return -x;}));
    // gjoints_.push_back(std::make_tuple("lgripper_finger1_inner_knuckle_joint", [](x){return -x;}));
    // gjoints_.push_back(std::make_tuple("lgripper_finger1_joint", [](x){return -x;}));
    // gjoints_.push_back(std::make_tuple("lgripper_finger2_finger_tip_joint", [](x){return -x;}));
    // gjoints_.push_back(std::make_tuple("lgripper_finger2_inner_knuckle_joint", [](x){return x;}));
    // gjoints_.push_back(std::make_tuple("lgripper_finger2_joint", [](x){return x;}));
  }
#endif


}
