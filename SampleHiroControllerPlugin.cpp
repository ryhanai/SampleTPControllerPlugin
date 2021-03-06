/**
   @author Ryo Hanai
*/

#include <cnoid/Plugin>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>

#include "I611Controller.h"
// #include "UR5Controller.h"
#include "UR3dualController.h"
#include "HiroNXController.h"

#include "ControllerManager.h"

using namespace cnoid;
using namespace boost;
using namespace teaching;

class SampleHiroControllerPlugin : public Plugin
{
public:

  SampleHiroControllerPlugin() : Plugin("SampleHiroController")
  {
    require("Body");
    require("Teaching");
  }

  virtual bool initialize()
  {
    ControllerManager::instance()->registController("I611Controller", I611Controller::instance());
    // ControllerManager::instance()->registController("UR5Controller", UR5Controller::instance());
    ControllerManager::instance()->registController("UR3dualController", UR3dualController::instance());
    ControllerManager::instance()->registController("HiroNXController", HiroNXController::instance());

    ToolBar* bar = new ToolBar("SampleHiroController");
    bar->addButton("Sycn with Real")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onSyncButtonClicked, this));
    bar->addButton("Initial Pose")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onInitialPoseButtonClicked, this));
    bar->addButton("Pose1")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onPose1ButtonClicked, this));
    bar->addButton("Pose2")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onPose2ButtonClicked, this));
    bar->addButton("Pose3")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onPose3ButtonClicked, this));
    addToolBar(bar);
    bar->setVisibleByDefault(true);

    return true;
  }

  void onSyncButtonClicked()
  {
#if 0
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");
    handler->syncWithReal();
#endif
  }

  void onPose1ButtonClicked()
  {
#if 0
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");

    std::vector<CompositeParamType> params;
    std::string commandName = "moveArm";
    VectorXd xyz(3);
    xyz(0) = 0.3057;
    xyz(1) = 0.2363;
    xyz(2) = 0.8578;
    VectorXd rpy(3);
    rpy(0) = -1.5;
    rpy(1) = 1.3;
    rpy(2) = -146.2;
    double duration = 10.0;
    int armID = 0;
    params.push_back(xyz);
    params.push_back(rpy);
    params.push_back(duration);
    params.push_back(armID);
    handler->executeCommand(commandName, params);
#endif
  }

  void onPose2ButtonClicked()
  {
#if 0
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");

    std::vector<CompositeParamType> params;
    std::string commandName = "moveArm";
    VectorXd xyz(3);
    xyz(0) = 0.3239;
    xyz(1) = 0.1863;
    xyz(2) = 0.9657;
    VectorXd rpy(3);
    rpy(0) = 2.3;
    rpy(1) = 29.2;
    rpy(2) = -132.6;
    double duration = 10.0;
    int armID = 0;
    params.push_back(xyz);
    params.push_back(rpy);
    params.push_back(duration);
    params.push_back(armID);
    handler->executeCommand(commandName, params);
#endif
  }

  void onPose3ButtonClicked()
  {
#if 0
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");

    std::vector<CompositeParamType> params;
    std::string commandName = "moveArm";
    VectorXd xyz(3);
    xyz(0) = 0.3483;
    xyz(1) = -0.1176;
    xyz(2) = 1.2746;
    VectorXd rpy(3);
    rpy(0) = -130.5;
    rpy(1) = -4.7;
    rpy(2) = 126.7;
    double duration = 10.0;
    int armID = 1;
    params.push_back(xyz);
    params.push_back(rpy);
    params.push_back(duration);
    params.push_back(armID);
    handler->executeCommand(commandName, params);
#endif
  }

  void onInitialPoseButtonClicked()
  {
#if 0
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");

    std::vector<CompositeParamType> params;
    double duration = 10.0;
    params.push_back(duration);
    handler->executeCommand("goInitial", params);
#endif
  }

  void onPubTrajButtonClicked()
  {
    ControllerBase* handler = ControllerManager::instance()->getController("FollowTrajectoryController");
    handler->setRootName("main_withHands");

#if 0
    FollowTrajectoryController::instance()->sendTrajectory();
#endif
  }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SampleHiroControllerPlugin);
