/**
   @author Ryo Hanai
*/

#include <cnoid/Plugin>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include "SampleHiroController.h"
#include "UR3dualController.h"
#ifdef ROS_ON
#include "FollowTrajectoryControllerROS.h"
#include "FollowTrajectoryControllerUR3Dual.h"
#endif

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
    ToolBar* bar = new ToolBar("SampleHiroController");
    bar->addButton("Test")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onTestButtonClicked, this));
    addToolBar(bar);
    // ControllerManager::instance()->registController("SampleHiroController", SampleHiroController::instance());
    ControllerManager::instance()->registController("UR3dualController", UR3dualController::instance());

    ToolBar* barft = new ToolBar("FollowTrajectoryController");
    barft->addButton("Sycn with Real")->sigClicked().connect(bind(&SampleHiroControllerPlugin::onTestButtonClicked, this));
    addToolBar(barft);

    // ControllerManager::instance()->registController("FollowTrajectoryController", FollowTrajectoryController::instance());
    ControllerManager::instance()->registController("FollowTrajectoryControllerUR3Dual", FollowTrajectoryControllerUR3Dual::instance());

    return true;
  }

  void onTestButtonClicked()
  {
    FollowTrajectoryControllerUR3Dual* handler =
      (FollowTrajectoryControllerUR3Dual*)ControllerManager::instance()->getController("FollowTrajectoryControllerUR3Dual");
    handler->setRootName("main_withHands");
    handler->syncWithReal();
    // std::vector<CompositeParamType> params;
    // double duration = 1.0;
    // params.push_back(duration);
    // handler->executeCommand("goInitial", params, true);

    // params.clear();
    // double width = 0.3;
    // int gripperID = 0;
    // params.push_back(width);
    // params.push_back(duration);
    // params.push_back(gripperID);
    // handler->executeCommand("moveGripper", params, true);

    // std::string commandName = "moveArm";
    // std::vector<CompositeParamType>& params;
    // VectorXd xyz(3);
    // xyz(0) = 0.0;
    // xyz(1) = 0.0;
    // xyz(2) = 0.0;
    // VectorXd rpy(3);
    // rpy(0) = 0.0;
    // rpy(1) = 0.0;
    // rpy(2) = 0.0;
    // double duration = 1.0;
    // int armID = 0;
    // params.push_back(xyz);
    // params.push_back(rpy);
    // params.push_back(double);
    // params.push_back(0);
    // handler->executeCommand(commandName, params, true);
  }

  void onPubTrajButtonClicked()
  {
    ControllerBase* handler = ControllerManager::instance()->getController("FollowTrajectoryController");
    handler->setRootName("main_withHands");

#ifdef ROS_ON
    FollowTrajectoryController::instance()->sendTrajectory();
#endif
  }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SampleHiroControllerPlugin);
