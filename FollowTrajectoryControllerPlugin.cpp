/**
   @author Ryo Hanai
*/

#include <cnoid/Plugin>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include "FollowTrajectoryControllerROS.h"

using namespace cnoid;
using namespace boost;
using namespace teaching;

class FollowTrajectoryControllerPlugin : public Plugin
{
public:

  FollowTrajectoryControllerPlugin() : Plugin("FollowTrajectoryController")
  {
    require("Body");
    require("Teaching");
  }

  virtual bool initialize()
  {
    ToolBar* bar = new ToolBar("FollowTrajectoryController");
    bar->addButton("publish sample trajectory")->sigClicked().connect(bind(&FollowTrajectoryControllerPlugin::onTestButtonClicked, this));
    addToolBar(bar);

    ControllerManager::instance()->registController("FollowTrajectoryController", FollowTrajectoryController::instance());

    return true;
  }

  void onTestButtonClicked()
  {
    ControllerBase* handler = ControllerManager::instance()->getController("FollowTrajectoryController");
    handler->setRootName("main_withHands");

    FollowTrajectoryController::instance()->sendTrajectory();
  }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(FollowTrajectoryControllerPlugin);
