/**
   @author Ryo Hanai
*/

#include <cnoid/Plugin>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include "SampleHiroController.h"
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
    ControllerManager::instance()->registController("SampleHiroController", SampleHiroController::instance());

    return true;
  }

  void onTestButtonClicked()
  {
    // SampleHiroController::instance()->test();
  }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SampleHiroControllerPlugin);
