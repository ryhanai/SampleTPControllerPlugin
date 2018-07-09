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

    return true;
  }


};

CNOID_IMPLEMENT_PLUGIN_ENTRY(FollowTrajectoryControllerPlugin);
