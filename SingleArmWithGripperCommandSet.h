/**
   @author Ryo Hanai
*/

#pragma once

#include "CommandSet.h"

namespace teaching
{

  class SingleArmWithGripperCommandSet : public CommandSet
  {
  public:
    SingleArmWithGripperCommandSet () {
      defineCommand ("moveArm", "Arm", "boolean",
                     {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)});
      defineCommand("moveGripper", "Gripper", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)});
      defineCommand ("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)});
    }
  };

}
