/**
   @author Ryo Hanai
*/

#pragma once

#include "CommandSet.h"

namespace teaching
{

  class SingleArmCommandSet : public CommandSet
  {
  public:
    SingleArmCommandSet () {
      defineCommand ("moveArm", "Arm", "boolean",
                     {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)});
      defineCommand ("goInitial", "Initial Pose", "boolean",
                     {A("tm", "double", 1)});
    }
  };

}
