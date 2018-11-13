/**
   @author Ryo Hanai
*/

#include "SingleArmControllerIF.h"

namespace teaching
{

  SingleArmControllerIF::SingleArmControllerIF ()
  {
    defineCommand ("moveArm", "Arm", "boolean",
                   {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)});
    defineCommand ("goInitial", "Initial Pose", "boolean",
                   {A("tm", "double", 1)});
  }

}
