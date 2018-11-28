/**
   @author Ryo Hanai
*/

#pragma once

#include "CommandSet.h"

namespace teaching
{

  class HiroNXCommandSet : public CommandSet
  {
  public:
    HiroNXCommandSet () {
      defineCommand ("moveTorso", "Torso", "boolean", {A("angle", "double", 1), A("tm", "double", 1)});
      defineCommand ("moveArm", "Arm", "boolean",
                     {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)});
      defineCommand ("moveHead", "Head", "boolean", {A("angles", "double", 2), A("tm", "double", 1)});
      defineCommand ("moveBothArms", "Both arms", "boolean",
                     {A("leftHandXyz", "double", 3), A("leftHandRpy", "double", 3),
                         A("rightHandXyz", "double", 3), A("rightHandRpy", "double", 3),
                         A("torsoAngle", "double", 1), A("tm", "double", 1)});
      defineCommand ("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)});
      defineCommand ("moveGripper", "Gripper", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)});
      // defineCommand ("screw", "Screw", "boolean", {A("torque", "double", 1), A("max_depth", "double", 1)});
      /* defineCommand("moveGripperTest", "GripperTest", "boolean", */
      /*               {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1), */
      /*                   A("result", "double", 1, A::var_prop::out)}); */
    }
  };

}
