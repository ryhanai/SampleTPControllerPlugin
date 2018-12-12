/**
   @author Ryo Hanai
*/

#pragma once

#include "CommandSet.h"

namespace teaching
{

  class ObjectPoseSensorCommandSet : public CommandSet
  {
  public:
    ObjectPoseSensorCommandSet () {
      defineCommand ("recognize", "Recognize", "boolean",
                     {A("object_id", "int", 1),
                         A("result", "double", 6, A::var_prop::out)});
      defineCommand ("recognize_double", "Recognize(double)", "boolean",
                     {A("object_id", "int", 1),
                         A("result", "double", 1, A::var_prop::out)});
      defineCommand ("recognize_int", "Recognize(int)", "boolean",
                     {A("object_id", "int", 1),
                         A("result", "int", 1, A::var_prop::out)});
    }
  };

}
