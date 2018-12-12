/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class ObjectPoseSensor
  {
  public:
    bool recognize (std::vector<CompositeParamType>& params, bool isReal);
    bool recognize_double (std::vector<CompositeParamType>& params, bool isReal);
    bool recognize_int (std::vector<CompositeParamType>& params, bool isReal);

  private:
  };

}
