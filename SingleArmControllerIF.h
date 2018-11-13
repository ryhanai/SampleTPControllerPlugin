/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerIF.h"

namespace teaching
{

  class SingleArmControllerIF : public ControllerIF
  {
  public:
    SingleArmControllerIF ();

    virtual bool moveArm (std::vector<CompositeParamType>& params) = 0;
    virtual bool goInitial (std::vector<CompositeParamType>& params) = 0;
  };

}
