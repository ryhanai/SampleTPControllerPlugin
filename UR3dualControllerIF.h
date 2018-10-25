/**
   @author Ryo Hanai
*/

#pragma once

namespace teaching
{

  class UR3dualControllerIF
  {
  public:
    virtual bool moveArm (std::vector<CompositeParamType>& params) = 0;
    virtual bool moveGripper (std::vector<CompositeParamType>& params) = 0;
    virtual bool goInitial (std::vector<CompositeParamType>& params) = 0;
  };

}
