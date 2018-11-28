/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class EZGripperFakeController
  {
  public:
    bool moveGripper (std::vector<CompositeParamType>& params);
    bool goInitial (std::vector<CompositeParamType>& params);

  private:
    //std::vector<std::tuple<std::string, double (*)(double)> > gjoints_;
    //std::vector<std::tuple<std::string, std::function<double (double)> > > gjoints_;
  };

}
