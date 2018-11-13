/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"
#include "SingleArmControllerIF.h"

namespace teaching
{

  class SingleArmFakeController : public Controller, public SingleArmControllerIF
  {
  public:
    static SingleArmFakeController* instance();

    bool moveArm (std::vector<CompositeParamType>& params) override;
    bool goInitial (std::vector<CompositeParamType>& params) override;

    std::vector<CommandDefParam*> getCommandDefList() { return getCommandDefinitions(); }

  private:
    SingleArmFakeController();
  };

}
