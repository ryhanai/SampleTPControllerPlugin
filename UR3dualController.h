/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class UR3dualController : public Controller
  {
  public:
    static UR3dualController* instance();

  private:
    class MoveArmCommand : public Command
    {
    public:
      MoveArmCommand(UR3dualController* c) { c_ = c; }
      UR3dualController* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

    class MoveGripperCommand : public Command
    {
    public:
      MoveGripperCommand(UR3dualController* c) { c_ = c; }
      UR3dualController* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

    class GoInitialCommand : public Command
    {
    public:
      GoInitialCommand(UR3dualController* c) { c_ = c; }
      UR3dualController* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

    class MoveCommand : public Command
    {
    public:
      MoveCommand(UR3dualController* c) { c_ = c; }
      UR3dualController* c_;
      virtual bool operator()(std::vector<CompositeParamType>& params);
    };

  private:
    UR3dualController();
    void registerCommands ();
    bool executeDualArmMotion();
    bool executeGripperMotion (const std::vector<std::string>& gripperLinks, double width);
  };

}
