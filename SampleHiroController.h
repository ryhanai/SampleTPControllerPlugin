#pragma once

#include "ControllerFramework.h"

namespace teaching
{

  class SampleHiroController : public Controller
  {
  public:
    static SampleHiroController* instance();

  private:
    class MoveTorsoCommand : public Command
    {
    public:
      MoveTorsoCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class MoveHeadCommand : public Command
    {
    public:
      MoveHeadCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class MoveArmCommand : public Command
    {
    public:
      MoveArmCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class MoveGripperCommand : public Command
    {
    public:
      MoveGripperCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class ScrewCommand : public Command
    {
    public:
      ScrewCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class GoInitialCommand : public Command
    {
    public:
      GoInitialCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

    class MoveCommand : public Command
    {
    public:
      MoveCommand(SampleHiroController* c) { c_ = c; }
      SampleHiroController* c_;
      virtual bool operator()(const std::vector<CompositeParamType>& params);
    };

  private:
    SampleHiroController();
    bool executeDualArmMotion(double duration);
    bool executeGripperMotion (const std::vector<std::string>& gripperLinks, double width, double duration);
    void registerCommands ();
  };

}
