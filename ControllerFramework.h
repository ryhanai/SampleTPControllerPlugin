/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerBase.h"
#include "CommandSet.h"
#include "TPUtil.h"

namespace teaching
{
  typedef std::function<bool(std::vector<CompositeParamType>&)> CommandFunction;

  class Controller : public ControllerBase
  {
  public:
    virtual bool executeCommand (const std::string& commandName, std::vector<CompositeParamType>& params, bool isReal = false);
    std::vector<CommandDefParam*> getCommandDefList() { return cmdset_->getCommandDefinitions(); }
    bool attachModelItem (cnoid::BodyItemPtr object, int target);
    bool detachModelItem (cnoid::BodyItemPtr object, int target);
    void bindCommandFunction (std::string internalName, bool isReal, CommandFunction commandFunction);
    void setCommandSet (CommandSet* commandSet) { cmdset_ = commandSet; }

  private:
    CommandSet* cmdset_;
    std::map<std::tuple<std::string, bool>, CommandFunction> commandTable_;
  };

}
