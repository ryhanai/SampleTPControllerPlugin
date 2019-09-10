/**
   @author Ryo Hanai
*/

#pragma once

#include "ControllerBase.h"
#include "CommandSet.h"
#include "TPUtil.h"

namespace teaching
{
  typedef std::function<bool(std::vector<CompositeParamType>&, bool)> CommandFunction;

  class Controller : public ControllerBase
  {
  public:
    virtual bool executeCommand (const std::string& commandName, std::vector<CompositeParamType>& params, bool isReal = false);
    std::vector<CommandDefParam*> getCommandDefList() { return cmdset_->getCommandDefinitions(); }
    bool attachModelItem (cnoid::BodyItemPtr object, int target);
    bool detachModelItem (cnoid::BodyItemPtr object, int target);
    void bindCommandFunction (std::string internalName, CommandFunction commandFunction);
    void setCommandSet (CommandSet* commandSet) { cmdset_ = commandSet; }
    cnoid::Link* getToolLink(int toolNumber);

  private:
    CommandSet* cmdset_;
    std::map<std::string, CommandFunction> commandTable_;
  };

}
