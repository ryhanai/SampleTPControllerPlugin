/**
   @author Ryo Hanai
*/

#include "CommandSet.h"

namespace teaching
{
  void CommandSet::defineCommand (std::string internalName, std::string displayName,
                                  std::string returnType,
                                  std::list<A> arguments)
  {
    CommandDefParam* cmd = new CommandDefParam(QString::fromStdString(internalName),
                                               QString::fromStdString(displayName), QString::fromStdString(returnType));
    for (auto arg: arguments) {
      ArgumentDefParam* a = new ArgumentDefParam(arg.name(), arg.type(), arg.numElms(), static_cast<int>(arg.direction()));
      cmd->addArgument(a);
    }

    commandDefs_.push_back(cmd);
  }

  std::vector<CommandDefParam*> CommandSet::getCommandDefinitions () { return commandDefs_; }

}
