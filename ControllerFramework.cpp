/**
   @author Ryo Hanai
*/

#include "ControllerFramework.h"

namespace teaching
{

  bool Controller::executeCommand(const std::string& commandName, std::vector<CompositeParamType>& params, bool isReal)
  {
    CommandFunction cmd;

    try {
      // cmd = commandTable_[std::make_tuple(commandName, isReal)];
      cmd = commandTable_[std::make_tuple(commandName, true)];
    } catch (...) {
      printLog ("Command ", commandName, " not found");
      return false;
    }

    try {
      if (cmd(params)) { return true; }
    } catch (ControllerException& e) {
      printLog (e.message());
    } catch (boost::bad_get& e) {
      printLog ("Unexpected Argument ", e.what());
    }

    TPInterface::instance().clearAttachedModels();
    return false;
  }

  bool Controller::attachModelItem (BodyItemPtr object, int target) // attach "object" to "target"
  {
    return TPInterface::instance().attachModelItem (object, target);
  }

  bool Controller::detachModelItem (BodyItemPtr object, int target)
  {
    return TPInterface::instance().detachModelItem (object, target);
  }

  void Controller::bindCommandFunction (std::string internalName, bool isReal, CommandFunction commandFunction)
  {
    commandTable_[std::make_tuple(internalName, isReal)] = commandFunction;
  }

}
