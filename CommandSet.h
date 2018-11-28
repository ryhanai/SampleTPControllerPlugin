/**
   @author Ryo Hanai
*/

#pragma once

#include <string>
#include <vector>
#include <list>
#include "CommandDefTypes.h"

namespace teaching
{
  class A
  {
  public:
    enum class var_prop: int
    {
      in, out, inout
    };

    A (std::string name, std::string type, int numElms, var_prop direction = var_prop::in)
    {
      _name = name;
      _type = type;
      _n = numElms;
      _d = direction;
    }

    std::string name() { return _name; }
    std::string type() { return _type; }
    int numElms() { return _n; }
    var_prop direction() { return _d; }

  private:
    std::string _name;
    std::string _type;
    int _n;
    var_prop _d;
  };

  class CommandSet
  {
  public:
    void defineCommand (std::string internalName, std::string displayName,
                        std::string returnType,
                        std::list<A> arguments);
    std::vector<CommandDefParam*> getCommandDefinitions ();

  private:
    std::vector<CommandDefParam*> commandDefs_;
    int registeredCommands_ = 1;
  };

}
