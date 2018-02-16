#pragma once

#include <string>
#include <list>
#include <vector>
#include <map>
#include <cnoid/BodyItem>
#include <cnoid/JointPath>
#include "Interpolator.h"
#include "ControllerBase.h"

namespace teaching
{

  class CartesianInterpolator // Nextage: straight-line, spherical linear interpolation
  {
  public:
    CartesianInterpolator() { }
    void clear();
    void appendSample(double t, const cnoid::Vector3& xyz, const cnoid::Matrix3d& rotation);
    void update();
    cnoid::SE3 interpolate(double t);

  private:
    cnoid::Interpolator<cnoid::VectorXd> vInterpolator_;
    std::vector<double> ts_;
    std::vector<cnoid::Matrix3d> qsamples_;
  };

  class AttachedModel 
  {
  public:
    AttachedModel() {};
    AttachedModel(AttachedModel& source) 
      : handLink(source.handLink), objectLink(source.objectLink),
        posVal(source.posVal),
        object(source.object) {
    };

    cnoid::Link* handLink;
    cnoid::Link* objectLink;
    cnoid::BodyItemPtr object;
    std::vector<double> posVal;
  };

  class A
  {
  public:
    A (std::string name, std::string type, int numElms)
    {
      _name = name;
      _type = type;
      _n = numElms;
    }

    std::string name() { return _name; }
    std::string type() { return _type; }
    int numElms() { return _n; }

  private:
    std::string _name;
    std::string _type;
    int _n;
  };

  class Controller : public ControllerBase
  {
  public:
    Controller ();

    class Command
    {
    public:
      virtual bool operator() (const std::vector<CompositeParamType>& params) = 0;
    };

    // Methods called by teachingPlugin
    std::vector<CommandDefParam*> getCommandDefList();
    virtual bool executeCommand(const std::string& commandName, const std::vector<CompositeParamType>& params, bool isReal=true);
    bool attachModelItem(cnoid::BodyItemPtr object, int target);
    bool detachModelItem(cnoid::BodyItemPtr object, int target);
    
    // Methods used to implement controllers
    void registerCommand(std::string internalName, std::string displayName, std::string returnType,
                         std::list<A> arguments, Command* commandFunc);
    void setToolLink(int toolNumber, std::string linkName) { toolLinks_[toolNumber] = linkName; }
    std::string getToolLink(int toolNumber) { return toolLinks_[toolNumber]; }
    cnoid::BodyItem* getRobotItem();
    cnoid::BodyItem* findItemByName(const std::string& name);
    bool executeJointMotion(BodyItem* robotItem, double duration);
    bool executeCartesianMotion(BodyItem* robotItem, Link* wrist, JointPathPtr jointPath, double duration);
    
    // Model action
    bool updateAttachedModels ();
    
    // Simulator configuration
    void setTimeStep (double seconds) { dt_ = seconds; }
    double getTimeStep () { return dt_; }

    cnoid::Interpolator<cnoid::VectorXd> jointInterpolator;
    CartesianInterpolator ci, ci2;
    
  private:
    Command* getCommand(const std::string& internalName) { return commands_[internalName]; }
    std::map<std::string, Command*> commands_;
    std::vector<CommandDefParam*> commandDefs_;
    int registeredCommands_ = 1;

    std::map<int, std::string> toolLinks_;
    double dt_ = 0.02;
    std::vector<AttachedModel*> attachedObjs_;
  };

}
