/**
   @author Ryo Hanai
*/

#pragma once

#include <string>
#include <list>
#include <vector>
#include <tuple>
#include <map>
#include <sstream>
#include <cnoid/BodyItem>
#include <cnoid/JointPath>
#include <cnoid/MessageView>
#include "Interpolator.h"
#include "ControllerBase.h"

namespace teaching
{
  double toRad (double deg);
  Vector3 toRad (Vector3 degs);
  VectorX toRad (VectorX degs);

  template<typename T>
    void printLogAux (std::stringstream& ss, const T& x)
  {
    ss << x;
    // std::cout << ss.str() << std::endl;
    MessageView::instance()->putln(ss.str());
  }

  template<typename T, typename ... Rest>
    void printLogAux (std::stringstream& ss, const T& x, const Rest& ... rest)
  {
    ss << x;
    printLogAux (ss, rest...);
  }

  template<typename T, typename ... Rest>
    void printLog (const T& x, const Rest& ... rest)
  {
    std::stringstream ss;
    printLogAux (ss, x, rest...);
  }


  typedef std::vector<std::tuple<double, cnoid::VectorXd> > Trajectory;

  class CartesianInterpolator // Nextage: straight-line, spherical linear interpolation
  {
  public:
    CartesianInterpolator() { }
    void clear();

    int numSamples() const { return ts_.size(); }
    double domainLower () const { return ts_.empty() ? 0.0 : ts_.front(); }
    double domainUpper () const { return ts_.empty() ? 0.0 : ts_.back(); }
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


  class ControllerException
  {
  public:
    ControllerException () { }
    ControllerException (const std::string& message) { setMessage (message); }
    virtual ~ControllerException () { }
    const std::string& message () const { return message_; }
    void setMessage (const std::string& message) { message_ = message; }
  private:
    std::string message_;
  };

  class RobotNotFoundException : public ControllerException
  {
  public:
    RobotNotFoundException (const std::string& name) {
      setMessage("Robot [" + name + "] not found");
    }
  };

  class ItemNotFoundException : public ControllerException
  {
  public:
    ItemNotFoundException (const std::string& name) {
      setMessage("Item [" + name + "] not found");
    }
  };

  class CommandNotFoundException : public ControllerException
  {
  public:
    CommandNotFoundException (const std::string& name) {
      setMessage("Command [" + name + "] not found");
    };
  };

  class UnexpectedArgumentException : public ControllerException
  {
  public:
    UnexpectedArgumentException (boost::bad_get& e) {
      setMessage(e.what());
    }
  };

  class UndefinedToolNumberException : public ControllerException
  {
  public:
    UndefinedToolNumberException (const int n) {
      setMessage("tool number [" + std::to_string(n) + "] is not defined in the controller");
    }
  };

  class UndefinedToolLinkException : public ControllerException
  {
  public:
    UndefinedToolLinkException (const std::string& s){
      setMessage("tool link [" + s + "] does not exist in the robot model.");
    }
  };

  class IKFailureException : public ControllerException
  {
  public:
    IKFailureException (const std::string& message) {
      setMessage("IK failure [" + message);
    }
  };
  
  class TPInterface
  {
  public:
    TPInterface () { clearAttachedModels(); }

    void setToolLink (int toolNumber, std::string linkName) { toolLinks_[toolNumber] = linkName; }
    std::string getToolLinkName (int toolNumber) { return toolLinks_[toolNumber]; }
    cnoid::Link* getToolLink (int toolNumber);
    cnoid::BodyItem* getRobotItem ();
    cnoid::BodyItem* findItemByName (const std::string& name);
    cnoid::BodyPtr getRobotBody ();
    void setRobotName (std::string robotName) { robotName_ = robotName; }

    // Model action
    bool updateAttachedModels ();
    void clearAttachedModels () { attachedModels_.clear(); }
    bool attachModelItem(cnoid::BodyItemPtr object, int target);
    bool detachModelItem(cnoid::BodyItemPtr object, int target);

    // Simulator configuration
    void setTimeStep (double seconds) { dt_ = seconds; }
    double getTimeStep () { return dt_; }


    //cnoid::Interpolator<cnoid::VectorXd> jointInterpolator;
    CartesianInterpolator ci_;
    bool interpolate(int toolNumber,
                     const Vector3& xyz, const Vector3& rpy, double duration,
                     Trajectory& traj);
    bool followTrajectory(int toolNumber, const Trajectory& traj);
    cnoid::VectorXd getCurrentJointAngles(cnoid::BodyPtr body);
    
  private:
    std::string robotName_;
    std::map<int, std::string> toolLinks_;
    double dt_ = 0.05;
    std::vector<AttachedModel*> attachedModels_;
  };

  typedef std::shared_ptr<TPInterface> TPInterfacePtr;

  class Controller : public ControllerBase
  {
  public:
    void setTPInterface (TPInterfacePtr tpif) { tpif_ = tpif; }

    typedef std::function<bool(std::vector<CompositeParamType>&)> Command;

    // Methods called by teachingPlugin
    // std::vector<CommandDefParam*> getCommandDefList();
    virtual bool executeCommand(const std::string& commandName, std::vector<CompositeParamType>& params);

    // delegate to ControllerContext
    bool attachModelItem(cnoid::BodyItemPtr object, int target);
    bool detachModelItem(cnoid::BodyItemPtr object, int target);

    // bool executeJointMotion();
    // bool executeCartesianMotion(Link* wrist, JointPathPtr jointPath);

    void registerCommandFunction (std::string internalName, Command command);

  protected:
    TPInterfacePtr tpif_;

  private:
    std::map<std::string, Command> commands_;
    //std::vector<CommandDefParam*> commandDefs_;
  };

}
