/**
   @author Ryo Hanai
*/

#include <QCoreApplication>
#include <cnoid/ItemList>
#include <cnoid/RootItem>

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include "ControllerFramework.h"

namespace teaching
{
  double toRad (double deg) {
    return deg * M_PI/180.0;
  }
  // template<typename T>
  // T toRad (T degs)
  // {
  //   T rads(degs.array() * M_PI/180.0);
  //   return rads;
  // }
  Vector3 toRad (Vector3 degs)
  {
    Vector3 rads(degs.array() * M_PI/180.0);
    return rads;
  }
  
  VectorX toRad (VectorX degs)
  {
    VectorX rads(degs.array() * M_PI/180.0);
    return rads;
  }
  
  void CartesianInterpolator::clear()
  {
    vInterpolator_.clear();
    ts_.clear();
    qsamples_.clear();
  }

  void CartesianInterpolator::appendSample(double t, const Vector3& xyz, const Matrix3d& rotation)
  {
    VectorXd p(3);
    p.head<3>() = xyz;
    vInterpolator_.appendSample(t, p);
    ts_.push_back(t);
    qsamples_.push_back(rotation);
  }

  void CartesianInterpolator::update()
  {
    vInterpolator_.update();
  }

  SE3 CartesianInterpolator::interpolate(double t)
  {
    VectorXd pd = vInterpolator_.interpolate(t);
    Vector3 p = pd.head<3>();
    double tdiff = ts_[1] - ts_[0];
    cnoid::Matrix3d mat0 = qsamples_[0];
    cnoid::Matrix3d mat1 = qsamples_[1];
    Quat q1(mat0);
    Quat q2(mat1);

    Quat q_slerp = q1.slerp(t / tdiff, q2);
    SE3 tf(p, q_slerp);
    return tf;
  }
  
  Controller::Controller()
  {
    attachedObjs_.clear();
  }

  std::vector<CommandDefParam*> Controller::getCommandDefList() { return commandDefs_; }

  bool Controller::executeCommand(const std::string& commandName, std::vector<CompositeParamType>& params)
  {
    Command cmd;

    try {
      cmd = commands_[commandName];
    } catch (...) {
      printLog ("Command ", commandName, " not found");
      return false;
    }

    try {
      if (cmd(params)) { return true; }
    } catch (RobotNotFoundException& e) {
      printLog ("Robot ", e.message(), " not found");
    } catch (ItemNotFoundException& e) {
      printLog ("Item ", e.message(), " not found");
    } catch (UndefinedToolException& e) {
      printLog ("Undefined tool ", e.message());
    } catch (IKFailureException& e) {
      printLog ("IK failure ", e.message());
    } catch (boost::bad_get& e) {
      printLog ("Unexpected Argument ", e.what());
    }

    attachedObjs_.clear();
    return false;
  }

  bool Controller::attachModelItem (BodyItemPtr object, int target) // attach "object" to "target"
  {
    BodyPtr robotBody = getRobotItem()->body();

    try {
      Link* handLink = getToolLink(target);
      Link* objectLink = object->body()->link(0);
      Position relTrans = handLink->position().inverse()*objectLink->position();
      AttachedModel* model = new AttachedModel();
      model->handLink = handLink;
      model->objectLink = objectLink;
      for (int index = 0; index < 12; index++) {
        model->posVal.push_back(relTrans.data()[index]);
      }
      model->object = object;
      attachedObjs_.push_back(model);

    } catch(...) {
      printLog("[attachModeItem] unknown link ID: ", target);
      return false;
    }

    return true;
  }

  bool Controller::detachModelItem (BodyItemPtr object, int target)
  {
    BodyPtr robotBody = getRobotItem()->body();

    try {
      Link* handLink = getToolLink(target);
      Link* objectLink = object->body()->link(0);

      for (std::vector<AttachedModel*>::iterator it = attachedObjs_.begin(); it != attachedObjs_.end();) {
        if ((*it)->handLink == handLink && (*it)->objectLink == objectLink) {
          it = attachedObjs_.erase(it);
          printLog("detachModelItem");
        }
        else {
          ++it;
        }
      }
    }
    catch (...) {
      printLog("[detachModeItem] unknown link ID: ", target);
      return false;
    }

    return true;
  }

  cnoid::Link* Controller::getToolLink(int toolNumber)
  {
    BodyPtr robotBody = getRobotBody();
    Link* link = robotBody->link(getToolLinkName(toolNumber));
    if (link) {
      return link;
    } else {
      throw UndefinedToolException(toolNumber);
    }
  }

  BodyItem* Controller::getRobotItem ()
  {
    BodyItem* robotItem = findItemByName(rootName);
    if (robotItem) {
      return robotItem;
    } else {
      throw RobotNotFoundException(rootName);
    }
  }

  BodyItem* Controller::findItemByName (const std::string& name)
  {
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(RootItem::instance());
    for (size_t i = 0; i < bodyItems.size(); i++) {
      BodyItem* item = bodyItems.get(i);
      if (item->name() == name) { return item; }
    }

    return NULL;
  }

  BodyPtr Controller::getRobotBody ()
  {
    BodyItem* robotItem = getRobotItem();
    BodyPtr robotBody = robotItem->body();
    return robotBody;
  }

  VectorXd Controller::getCurrentJointAngles (BodyPtr body)
  {
    VectorXd q;
    int n = body->numJoints();
    q.resize(n);
    for (int i = 0; i < n; i++) { q[i] = body->joint(i)->q(); }
    return q;
  }

  bool Controller::updateAttachedModels ()
  {
    for (unsigned int index = 0; index<attachedObjs_.size(); index++) {
      AttachedModel* model = attachedObjs_[index];
      Link* hand = model->handLink;
      Link* object = model->objectLink;

      std::vector<double> vecPos = model->posVal;
      Position objHandTrans;
      for (int index = 0; index < 12; index++) {
        objHandTrans.data()[index] = vecPos[index];
      }

      BodyItemPtr objItem = model->object;

      object->R() = hand->R() * objHandTrans.linear();
      object->p() = hand->p() + hand->R() * objHandTrans.translation();
      objItem->notifyKinematicStateChange(true);
    }

    return true;
  }

  void Controller::registerCommand (std::string internalName, std::string displayName, std::string returnType,
                                    std::list<A> arguments,
                                    Command commandFunc)
  {
    CommandDefParam* cmd = new CommandDefParam(registeredCommands_++, QString::fromStdString(internalName),
                                               QString::fromStdString(displayName), QString::fromStdString(returnType));
    for (auto arg: arguments) {
      ArgumentDefParam* a = new ArgumentDefParam(arg.name(), arg.type(), arg.numElms(), static_cast<int>(arg.direction()));
      cmd->addArgument(a);
    }

    commands_[internalName] = commandFunc;
    commandDefs_.push_back(cmd);
  }

  bool Controller::interpolate(int toolNumber, const Vector3& xyz, const Vector3& rpy, double duration, Trajectory& traj)
  {
    BodyItem* robotItem = getRobotItem();
    BodyPtr body = getRobotBody();
    Link* base = body->rootLink();
    Link* tool = getToolLink(toolNumber);
    JointPathPtr jointPath = getCustomJointPath(body, base, tool);
    jointPath->calcForwardKinematics();

    traj.clear();

    ci_.clear();
    ci_.appendSample(0, tool->p(), tool->attitude());
    ci_.appendSample(duration, xyz, rotFromRpy(rpy));
    ci_.update();

    // double duration = ci.domainUpper();

    for (double time = 0.0; time < duration+dt_; time += dt_) {
      if (time > duration) { time = duration; }

      SE3 tf = ci_.interpolate(time);
      if (jointPath->calcInverseKinematics(tf.translation(),
                                           tool->calcRfromAttitude(tf.rotation().toRotationMatrix()))) {
        updateAttachedModels();
        robotItem->notifyKinematicStateChange(true);
        VectorXd q;
        q.resize(jointPath->numJoints());
        for (int i = 0; i < jointPath->numJoints(); i++) {
          q[i] = jointPath->joint(i)->q();
          std::cout << q[i] << ",";
        }
        std::cout << std::endl;

        auto wp = std::make_tuple(time, q);
        traj.push_back(wp);
      } else {
        return false; // throwing IK failure exception might be better
      }
    }

    return true;
  }
  
  bool Controller::followTrajectory(int toolNumber, const Trajectory& traj)
  {
    printLog("followTrajectory");

    BodyItem* robotItem = getRobotItem();
    BodyPtr body = getRobotBody();
    Link* base = body->rootLink();
    Link* tool = getToolLink(toolNumber);
    JointPathPtr jointPath = getCustomJointPath(body, base, tool);

    double last_tm = 0.0;

    for (auto wp : traj) {
      double tm = std::get<0>(wp);
      VectorXd q = std::get<1>(wp);

      double dt = tm - last_tm;
      last_tm = tm;

#ifndef _WIN32
      auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(dt*1000));
#endif

      for (int i = 0; i < jointPath->numJoints(); i++) {
        jointPath->joint(i)->q() = q[i];
      }

      updateAttachedModels();
      robotItem->notifyKinematicStateChange(true);
      //QCoreApplication::sendPostedEvents();
      QCoreApplication::processEvents();

#ifdef _WIN32
      Sleep((int)(dt*1000));
#else
      std::this_thread::sleep_until(abs_time);
#endif
    }

    printLog("followTrajectory finished");

    return true;

  }
  
  bool Controller::executeJointMotion()
  {
    printLog("executeJointMotion");

    double duration = jointInterpolator.domainUpper();
    BodyItem* robotItem = getRobotItem();
    BodyPtr body = robotItem->body();

    for (double time = 0.0; time < duration+dt_; time += dt_) {
      if (time > duration) { time = duration; }

#ifndef _WIN32
      auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(dt_*1000));
#endif
      VectorXd qRef;
      qRef = jointInterpolator.interpolate(time);
      for (int i = 0; i < body->numJoints(); i++) {
        body->joint(i)->q() = qRef[i];
      }

      updateAttachedModels();
      robotItem->notifyKinematicStateChange(true);
      //QCoreApplication::sendPostedEvents();
      QCoreApplication::processEvents();

#ifdef _WIN32
      Sleep((int)(dt_*1000));
#else
      std::this_thread::sleep_until(abs_time);
#endif
    }

    printLog("executeJointMotion Finished");

    return true;
  }

}
