/**
   @author Ryo Hanai
*/

#include <QCoreApplication>
#include <cnoid/MessageView>
#include <cnoid/ItemList>
#include <cnoid/RootItem>

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif

#include "ControllerFramework.h"

namespace teaching
{
  void CartesianInterpolator::clear()
  {
    vInterpolator_.clear();
    ts_.clear();
    qsamples_.clear();
  }

  void CartesianInterpolator::appendSample(double t, const::Vector3& xyz, const Matrix3d& rotation)
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

  bool Controller::executeCommand(const std::string& commandName, const std::vector<CompositeParamType>& params, bool isReal)
  {
    try {
      if ((*getCommand(commandName))(params)) {
        return true;
      } else {
        attachedObjs_.clear();
        return false;
      }
    } catch (...) {
      //std::cerr << "unknown command: " << commandName << std::endl;
      attachedObjs_.clear();
      return false;
    }
  }

  bool Controller::attachModelItem (BodyItemPtr object, int target) // attach "object" to "target"
  {
    BodyPtr robotBody = getRobotItem()->body();

    try {
      Link* handLink = robotBody->link(getToolLink(target));
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
      MessageView::instance()->putln((boost::format("[attachModeItem] unknown link ID: %d") % target).str());
      return false;
    }

    return true;
  }

  bool Controller::detachModelItem (BodyItemPtr object, int target)
  {
    BodyPtr robotBody = getRobotItem()->body();

    try {
      Link* handLink = robotBody->link(getToolLink(target));
      Link* objectLink = object->body()->link(0);

      for (std::vector<AttachedModel*>::iterator it = attachedObjs_.begin(); it != attachedObjs_.end();) {
        if ((*it)->handLink == handLink && (*it)->objectLink == objectLink) {
          it = attachedObjs_.erase(it);
          MessageView::instance()->putln("detachModelItem");
        }
        else {
          ++it;
        }
      }
    }
    catch (...) {
      MessageView::instance()->putln((boost::format("[attachModeItem] unknown link ID: %d") % target).str());
      return false;
    }

    return true;
  }

  BodyItem* Controller::getRobotItem ()
  {
    return findItemByName(rootName);
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
                                    std::list<A> arguments, Command* commandFunc)
  {
    CommandDefParam* cmd = new CommandDefParam(registeredCommands_++, QString::fromStdString(internalName),
                                               QString::fromStdString(displayName), QString::fromStdString(returnType));
    for (auto arg: arguments) {
      ArgumentDefParam* a = new ArgumentDefParam(arg.name(), arg.type(), arg.numElms());
      cmd->addArgument(a);
    }

    commands_[internalName] = commandFunc;
    commandDefs_.push_back(cmd);
  }

  bool Controller::executeJointMotion(BodyItem* robotItem, double duration)
  {
    std::stringstream ss;
    ss << "executeJointMotion : " << duration;
    MessageView::instance()->putln(ss.str());

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

    std::stringstream ss2;
    ss2 << "executeJointMotion Finished";
    MessageView::instance()->putln(ss2.str());

    return true;
  }

  bool Controller::executeCartesianMotion(BodyItem* robotItem, Link* wrist, JointPathPtr jointPath, double duration)
  {
    for (double time = 0.0; time < duration+dt_; time += dt_) {
      if (time > duration) { time = duration; }

#ifndef _WIN32
      auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(dt_*1000));
#endif

      SE3 tf = ci.interpolate(time);

      if (jointPath->calcInverseKinematics(tf.translation(),
                                           wrist->calcRfromAttitude(tf.rotation().toRotationMatrix()))) {
        updateAttachedModels();
        robotItem->notifyKinematicStateChange(true);
        QCoreApplication::processEvents();
#ifdef _WIN32
        Sleep((int)(dt_*1000));
#else
        std::this_thread::sleep_until(abs_time);
#endif
      } else {
        return false;
      }
    }

    return true;
  }

}
