/**
   @author Ryo Hanai
*/

#include <sstream>
#include <cnoid/ValueTree> // for Listing
#include <QCoreApplication>
#include "SampleHiroController.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif

namespace teaching
{

  SampleHiroController* SampleHiroController::instance()
  {
    static SampleHiroController* controller = new SampleHiroController();
    return controller;
  }

  SampleHiroController::SampleHiroController()
  {
    registerCommands();
    setToolLink(0, "LARM_JOINT5");
    setToolLink(1, "RARM_JOINT5");
  }

  bool SampleHiroController::MoveTorsoCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double angle = toRad(boost::get<double>(params[0]));
    double duration = boost::get<double>(params[1]);
    printLog("moveTorso(", angle, ", ", duration, ")");

    BodyPtr body = c_->getRobotBody();
    VectorXd qCur = c_->getCurrentJointAngles(body);
    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, qCur);
    qCur[body->link("CHEST_JOINT0")->jointId()] = angle;
    c_->jointInterpolator.appendSample(duration, qCur);
    c_->jointInterpolator.update();

    return c_->executeJointMotion();
  }

  bool SampleHiroController::MoveHeadCommand::operator()(std::vector<CompositeParamType>& params)
  {
    VectorX angles = (boost::get<Vector2>(params[0]));
    VectorX angles2 = toRad(angles);
    double duration = boost::get<double>(params[1]);
    printLog("moveHead(", angles2.transpose(), ", ", duration, ")");

    BodyPtr body = c_->getRobotBody();
    VectorXd qCur = c_->getCurrentJointAngles(body);
    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, qCur);
    qCur[body->link("HEAD_JOINT0")->jointId()] = angles2[0];
    qCur[body->link("HEAD_JOINT1")->jointId()] = angles2[1];
    c_->jointInterpolator.appendSample(duration, qCur);
    c_->jointInterpolator.update();

    return c_->executeJointMotion();
  }

  bool SampleHiroController::MoveArmCommand::operator()(std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    try {
      BodyPtr body = c_->getRobotBody();
      Link* base = body->rootLink();
      Link* wrist = body->link(c_->getToolLinkName(armID));

      JointPathPtr jointPath = getCustomJointPath(body, base, wrist);
      jointPath->calcForwardKinematics();

      c_->ci.clear();
      c_->ci.appendSample(0, wrist->p(), wrist->attitude());
      c_->ci.appendSample(duration, xyz, rotFromRpy(rpy));
      c_->ci.update();
      return c_->executeCartesianMotion(wrist, jointPath);
    } catch (...) {
      printLog("unknown armID: ", armID);
      return false;
    }
  }

  bool SampleHiroController::MoveGripperCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ", ", gripperID, ")");

    std::vector<std::string> gripperLinks;
    if (gripperID == 0) {
      gripperLinks = {"LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3"};
    } else if (gripperID == 1) {
      gripperLinks = {"RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3"};
    } else {
      printLog("unknown gripperID: ", gripperID);
      return false;
    }

    return c_->executeGripperMotion(gripperLinks, width);
  }

  bool SampleHiroController::ScrewCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double torque = boost::get<double>(params[0]);
    double max_depth = boost::get<double>(params[1]);
    printLog("screw(", torque, ", ", max_depth, "): not yet implemented");
    // BodyPtr body = c_->getRobotBody();
    // Link* base = body->rootLink();
    // string linkName = "RARM_JOINT5";
    // Link* wrist = body->link(linkName);
    // JointPathPtr jointPath = getCustomJointPath(body, base, wrist);
    return true;
  }

  bool SampleHiroController::GoInitialCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);

    BodyPtr body = c_->getRobotBody();
    VectorXd qCur = c_->getCurrentJointAngles(body);

    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, qCur);
    int jointIndex = 0;
    const Listing& pose = *body->info()->findListing("standardPose");
    if(pose.isValid()){
      const int nn = std::min(pose.size(), body->numJoints());
      while(jointIndex < nn){
        qCur[jointIndex] = radian(pose[jointIndex].toDouble());
        jointIndex++;
      }
      c_->jointInterpolator.appendSample(duration, qCur);
      c_->jointInterpolator.update();
      return c_->executeJointMotion();
    }
    return false;
  }

  bool SampleHiroController::MoveCommand::operator()(std::vector<CompositeParamType>& params)
  {
    Vector3 leftHandXyz(boost::get<Vector3>(params[0]));
    Vector3 leftHandRpy(boost::get<Vector3>(params[1]));
    Vector3 leftHandRpy2 = toRad(leftHandRpy);
    Vector3 rightHandXyz(boost::get<Vector3>(params[2]));
    Vector3 rightHandRpy(boost::get<Vector3>(params[3]));
    Vector3 rightHandRpy2 = toRad(rightHandRpy);
    double torsoAngle2 = toRad(boost::get<double>(params[4]));
    double duration = boost::get<double>(params[5]);
    printLog("move(", leftHandXyz.transpose(), ", ", leftHandRpy2.transpose(), ", ",
             rightHandXyz.transpose(), ", ", rightHandRpy2.transpose(), ", ", duration, ")");

    BodyPtr body = c_->getRobotBody();
    Link* base = body->rootLink();
    Link* lwrist = body->link("LARM_JOINT5");
    Link* rwrist = body->link("RARM_JOINT5");
    JointPathPtr lJointPath = getCustomJointPath(body, base, lwrist);
    lJointPath->calcForwardKinematics();
    JointPathPtr rJointPath = getCustomJointPath(body, base, rwrist);
    rJointPath->calcForwardKinematics();

    c_->ci.clear();
    c_->ci.appendSample(0, lwrist->p(), lwrist->attitude());
    c_->ci.appendSample(duration, leftHandXyz, rotFromRpy(leftHandRpy2));
    c_->ci.update();
    c_->ci2.clear();
    c_->ci2.appendSample(0, rwrist->p(), rwrist->attitude());
    c_->ci2.appendSample(duration, rightHandXyz, rotFromRpy(rightHandRpy2));
    c_->ci2.update();

    return c_->executeDualArmMotion();
  }

  bool SampleHiroController::MoveGripperTestCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripperTest(", width, ", ", duration, ", ", gripperID, ")");

    cnoid::VectorX result(3);
    result[0] = 1.23;
    result[1] = 9.87;
    result[2] = 7.53;
    CompositeParamType retVal = result;
    params[3] = retVal;

    return true;
  }

  void SampleHiroController::registerCommands()
  {
    registerCommand("moveTorso", "Torso", "boolean", {A("angle", "double", 1), A("tm", "double", 1)},
                    new MoveTorsoCommand(this));
    registerCommand("moveArm", "Arm", "boolean",
                    {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)},
                    new MoveArmCommand(this)); // 0=left, 1=right
    registerCommand("moveHead", "Head", "boolean", {A("angles", "double", 2), A("tm", "double", 1)},
                    new MoveHeadCommand(this));
    registerCommand("moveGripper", "Gripper", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)},
                    new MoveGripperCommand(this)); // 0=left, 1=right
    registerCommand("screw", "Screw", "boolean", {A("torque", "double", 1), A("max_depth", "double", 1)},
                    new ScrewCommand(this));
    registerCommand("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)},
                    new GoInitialCommand(this));
    registerCommand("move", "Both arms", "boolean",
                    {A("leftHandXyz", "double", 3), A("leftHandRpy", "double", 3), A("rightHandXyz", "double", 3),
                        A("rightHandRpy", "double", 3), A("torsoAngle", "double", 1), A("tm", "double", 1)},
                    new MoveCommand(this));
    registerCommand("moveGripperTest", "GripperTest", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1),
                        A("result", "double", 3, A::var_prop::out)},
                    new MoveGripperTestCommand(this));
  }

  bool SampleHiroController::executeDualArmMotion()
  {
    printLog("SampleHiroController::executeDualArmMotion");

    BodyPtr body = getRobotBody();
    Link* base = body->link("CHEST_JOINT0");
    Link* lwrist = body->link("LARM_JOINT5");
    Link* rwrist = body->link("RARM_JOINT5");
    JointPathPtr lJointPath = getCustomJointPath(body, base, lwrist);
    JointPathPtr rJointPath = getCustomJointPath(body, base, rwrist);

    double dt = getTimeStep();
    double duration = ci.domainUpper();

    for (double time = 0.0; time < duration+dt; time += dt) {
      if (time > duration) { time = duration; }

#ifndef _WIN32
      auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(dt*1000));
#endif

      SE3 ltf = ci.interpolate(time);
      SE3 rtf = ci2.interpolate(time);
      if (!lJointPath->calcInverseKinematics(ltf.translation(),
                                             lwrist->calcRfromAttitude(ltf.rotation().toRotationMatrix())))
      {
        printLog("larm IK failed");
        return false;
      }

      if (!rJointPath->calcInverseKinematics(rtf.translation(),
                                             rwrist->calcRfromAttitude(rtf.rotation().toRotationMatrix())))
      {
        printLog("rarm IK failed");
        return false;
      }

      updateAttachedModels();
      BodyItem* robotItem = getRobotItem();
      robotItem->notifyKinematicStateChange(true);
      QCoreApplication::processEvents();
#ifdef _WIN32
      Sleep((int)(dt*1000));
#else
      std::this_thread::sleep_until(abs_time);
#endif
    }

    return true;
  }

  bool SampleHiroController::executeGripperMotion (const std::vector<std::string>& gripperLinks, double width)
  {
    printLog("SampleHiroController::executeGripperMotion");
    double duration = jointInterpolator.domainUpper();

    BodyPtr body = getRobotBody();
    VectorXd qCur = getCurrentJointAngles(body);
    jointInterpolator.clear();
    jointInterpolator.appendSample(0, qCur);

    double th = asin(((width/2.0) - 0.015) / 0.042);
    qCur[body->link(gripperLinks[0])->jointId()] = th;
    qCur[body->link(gripperLinks[1])->jointId()] = -th;
    qCur[body->link(gripperLinks[2])->jointId()] = -th;
    qCur[body->link(gripperLinks[3])->jointId()] = th;
    jointInterpolator.appendSample(duration, qCur);
    jointInterpolator.update();
    return executeJointMotion();
  }
  
}
