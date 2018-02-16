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
    static SampleHiroController* myController = new SampleHiroController();
    return myController;
  }

  SampleHiroController::SampleHiroController()
  {
    registerCommands();
    setToolLink(0, "LARM_JOINT5");
    setToolLink(1, "RARM_JOINT5");
  }

  bool SampleHiroController::MoveTorsoCommand::operator()(const std::vector<CompositeParamType>& params)
  {
    double angle = toRad(boost::get<double>(params[0]));
    double duration = boost::get<double>(params[1]);
    printLog("moveTorso(", angle, ", ", duration, ")");

    BodyPtr body = c_->getRobotBody();

    int n = body->numJoints();
    VectorXd qCur;
    qCur.resize(n);
    for (int i = 0; i < n; i++) { qCur[i] = body->joint(i)->q(); }
    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, qCur);
    qCur[body->link("CHEST_JOINT0")->jointId()] = angle;
    c_->jointInterpolator.appendSample(duration, qCur);
    c_->jointInterpolator.update();

    return c_->executeJointMotion(duration);
  }

  bool SampleHiroController::MoveHeadCommand::operator()(const std::vector<CompositeParamType>& params)
  {
    VectorX angles = (boost::get<Vector2>(params[0]));
    VectorX angles2 = toRad(angles);
    double duration = boost::get<double>(params[1]);
    printLog("moveHead(", angles2.transpose(), ", ", duration, ")");

    // Trajectory traj;
    // VectorXd q0 = body->joints->q();
    // traj.append(0, q0);
    // q0[body->link("HEAD_JOINT0")->jointId()] = (*angles2)[0];
    // q0[body->link("HEAD_JOINT1")->jointId()] = (*angles2)[1];
    // traj.append(duration, q0);
    // executeJointMotion(traj);

    BodyPtr body = c_->getRobotBody();
    c_->jointInterpolator.clear();
    VectorXd qCur;
    int n = body->numJoints();
    qCur.resize(n);

    for (int i = 0; i < n; i++) { qCur[i] = body->joint(i)->q(); }
    c_->jointInterpolator.appendSample(0, qCur);
    qCur[body->link("HEAD_JOINT0")->jointId()] = angles2[0];
    qCur[body->link("HEAD_JOINT1")->jointId()] = angles2[1];
    c_->jointInterpolator.appendSample(duration, qCur);
    c_->jointInterpolator.update();

    return c_->executeJointMotion(duration);
  }

  bool SampleHiroController::MoveArmCommand::operator()(const std::vector<CompositeParamType>& params)
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
      return c_->executeCartesianMotion(wrist, jointPath, duration);
    } catch (...) {
      printLog("unknown armID: ", armID);
      return false;
    }
  }

  bool SampleHiroController::MoveGripperCommand::operator()(const std::vector<CompositeParamType>& params)
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

    return c_->executeGripperMotion(gripperLinks, width, duration);
  }

  bool SampleHiroController::ScrewCommand::operator()(const std::vector<CompositeParamType>& params)
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

  bool SampleHiroController::GoInitialCommand::operator()(const std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);

    BodyPtr body = c_->getRobotBody();
    c_->jointInterpolator.clear();
    VectorXd qCur;
    int n = body->numJoints();
    qCur.resize(n);
    for (int i = 0; i < n; i++) {
      qCur[i] = body->joint(i)->q();
    }
    c_->jointInterpolator.appendSample(0, qCur);

    int jointIndex = 0;
    const Listing& pose = *body->info()->findListing("standardPose");
    if(pose.isValid()){
      const int nn = std::min(pose.size(), n);
      while(jointIndex < nn){
        qCur[jointIndex] = radian(pose[jointIndex].toDouble());
        jointIndex++;
      }

      c_->jointInterpolator.appendSample(duration, qCur);
      c_->jointInterpolator.update();
      return c_->executeJointMotion(duration);
    }
    return false;
  }

  bool SampleHiroController::MoveCommand::operator()(const std::vector<CompositeParamType>& params)
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

    return c_->executeDualArmMotion(duration);
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
  }

  bool SampleHiroController::executeDualArmMotion(double duration)
  {
    printLog("SampleHiroController::executeDualArmMotion");

    BodyPtr body = getRobotBody();
    Link* base = body->link("CHEST_JOINT0");
    Link* lwrist = body->link("LARM_JOINT5");
    Link* rwrist = body->link("RARM_JOINT5");
    JointPathPtr lJointPath = getCustomJointPath(body, base, lwrist);
    JointPathPtr rJointPath = getCustomJointPath(body, base, rwrist);

    double dt = getTimeStep();
    
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

  bool SampleHiroController::executeGripperMotion (const std::vector<std::string>& gripperLinks,
                                                   double width, double duration)
  {
    printLog("SampleHiroController::executeGripperMotion");

    BodyPtr body = getRobotBody();
    jointInterpolator.clear();
    VectorXd qCur;
    int n = body->numJoints();
    qCur.resize(n);

    for (int i = 0; i < n; i++) {
      qCur[i] = body->joint(i)->q();
    }
    jointInterpolator.appendSample(0, qCur);
 
    double th = asin(((width/2.0) - 0.015) / 0.042);
    qCur[body->link(gripperLinks[0])->jointId()] = th;
    qCur[body->link(gripperLinks[1])->jointId()] = -th;
    qCur[body->link(gripperLinks[2])->jointId()] = -th;
    qCur[body->link(gripperLinks[3])->jointId()] = th;
    jointInterpolator.appendSample(duration, qCur);
    jointInterpolator.update();
    return executeJointMotion(duration);
  }

}
