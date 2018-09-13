/**
   @author Ryo Hanai
*/

#include <sstream>
#include <cnoid/ValueTree> // for Listing
#include <QCoreApplication>
#include "UR3dualController.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif

namespace teaching
{

  UR3dualController* UR3dualController::instance()
  {
    static UR3dualController* controller = new UR3dualController();
    return controller;
  }

  UR3dualController::UR3dualController()
  {
    registerCommands();
    setToolLink(0, "larm_wrist_3_joint");
    setToolLink(1, "rarm_wrist_3_joint");
  }

  bool UR3dualController::MoveArmCommand::operator()(std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

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
  }

  bool UR3dualController::MoveGripperCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ", ", gripperID, ")");

    std::vector<std::string> gripperLinks;
    if (gripperID == 0) {
      gripperLinks = {"lgripper_finger1_finger_tip_joint",
                      "lgripper_finger1_inner_knuckle_joint",
                      "lgripper_finger1_joint",
                      "lgripper_finger2_finger_tip_joint",
                      "lgripper_finger2_inner_knuckle_joint",
                      "lgripper_finger2_joint"};
    } else if (gripperID == 1) {
      gripperLinks = {"rgripper_finger1_finger_tip_joint",
                      "rgripper_finger1_inner_knuckle_joint",
                      "rgripper_finger1_joint",
                      "rgripper_finger2_finger_tip_joint",
                      "rgripper_finger2_inner_knuckle_joint",
                      "rgripper_finger2_joint"};
    } else {
      printLog("unknown gripperID: ", gripperID);
      return false;
    }

    return c_->executeGripperMotion(gripperLinks, width);
  }

  bool UR3dualController::GoInitialCommand::operator()(std::vector<CompositeParamType>& params)
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

  bool UR3dualController::MoveCommand::operator()(std::vector<CompositeParamType>& params)
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

  void UR3dualController::registerCommands()
  {
    registerCommand("moveArm", "Arm", "boolean",
                    {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)},
                    new MoveArmCommand(this)); // 0=left, 1=right
    registerCommand("moveGripper", "Gripper", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)},
                    new MoveGripperCommand(this)); // 0=left, 1=right
    registerCommand("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)},
                    new GoInitialCommand(this));
    registerCommand("move", "Both arms", "boolean",
                    {A("leftHandXyz", "double", 3), A("leftHandRpy", "double", 3), A("rightHandXyz", "double", 3),
                        A("rightHandRpy", "double", 3), A("torsoAngle", "double", 1), A("tm", "double", 1)},
                    new MoveCommand(this));
  }

  bool UR3dualController::executeDualArmMotion()
  {
    printLog("UR3dualController::executeDualArmMotion [ NOT IMPLEMENTED ]");
    return false;
  }

  bool UR3dualController::executeGripperMotion (const std::vector<std::string>& gripperLinks, double width)
  {
    printLog("UR3dualController::executeGripperMotion");
    double duration = jointInterpolator.domainUpper();

    BodyPtr body = getRobotBody();
    VectorXd qCur = getCurrentJointAngles(body);
    jointInterpolator.clear();
    jointInterpolator.appendSample(0, qCur);

    const double a = -8.448133;
    const double b = 0.75585477;
    const double th = a * width + b;

    qCur[body->link(gripperLinks[0])->jointId()] = -th;
    qCur[body->link(gripperLinks[1])->jointId()] = -th;
    qCur[body->link(gripperLinks[2])->jointId()] = -th;
    qCur[body->link(gripperLinks[3])->jointId()] = -th;
    qCur[body->link(gripperLinks[4])->jointId()] = th;
    qCur[body->link(gripperLinks[5])->jointId()] = th;

    jointInterpolator.appendSample(duration, qCur);
    jointInterpolator.update();
    return executeJointMotion();
  }

}
