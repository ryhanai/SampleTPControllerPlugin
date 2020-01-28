/**
   @author Ryo Hanai
*/

#include <QCoreApplication>
#include "SingleArmController.h"

#ifdef ROS_ON
#include <chrono>
#include <thread>
#endif

namespace teaching
{

  bool SingleArmController::moveL (std::vector<CompositeParamType>& params, bool isReal)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveL(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    TPInterface& tpif = TPInterface::instance();
    JointPathPtr joint_path = tpif.getJointPath(baseLinkName_, targetLinkName_);
    JointTrajectory traj;
    if (tpif.interpolate(joint_path, xyz, rpy, duration, traj)) {
      if (isReal) {
#ifdef ROS_ON
        ROSInterface::instance().sendGoal(client_, traj);
        return waitAndUpdateRobotModel();
#endif
      } else {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }

  bool SingleArmController::moveJ (std::vector<CompositeParamType>& params, bool isReal)
  {
    VectorXd qGoal(boost::get<VectorX>(params[0]));
    double duration = boost::get<double>(params[1]);
    int armID = boost::get<int>(params[2]);
    printLog("moveJ(", qGoal.transpose(), ", ", duration, ", ", armID, ")");

    TPInterface& tpif = TPInterface::instance();
    JointPathPtr joint_path = tpif.getJointPath(baseLinkName_, targetLinkName_);

    JointTrajectory traj;
    if (tpif.interpolate(joint_path, qGoal, duration, traj)) {
      if (isReal) {
        printLog("goInitial(isReal) is not yet implemented");
        return true;
      } else {
        return tpif.followTrajectory(traj);
      }
    }
  }

  bool SingleArmController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    double duration = boost::get<double>(params[0]);
    printLog("goInitial(", duration, ")");

    TPInterface& tpif = TPInterface::instance();
    VectorXd qStart = tpif.getCurrentJointAngles();
    VectorXd qGoal = tpif.getStandardPose();

    JointTrajectory traj;
    BodyPtr body = tpif.getRobotBody();
    for (int i = 0; i < body->numJoints(); i++) {
      std::get<0>(traj).push_back(body->joint(i)->name());
    }

    if (tpif.interpolate(qStart, qGoal, duration, traj)) {
      if (isReal) {
        printLog("goInitial(isReal) is not yet implemented");
        return true;
      } else {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }


#ifdef ROS_ON
  
  void SingleArmController::setTrajectoryActionClient (const std::string& actionName)
  {
    client_ = TrajClientPtr(new TrajClient(actionName));
  }

  void SingleArmController::setJointStateListener (const std::string& topicName)
  {
    ROSInterface::instance().getNodeHandle()->subscribe(topicName, 1, &SingleArmController::jointStateListener, this);
  }

  void SingleArmController::jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    for (int i = 0; i < jointstate->name.size(); i++) {
      joint_state_[jointstate->name[i]] = jointstate->position[i];
    }
  }

  void SingleArmController::updateRobotModel ()
  {
    JointPathPtr joint_path = TPInterface::instance().getJointPath(baseLinkName_, targetLinkName_);
    for (int i = 0; i < joint_path->numJoints(); i++) {
      joint_path->joint(i)->q() = joint_state_[joint_path->joint(i)->name()];
    }
  }

  bool SingleArmController::waitAndUpdateRobotModel ()
  {
    // bool completed = client->waitForResult(ros::Duration(10.0));
    auto start = std::chrono::system_clock::now();
    while (!client_->getState().isDone() && ros::ok()) {
      updateRobotModel ();

      auto now = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = now - start;
      // std::cout << "elapsed time[s] = " << elapsed_seconds.count() << std::endl;
      if (elapsed_seconds.count() > 10.0) {
        return false;
      }
      QCoreApplication::processEvents();
      TPInterface::instance().updateAttachedModels();
      auto next = now + std::chrono::milliseconds((int)(TPInterface::instance().getTimeStep()*1000));
      std::this_thread::sleep_until(next);
    }

    return true;
  }

#endif

}
