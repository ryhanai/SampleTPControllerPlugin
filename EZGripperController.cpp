/**
   @author Ryo Hanai
*/

#ifdef _WIN32
#include <Windows.h>
#else
#include <chrono>
#include <thread>
#endif
#include <QCoreApplication>
#include <algorithm>
#include "EZGripperController.h"

namespace teaching
{

  bool EZGripperController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    printLog("moveGripper(", width, ", ", duration, ")");

    TPInterface& tpif = TPInterface::instance();
    BodyPtr body = tpif.getRobotBody();

    // IK: width -> theta (適当)
    double th = 111.2 - width * 1000.0;
    //th = 0 ? th < 0 : 111.2;
    th = std::min(std::max(0.0, th), 111.2);

    if (isReal) {
#ifdef ROS_ON
      control_msgs::GripperCommandGoal goal;
      // translate width into th
      goal.command.position = 0.1;
      client_->sendGoal(goal);
      bool finishedBeforeTimeout = client_->waitForResult(ros::Duration(30));
      if (!finishedBeforeTimeout) {
        return true;
      }

#endif
      return false;
    } else {
      JointTrajectory traj;
      VectorXd qGoal(4);
      qGoal[0] = toRad(th);
      qGoal[1] = toRad(111.2 - th);
      qGoal[2] = toRad(th);
      qGoal[3] = toRad(111.2 - th);
      if (tpif.interpolate(gripperJoints_, qGoal, duration, traj)) {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }

  bool EZGripperController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    printLog("not yet implemented");
    return true;
  }

  #ifdef ROS_ON
  void EZGripperController::setGripperActionClient (const std::string& actionName)
  {
    client_ = GripperActionClientPtr(new GripperActionClient(
                                       "/crane_plus_gripper/gripper_command",
                                       "true"));
    // client_->waitForServer();
  }

  void EZGripperController::setJointStateListener (const std::string& topicName)
  {
    ROSInterface::instance().getNodeHandle()->subscribe(topicName, 1, &EZGripperController::jointStateListener, this);
  }

  void EZGripperController::jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    for (int i = 0; i < jointstate->name.size(); i++) {
      joint_state_[jointstate->name[i]] = jointstate->position[i];
    }
  }

  void EZGripperController::updateRobotModel ()
  {
    
  }

  bool EZGripperController::waitAndUpdateRobotModel ()
  {
    auto start = std::chrono::system_clock::now();
    while (!client_->getState().isDone() && ros::ok()) {
      updateRobotModel ();

      auto now = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = now - start;
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
