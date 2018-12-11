/**
   @author Ryo Hanai
*/

#include <QCoreApplication>
#include "RobotiqGripperController.h"

namespace teaching
{

  bool RobotiqGripperController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ",", gripperID, ")");

    TPInterface& tpif = TPInterface::instance();
    BodyPtr body = tpif.getRobotBody();

    // IK
    const double a = -8.448133;
    const double b = 0.75585477;
    const double th = a * width + b;

    if (isReal) {
      JointTrajectory traj;
      std::vector<std::string> joint_names;
      joint_names.push_back(driverJoint_);

      VectorXd qGoal(1);
      qGoal[0] = -th;
      if (tpif.interpolate(joint_names, qGoal, duration, traj)) {
#ifdef ROS_ON
        ROSInterface::instance().sendGoal(client_, traj);
        return waitAndUpdateRobotModel();
#endif
      }
    } else {
      JointTrajectory traj;

      VectorXd qGoal(6);
      qGoal[0] = -th;
      qGoal[1] = -th;
      qGoal[2] = -th;
      qGoal[3] = -th;
      qGoal[4] = th;
      qGoal[5] = th;
      if (tpif.interpolate(gripperJoints_, qGoal, duration, traj)) {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }

  bool RobotiqGripperController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    printLog("not yet implemented");
    return true;
  }

#ifdef ROS_ON

#include <chrono>
#include <thread>

  void RobotiqGripperController::setTrajectoryActionClient (const std::string& actionName)
  {
    client_ = TrajClientPtr(new TrajClient(actionName));
  }

  void RobotiqGripperController::setJointStateListener (const std::string& topicName)
  {
    ROSInterface::instance().getNodeHandle()->subscribe(topicName, 1, &RobotiqGripperController::jointStateListener, this);
  }

  void RobotiqGripperController::jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    for (int i = 0; i < jointstate->name.size(); i++) {
      joint_state_[jointstate->name[i]] = jointstate->position[i];
    }
  }

  void RobotiqGripperController::updateRobotModel ()
  {

  }

  bool RobotiqGripperController::waitAndUpdateRobotModel ()
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
