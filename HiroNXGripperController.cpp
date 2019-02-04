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

#include "HiroNXGripperController.h"
#ifdef ROS_ON
#include "ROSUtil.h"
#endif

namespace teaching
{

  bool HiroNXGripperController::moveGripper (std::vector<CompositeParamType>& params, bool isReal)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ",", gripperID, ")");

    TPInterface& tpif = TPInterface::instance();
    BodyPtr body = tpif.getRobotBody();

    // IK
    const double th = asin(((width/2.0) - 0.015) / 0.042);

    JointTrajectory traj;
    VectorXd qGoal(4);
    qGoal[0] = th;
    qGoal[1] = -th;
    qGoal[2] = -th;
    qGoal[3] = th;
    if (tpif.interpolate(gripperJoints_, qGoal, duration, traj)) {
      if (isReal) {
        printLog("HiroNXGripperController does not support real robot");
        return false;
      } else {
        return tpif.followTrajectory(traj);
      }
    }

    return false;
  }

  bool HiroNXGripperController::goInitial (std::vector<CompositeParamType>& params, bool isReal)
  {
    printLog("not yet implemented");
    return true;
  }

#ifdef ROS_ON

  void HiroNXGripperController::setTrajectoryActionClient (const std::string& actionName)
  {
    client_ = TrajClientPtr(new TrajClient(actionName));
  }

  void HiroNXGripperController::setJointStateListener (const std::string& topicName)
  {
    ROSInterface::instance().getNodeHandle()->subscribe(topicName, 1, &HiroNXGripperController::jointStateListener, this);
  }

  void HiroNXGripperController::jointStateListener (const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    for (int i = 0; i < jointstate->name.size(); i++) {
      joint_state_[jointstate->name[i]] = jointstate->position[i];
    }
  }

  void HiroNXGripperController::updateRobotModel ()
  {

  }

  bool HiroNXGripperController::waitAndUpdateRobotModel ()
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
