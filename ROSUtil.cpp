/**
   @author Ryo Hanai
*/

#include "ROSUtil.h"

using namespace cnoid;

namespace teaching
{

  ROSInterface& ROSInterface::instance ()
  {
    static ROSInterface rosif;
    return rosif;
  }

  ROSInterface::ROSInterface ()
  {
    node_name_ = "teaching_plugin";

    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv;
      ros::init(argc, argv, node_name_);
    }

    node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());    spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner_->start();
  }

  void ROSInterface::addTrajectoryClient (int id, std::string action_name)
  {
    clientTable_[id] = TrajClientPtr(new TrajClient(action_name));
  }

  void ROSInterface::setJointStateTopic (std::string topic_name)
  {
    js_sub_ = node_->subscribe("/joint_states", 1,
                               &ROSInterface::updateState, this);
  }

  bool ROSInterface::followTrajectory (const Trajectory& traj)
  {
    return false;
  }

  bool ROSInterface::followTrajectory (int toolNumber, const Trajectory& traj)
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory rtraj;

    for (auto wp : traj) {
      double tm = std::get<0>(wp);
      VectorXd q = std::get<1>(wp);

      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.resize(q.size());
      p.velocities.resize(q.size());
      for (int i = 0; i < q.size(); i++) {
        p.positions[i] = q[i];
        p.velocities[i] = 0.1;
      }
      p.time_from_start = ros::Duration(tm);
      rtraj.points.push_back(p);
    }

    goal.trajectory = rtraj;
    TrajClientPtr traj_client = clientTable_[toolNumber];
    traj_client->sendGoal(goal);
    while (!traj_client->getState().isDone() && ros::ok()) {
      syncWithReal();
    }

    syncWithReal();
    return true;
  }

  void ROSInterface::updateState (const sensor_msgs::JointState::ConstPtr& joint_state)
  {

  }

  void ROSInterface::syncWithReal ()
  {

  }

  void ROSInterface::cancelFollowTrajectory ()
  {
    
  }

}
