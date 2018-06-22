/**
   @author Ryo Hanai
*/

#include <sstream>
#include "FollowTrajectoryControllerROS.h"

#include <cnoid/RootItem>
#include <cnoid/BodyItem>

using namespace cnoid;

namespace teaching
{

  FollowTrajectoryController* FollowTrajectoryController::instance()
  {
    static FollowTrajectoryController* controller = new FollowTrajectoryController();
    return controller;
  }

  FollowTrajectoryController::FollowTrajectoryController()
  {
    name_ = "teaching_plugin";
    topic_name_ = "joint_trajectory";

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv;
      ros::init(argc, argv, name_);
    }

    node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(topic_name_, 1);
  }

  BodyItem* findItemByName (const std::string& name)
  {
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(RootItem::instance());
    for (size_t i = 0; i < bodyItems.size(); i++) {
      BodyItem* item = bodyItems.get(i);
      if (item->name() == name) { return item; }
    }

    return NULL;
  }

  BodyItem* getRobotItem ()
  {
    BodyItem* robotItem = findItemByName(rootName);
    if (robotItem == NULL) {
      throw RobotNotFoundException(rootName);
    }

    return robotItem;
  }

  BodyPtr getRobotBody ()
  {
    BodyItem* robotItem = getRobotItem();
    BodyPtr robotBody = robotItem->body();
    return robotBody;
  }

  bool FollowTrajectoryController::sendTrajectory()
  {
    // actionにして、動作実行の終了待ちをするべきかも

    BodyPtr body = getRobotBody();
    int n = body->numJoints();
    for (int i = 0; i < n; i++)
    {
      std::cout << body->joint(i)->name() << std::endl;
      std::cout << body->joint(i)->q() << std::endl;
    }
    // joint->name().c_str();
    //const trajectory_msgs::JointTrajectoryPoint* traj;
    //trajectory_msgs::JointTrajectoryPoint traj;
    trajectory_msgs::JointTrajectory traj;
    //traj.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj.header.stamp = ros::Time::now();

    traj.joint_names.push_back("rarm1_joint");
    traj.joint_names.push_back("rarm2_joint");
    traj.joint_names.push_back("rarm3_joint");

    traj.points.resize(2);
    int ind = 0;
    traj.points[ind].positions.resize(3);
    traj.points[ind].positions[0] = 0.0;
    traj.points[ind].positions[1] = 0.1;
    traj.points[ind].positions[2] = 0.2;
    traj.points[ind].velocities.resize(3);
    for (size_t j = 0; j < 3; ++j)
    {
      traj.points[ind].velocities[j] = 0.0;
    }
    traj.points[ind].time_from_start = ros::Duration(1.0);

    ind++;
    traj.points[ind].positions.resize(3);
    traj.points[ind].positions[0] = 1.0;
    traj.points[ind].positions[1] = 1.1;
    traj.points[ind].positions[2] = 1.2;
    traj.points[ind].velocities.resize(3);
    for (size_t j = 0; j < 3; ++j)
    {
      traj.points[ind].velocities[j] = 0.0;
    }
    traj.points[ind].time_from_start = ros::Duration(2.0);
    traj_pub_.publish(traj);

  }

}
