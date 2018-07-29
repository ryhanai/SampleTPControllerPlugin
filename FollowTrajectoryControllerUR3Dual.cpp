/**
   @author Ryo Hanai
*/

#include <sstream>
#include <cnoid/ValueTree> // for Listing
#include "FollowTrajectoryControllerUR3Dual.h"

#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <QCoreApplication>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// 
#include <chrono>
#include <thread>

//

using namespace cnoid;

namespace teaching
{

  namespace pi = moveit::planning_interface;
  
  FollowTrajectoryControllerUR3Dual* FollowTrajectoryControllerUR3Dual::instance()
  {
    static FollowTrajectoryControllerUR3Dual* controller = new FollowTrajectoryControllerUR3Dual();
    return controller;
  }

  FollowTrajectoryControllerUR3Dual::FollowTrajectoryControllerUR3Dual()
  {
    registerCommands();
    setToolLink(0, "larm_wrist_3_joint");
    setToolLink(1, "rarm_wrist_3_joint");
    name_ = "teaching_plugin";
    rarm_topic_name_ = "/right_arm/follow_joint_trajectory/goal";
    larm_topic_name_ = "/left_arm/follow_joint_trajectory/goal";
    rhand_topic_name_ = "/right_hand/joint_trajectory_controller/follow_joint_trajectory/goal";
    lhand_topic_name_ = "/left_hand/joint_trajectory_controller/follow_joint_trajectory/goal";

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv;
      ros::init(argc, argv, name_);
    }

    node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    // rarm_traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(rarm_topic_name_, 1);
    // larm_traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(larm_topic_name_, 1);
    // rhand_traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(rhand_topic_name_, 1);
    // lhand_traj_pub_ = node_->advertise<trajectory_msgs::JointTrajectory>(lhand_topic_name_, 1);

    rarm_traj_client_ = TrajClientPtr(new TrajClient("/right_arm/follow_joint_trajectory", true));
    larm_traj_client_ = TrajClientPtr(new TrajClient("/left_arm/follow_joint_trajectory", true));
    rhand_traj_client_ = TrajClientPtr(new TrajClient("/right_hand/joint_trajectory_controller/follow_joint_trajectory", true));
    lhand_traj_client_ = TrajClientPtr(new TrajClient("/left_hand/joint_trajectory_controller/follow_joint_trajectory", true));
    
    js_sub_ = node_->subscribe("/joint_states", 1, &FollowTrajectoryControllerUR3Dual::updateState, this);
    spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner_->start();

    // planning_scene_interface_ = boost::shared_ptr<pi::PlanningSceneInterface>(new pi::PlanningSceneInterface);
    // rarm_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryControllerUR3Dual::RARM_GROUP));
    // larm_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryControllerUR3Dual::LARM_GROUP));
    // rhand_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryControllerUR3Dual::RHAND_GROUP));
    // lhand_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryControllerUR3Dual::LHAND_GROUP));

    // rarm_group_->setPoseReferenceFrame("stage_link");
    // larm_group_->setPoseReferenceFrame("stage_link");
  }

  bool FollowTrajectoryControllerUR3Dual::MoveArmCommand::operator()(std::vector<CompositeParamType>& params)
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

    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory traj;
    if (c_->interpolate(wrist, jointPath, traj)) {
      goal.trajectory = traj;
      TrajClientPtr traj_client;
      if (armID == 0) {
        traj_client = c_->larm_traj_client_;
      } else {
        traj_client = c_->rarm_traj_client_;
      }

      traj_client->sendGoal(goal);
      while (!traj_client->getState().isDone()) {
        c_->syncWithReal();
      }

      // auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(duration*1000));
      // std::this_thread::sleep_until(abs_time);

      c_->syncWithReal();
      return true;
    } else {
      return false;
    }
  }

  bool FollowTrajectoryControllerUR3Dual::MoveGripperCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ", ", gripperID, ")");

    // least-square fitted
    const double a = -8.448133;
    const double b = 0.75585477;
    const double qref = a * width + b;

    BodyItem* robotItem = c_->getRobotItem();
    BodyPtr body = c_->getRobotBody();
    VectorXd qCur = c_->getCurrentJointAngles(body);
    double dt = c_->getTimeStep();
    
    std::string gripperJoint;
    std::string gripperDriverJoint;
    TrajClientPtr traj_client;
    if (gripperID == 0) { // LHAND
      gripperJoint = "lgripper_finger2_joint";
      gripperDriverJoint = "lhand_right_driver_joint";
      traj_client = c_->lhand_traj_client_;
    } else if (gripperID == 1) {
      gripperJoint = "rgripper_finger2_joint";
      gripperDriverJoint = "rhand_right_driver_joint";
      traj_client = c_->rhand_traj_client_;
    } else {
      printLog("unknown gripperID: ", gripperID);
      return false;
    }

    VectorXd q;
    q.resize(1);
    q[0] = qCur[body->link(gripperJoint)->jointId()];

    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, q);
    q[0] = qref;
    c_->jointInterpolator.appendSample(duration, q);
    c_->jointInterpolator.update();

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    int nJoints = 1;
    traj.joint_names.push_back(gripperDriverJoint);

    for (double time = 0.0; time < duration+dt; time += dt) {
      if (time > duration) { time = duration; }
      VectorXd qRef;
      qRef = c_->jointInterpolator.interpolate(time);

      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.resize(nJoints);
      p.velocities.resize(nJoints);
      for (int i = 0; i < nJoints; i++) {
        p.positions[i] = qRef[i];
        p.velocities[i] = 0.0;
      }
      p.time_from_start = ros::Duration(time);
      traj.points.push_back(p);
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    traj_client->sendGoal(goal);
    while (!traj_client->getState().isDone()) {
      c_->syncWithReal();
    }

    // auto abs_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)(duration*1000));
    // std::this_thread::sleep_until(abs_time);
    // robotItem->notifyKinematicStateChange(true);
    // QCoreApplication::processEvents();

    c_->syncWithReal();
    return true;
  }

  bool FollowTrajectoryControllerUR3Dual::GoInitialCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);

    // RARM
    BodyItem* robotItem = c_->getRobotItem();
    BodyPtr body = robotItem->body();
    Link* base = body->rootLink();

    Link* rwrist = body->link(c_->getToolLinkName(1));
    JointPathPtr rJointPath = getCustomJointPath(body, base, rwrist);
    VectorXd q0;
    q0.resize(rJointPath->numJoints());
    for (int i = 0; i < rJointPath->numJoints(); i++) {
      q0[i] = rJointPath->joint(i)->q();
    }
    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, q0);
    q0[0] = radian(60);
    q0[1] = radian(-34);
    q0[2] = radian(78);
    q0[3] = radian(-94);
    q0[4] = radian(-218);
    q0[5] = radian(-157);
    c_->jointInterpolator.appendSample(duration, q0);
    c_->jointInterpolator.update();

    trajectory_msgs::JointTrajectory traj;
    if (c_->interpolateJ(rJointPath, traj)) {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = traj;
      c_->rarm_traj_client_->sendGoal(goal);
      while (!c_->rarm_traj_client_->getState().isDone()) {
        c_->syncWithReal();
        usleep(50000);
      }
    }

    // LARM
    Link* lwrist = body->link(c_->getToolLinkName(0));
    JointPathPtr lJointPath = getCustomJointPath(body, base, lwrist);
    VectorXd q1;
    q1.resize(lJointPath->numJoints());
    for (int i = 0; i < lJointPath->numJoints(); i++) {
      q1[i] = lJointPath->joint(i)->q();
    }
    c_->jointInterpolator.clear();
    c_->jointInterpolator.appendSample(0, q1);
    q1[0] = radian(-64);
    q1[1] = radian(-133);
    q1[2] = radian(-70);
    q1[3] = radian(250);
    q1[4] = radian(-160);
    q1[5] = radian(-6);
    c_->jointInterpolator.appendSample(duration, q1);
    c_->jointInterpolator.update();

    trajectory_msgs::JointTrajectory traj2;
    if (c_->interpolateJ(lJointPath, traj2)) {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = traj2;
      c_->larm_traj_client_->sendGoal(goal);
      while (!c_->larm_traj_client_->getState().isDone()) {
        c_->syncWithReal();
        usleep(50000);
      }
    }

    // for (int i = 0; i < rJointPath->numJoints(); i++) {
    //   rJointPath->joint(i)->q() = q0[i];
    // }
    // for (int i = 0; i < lJointPath->numJoints(); i++) {
    //   lJointPath->joint(i)->q() = q1[i];
    // }

    // c_->updateAttachedModels();
    // robotItem->notifyKinematicStateChange(true);

    c_->syncWithReal();
    return true;
  }

  void FollowTrajectoryControllerUR3Dual::syncWithReal()
  {
    BodyItem* robotItem = getRobotItem();
    BodyPtr body = getRobotBody();

    for (int i = 0; i < body->numJoints(); i++) {
      body->joint(i)->q() = joint_state_[body->joint(i)->name()];
    }

    for (auto x : joint_state_) {
      // std::cout << x.first << " " << x.second << std::endl;

      // Note that lhand_right_driver_joint and rhand_right_driver_joint does not exist
      // in the robot model.
      if (body->link(x.first)) {
        body->joint(body->link(x.first)->jointId())->q() = x.second;
      }
      double th = joint_state_["rhand_right_driver_joint"];
      body->joint(body->link("rgripper_finger1_finger_tip_joint")->jointId())->q() = -th;
      body->joint(body->link("rgripper_finger1_inner_knuckle_joint")->jointId())->q() = -th;
      body->joint(body->link("rgripper_finger1_joint")->jointId())->q() = -th;
      body->joint(body->link("rgripper_finger2_finger_tip_joint")->jointId())->q() = -th;
      body->joint(body->link("rgripper_finger2_inner_knuckle_joint")->jointId())->q() = th;
      body->joint(body->link("rgripper_finger2_joint")->jointId())->q() = th;

      th = joint_state_["lhand_right_driver_joint"];
      body->joint(body->link("lgripper_finger1_finger_tip_joint")->jointId())->q() = -th;
      body->joint(body->link("lgripper_finger1_inner_knuckle_joint")->jointId())->q() = -th;
      body->joint(body->link("lgripper_finger1_joint")->jointId())->q() = -th;
      body->joint(body->link("lgripper_finger2_finger_tip_joint")->jointId())->q() = -th;
      body->joint(body->link("lgripper_finger2_inner_knuckle_joint")->jointId())->q() = th;
      body->joint(body->link("lgripper_finger2_joint")->jointId())->q() = th;
    }

    updateAttachedModels();
    robotItem->notifyKinematicStateChange(true);
    QCoreApplication::processEvents();
  }

  void FollowTrajectoryControllerUR3Dual::updateState(const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    for (int i = 0; i < jointstate->name.size(); i++) {
      joint_state_[jointstate->name[i]] = jointstate->position[i];
    }
  }

  bool FollowTrajectoryControllerUR3Dual::interpolateJ(JointPathPtr jointPath, trajectory_msgs::JointTrajectory& traj)
  {
    double duration = jointInterpolator.domainUpper();
    double dt = getTimeStep();

    traj.points.clear();
    traj.header.stamp = ros::Time::now();
    for (int i = 0; i < jointPath->numJoints(); i++) {
      traj.joint_names.push_back(jointPath->joint(i)->name());
    }

    for (double time = 0.0; time < duration+dt; time += dt) {
      if (time > duration) { time = duration; }

      VectorXd qRef;
      qRef = jointInterpolator.interpolate(time);

      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.resize(jointPath->numJoints());
      p.velocities.resize(jointPath->numJoints());
      for (int i = 0; i < jointPath->numJoints(); i++) {
        p.positions[i] = qRef[i];
        p.velocities[i] = 0.0;
      }
      p.time_from_start = ros::Duration(time);
      traj.points.push_back(p);
    }

    return true;
  }

  bool FollowTrajectoryControllerUR3Dual::interpolate(Link* wrist, JointPathPtr jointPath, trajectory_msgs::JointTrajectory& traj)
  {
    double duration = ci.domainUpper();
    double dt = getTimeStep();

    traj.points.clear();
    traj.header.stamp = ros::Time::now();
    for (int i = 0; i < jointPath->numJoints(); i++) {
      traj.joint_names.push_back(jointPath->joint(i)->name());
    }

    for (double time = 0.0; time < duration+dt; time += dt) {
      if (time > duration) { time = duration; }
      SE3 tf = ci.interpolate(time);

      if (jointPath->calcInverseKinematics(tf.translation(),
                                           wrist->calcRfromAttitude(tf.rotation().toRotationMatrix()))) {
        updateAttachedModels();
        BodyItem* robotItem = getRobotItem();
        robotItem->notifyKinematicStateChange(true);
        BodyPtr body = getRobotBody();

        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.resize(jointPath->numJoints());
        p.velocities.resize(jointPath->numJoints());
        for (int i = 0; i < jointPath->numJoints(); i++) {
          p.positions[i] = jointPath->joint(i)->q();
          p.velocities[i] = 0.0;
        }
        p.time_from_start = ros::Duration(time);
        traj.points.push_back(p);

        //QCoreApplication::processEvents();
      } else {
        return false;
      }
    }

    return true;
  }


  
  bool FollowTrajectoryControllerUR3Dual::sendTrajectory()
  {
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();

    BodyPtr body = getRobotBody();
    int n = body->numJoints();
    int num_points = 1;

    for (int i = 0; i < n; i++)
    {
      //std::cout << body->joint(i)->name() << std::endl;
      traj.joint_names.push_back(body->joint(i)->name());
    }

    traj.points.resize(num_points);
    for (int ind = 0; ind < num_points; ind++)
    {
      traj.points[ind].positions.resize(n);
      traj.points[ind].velocities.resize(n);
      for (int i = 0; i < n; i++)
      {
        traj.points[ind].positions[i] = body->joint(i)->q();
        traj.points[ind].velocities[i] = 0.0;
      }
      traj.points[ind].time_from_start = ros::Duration(1.0);
    }

    // traj_pub_.publish(traj);

    // add time stamp and other required info
    // controllerをmoveitにする
    // sim => plan => 結果を受け取って可視化
    // real => execute （ボタン）
    // ハンド関節を除外して送る

#if 0
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
    //traj_pub_.publish(traj);
#endif

    // wait the completion of the execution
    // update the robot status in Scene View on the completion
    // subscribe to /joint_state and update the local model
  }

  void FollowTrajectoryControllerUR3Dual::registerCommands()
  {
    registerCommand("moveArm", "Arm", "boolean",
                    {A("xyz", "double", 3), A("rpy", "double", 3), A("tm", "double", 1), A("armID", "int", 1)},
                    new MoveArmCommand(this)); // 0=left, 1=right
    registerCommand("moveGripper", "Gripper", "boolean",
                    {A("width", "double", 1), A("tm", "double", 1), A("gripperID", "int", 1)},
                    new MoveGripperCommand(this)); // 0=left, 1=right
    registerCommand("goInitial", "Initial Pose", "boolean", {A("tm", "double", 1)},
                    new GoInitialCommand(this));
  }

}
