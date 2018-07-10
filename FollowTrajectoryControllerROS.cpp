/**
   @author Ryo Hanai
*/

#include <sstream>
#include <cnoid/ValueTree> // for Listing
#include "FollowTrajectoryControllerROS.h"

#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <QCoreApplication>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace cnoid;


namespace teaching
{

  namespace pi = moveit::planning_interface;
  
  FollowTrajectoryController* FollowTrajectoryController::instance()
  {
    static FollowTrajectoryController* controller = new FollowTrajectoryController();
    return controller;
  }

  FollowTrajectoryController::FollowTrajectoryController()
  {
    registerCommands();
    setToolLink(0, "LARM_JOINT5");
    setToolLink(1, "RARM_JOINT5");
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
    js_sub_ = node_->subscribe("/joint_states", 1, &FollowTrajectoryController::updateState, this);
    spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner_->start();

    planning_scene_interface_ = boost::shared_ptr<pi::PlanningSceneInterface>(new pi::PlanningSceneInterface);
    rarm_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryController::RARM_GROUP));
    larm_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryController::LARM_GROUP));
    ubody_group_ = boost::shared_ptr<pi::MoveGroupInterface>(new pi::MoveGroupInterface(FollowTrajectoryController::UBODY_GROUP));
  }

  bool FollowTrajectoryController::MoveArmCommand::doMove(boost::shared_ptr<pi::MoveGroupInterface> arm_group,
                                                          geometry_msgs::Pose& target_pose,
                                                          const std::string& arm_group_name)
  {
    const robot_state::JointModelGroup* arm_joint_model_group =
      arm_group->getCurrentState()->getJointModelGroup(arm_group_name);
    arm_group->setPoseTarget(target_pose);

    pi::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group->plan(my_plan) == pi::MoveItErrorCode::SUCCESS);
    printLog("plan for target_pose1: ", success ? "SUCCEEDED" : "FAILED");

    trajectory_msgs::JointTrajectory& jt = my_plan.trajectory_.joint_trajectory;
    std::cout << "Number of joints: " << jt.joint_names.size() << std::endl;
    for (const auto& name : jt.joint_names) { std::cout << name << " "; }
    std::cout << std::endl;

    std::cout << "Number of points: " << jt.points.size() << std::endl;
    int np = 0;
    for (const auto& p : jt.points) {
      std::cout << "Point: " << np++ << "," << "Time: " << p.time_from_start << std::endl;
      for (const auto& x : p.positions) {
        std::cout << x << " ";
      }
      std::cout << std::endl;
    }

    arm_group->move();
    return success;
    // ros::shutdown()
  }

  bool FollowTrajectoryController::MoveArmCommand::operator()(std::vector<CompositeParamType>& params)
  {
    Vector3 xyz(boost::get<VectorX>(params[0]));
    Vector3 rpy_tmp(boost::get<VectorX>(params[1]));
    Vector3 rpy = toRad(rpy_tmp);
    double duration = boost::get<double>(params[2]);
    int armID = boost::get<int>(params[3]);
    printLog("moveArm(", xyz.transpose(), ", ", rpy.transpose(), ", ", duration, ", ", armID, ")");

    try {
      // Transform xyz,rpy to WASIT-based and convert the rotation to quaternion
      BodyPtr body = c_->getRobotBody();
      Link* waistLink = body->link("WAIST");
      Position waistToWristT; // Position == Eigen::Transform
      Position wristT;
      wristT.linear() = rotFromRpy(rpy);
      wristT.translation() = xyz;
      waistToWristT = waistLink->T().inverse() * wristT;
      // std::cout << waistToWristT.linear() << std::endl;
      // std::cout << waistToWristT.translation() << std::endl;
      Eigen::Quaterniond quat(waistToWristT.linear());

      // now send (quat, waistToWristT.translation()) by packing to a ROS message
      geometry_msgs::Pose target_pose1;
      target_pose1.orientation.x = quat.x();
      target_pose1.orientation.y = quat.y();
      target_pose1.orientation.z = quat.z();
      target_pose1.orientation.w = quat.w();
      target_pose1.position.x = waistToWristT.translation().x();
      target_pose1.position.y = waistToWristT.translation().y();
      target_pose1.position.z = waistToWristT.translation().z();
      if (armID == 0) {
        return doMove(c_->larm_group_, target_pose1, FollowTrajectoryController::LARM_GROUP);
      } else {
        return doMove(c_->rarm_group_, target_pose1, FollowTrajectoryController::RARM_GROUP);
      }


#if 0 // move the robot model explicitly in the simulator
      Link* base = body->rootLink();
      Link* wrist = body->link(c_->getToolLinkName(armID));

      JointPathPtr jointPath = getCustomJointPath(body, base, wrist);
      jointPath->calcForwardKinematics();

      c_->ci.clear();
      c_->ci.appendSample(0, wrist->p(), wrist->attitude());
      c_->ci.appendSample(duration, xyz, rotFromRpy(rpy));
      c_->ci.update();
      return c_->executeCartesianMotion(wrist, jointPath);
#endif
    } catch (...) {
      printLog("unknown armID: ", armID);
      return false;
    }
  }

  bool FollowTrajectoryController::MoveGripperCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double width = boost::get<double>(params[0]);
    double duration = boost::get<double>(params[1]);
    int gripperID = boost::get<int>(params[2]);
    printLog("moveGripper(", width, ", ", duration, ", ", gripperID, ")");

    return true;
  }

  bool FollowTrajectoryController::GoInitialCommand::operator()(std::vector<CompositeParamType>& params)
  {
    double duration = boost::get<double>(params[0]);

    BodyPtr body = c_->getRobotBody();
    VectorXd qCur = c_->getCurrentJointAngles(body);

    moveit::core::RobotStatePtr current_state = c_->ubody_group_->getCurrentState();
    const robot_state::JointModelGroup* joint_model_group
      = c_->ubody_group_->getCurrentState()->getJointModelGroup(FollowTrajectoryController::UBODY_GROUP);
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // for (const auto& n : joint_model_group->getVariableNames()) { printLog(n, " "); }
    // CHEST_JOINT0, HEAD_JOINT0 - 1, LARM_JOINT0 - 5, RARM_JOINT0 - 5
    // In choreonoid, the order of LARM_JOINTs and RARM_joints is opposite.
    pi::MoveGroupInterface::Plan my_plan;
    bool success = false;
    const Listing& pose = *body->info()->findListing("standardPose");
    if(pose.isValid()){
      const int nn = std::min(pose.size(), (int)joint_group_positions.size());
      for (int i = 0; i < 3; i++){
        joint_group_positions[i] = radian(pose[i].toDouble());
      }
      for (int i = 3; i < 9; i++){
        joint_group_positions[i+6] = radian(pose[i].toDouble());
      }
      for (int i = 9; i < 15; i++){
        joint_group_positions[i-6] = radian(pose[i].toDouble());
      }
      c_->ubody_group_->setJointValueTarget(joint_group_positions);
      success = c_->ubody_group_->plan(my_plan) == pi::MoveItErrorCode::SUCCESS;
      c_->ubody_group_->move();
    }

    return success;
  }

  void FollowTrajectoryController::updateState(const sensor_msgs::JointState::ConstPtr& jointstate)
  {
    try {
      // avoid accessing robot item before loading
      BodyItem* robotItem = getRobotItem();
      BodyPtr body = getRobotBody();
      // for (int i = 0; i < body->numJoints(); i++) {

      // The order of joints is the same as Choreonoid in /joint_states
      for (int i = 0; i < 15; i++) {
        body->joint(i)->q() = jointstate->position[i];
      }

      updateAttachedModels();
      robotItem->notifyKinematicStateChange(true);
      QCoreApplication::processEvents();

    } catch (...) {
      
    }
  }

  bool FollowTrajectoryController::sendTrajectory()
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

    traj_pub_.publish(traj);

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
    traj_pub_.publish(traj);
#endif

    // wait the completion of the execution
    // update the robot status in Scene View on the completion
    // subscribe to /joint_state and update the local model
  }

  void FollowTrajectoryController::registerCommands()
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
