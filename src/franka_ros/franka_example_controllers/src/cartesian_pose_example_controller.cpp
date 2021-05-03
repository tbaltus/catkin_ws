// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/WrenchStamped.h"

namespace franka_example_controllers {

double x_rot = 0;
double x_rot_v = 0;
double x_rot_acc = 0;
double x_rot_franka_desired = 0;
double x_rot_err = 0;
double x_rot_prev_err = 0;

double x_min_wrist = 0.240;
double x_max_wrist = 0.749;

double y_rot = 0;
double y_rot_v = 0;
double y_rot_acc = 0;
double y_rot_franka_desired = 0;
double y_rot_err = 0;
double y_rot_prev_err = 0;

double y_min_wrist = 0.644;
double y_max_wrist = 0.384;


double z_rot = 0;
double z_rot_v = 0;
double z_rot_acc = 0;
double z_rot_franka_desired = 0;
double z_rot_err = 0;
double z_rot_prev_err = 0;

double z_min_wrist = 0.515;
double z_max_wrist = 0;

double wrist_x= 0;
double wrist_y= 0;
double wrist_z= 0;

double x_rot_min_franka = - M_PI/6.0;
double x_rot_max_franka = M_PI/6.0;

double y_rot_min_franka = -M_PI / 6.0;
double y_rot_max_franka = M_PI / 6.0;

double z_rot_min_franka = -M_PI / 6.0;
double z_rot_max_franka = M_PI / 6.0;

double x_v = 0;
double x_acc = 0;

double y_v = 0;
double y_acc = 0;

double z_v = 0;
double z_acc = 0;

double x_franka_desired = 0;
double y_franka_desired = 0;
double z_franka_desired = 0;

double x_err = 0;

double y_err = 0;

double z_err = 0;

double x_min_omega = -0.0525;
double x_max_omega = 0.073;

double y_min_omega = -0.111;
double y_max_omega = 0.111;

double z_min_omega = -0.0756;
double z_max_omega = 0.121;

double x_homing_franka = 0;
double x_min_franka = 0;
double x_max_franka = 0;

double y_homing_franka = 0;
double y_min_franka = 0;
double y_max_franka = 0;

double z_homing_franka = 0;
double z_min_franka = 0;
double z_max_franka = 0;

double x_omega_at_press = 0;
double x_omega_at_release = 0;

double y_omega_at_press = 0;
double y_omega_at_release = 0;

double z_omega_at_press = 0;
double z_omega_at_release = 0;

bool click_2_shift = false;

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  omega_subscription_ = node_handle.subscribe(
      "/EE_cartesian_position", 20, &CartesianPoseExampleController::omegaCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  phidget_subscription_ = node_handle.subscribe(
    "/EE_cartesian_orientation", 20, &CartesianPoseExampleController::phidgetCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  click_subscription_ = node_handle.subscribe(
      "/button_state", 20, &CartesianPoseExampleController::clickCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  //High force and torque value
  ros::ServiceClient set_high_force_threshold_srv_ = node_handle.serviceClient<franka_msgs::SetForceTorqueCollisionBehavior>("/franka_control/set_force_torque_collision_behavior");

  var_force_threshold_.request.upper_torque_thresholds_nominal = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
  var_force_threshold_.request.upper_force_thresholds_nominal = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
  var_force_threshold_.request.lower_torque_thresholds_nominal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  var_force_threshold_.request.lower_force_thresholds_nominal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  set_high_force_threshold_srv_.call(var_force_threshold_);

  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  ROS_INFO("Homing Pose : x = %f, y = %f,z = %f", initial_pose_[12], initial_pose_[13] ,initial_pose_[14]); //added

  ROS_INFO("%f, %f, %f, %f", initial_pose_[0], initial_pose_[4] ,initial_pose_[8], initial_pose_[12]); //added
  ROS_INFO("%f, %f, %f, %f", initial_pose_[1], initial_pose_[5] ,initial_pose_[9], initial_pose_[13]); //added
  ROS_INFO("%f, %f, %f, %f", initial_pose_[2], initial_pose_[6] ,initial_pose_[10], initial_pose_[14]); //added
  ROS_INFO("%f, %f, %f, %f", initial_pose_[3], initial_pose_[7] ,initial_pose_[11], initial_pose_[15]); //added

  x_homing_franka = initial_pose_[12];
  y_homing_franka = initial_pose_[13];
  z_homing_franka = initial_pose_[14];

  x_min_franka = x_homing_franka - 0.1;
  x_max_franka = x_homing_franka + 0.1;

  y_min_franka = y_homing_franka - 0.1;
  y_max_franka = y_homing_franka + 0.1;

  z_min_franka = z_homing_franka - 0.1;
  z_max_franka = z_homing_franka + 0.1;

  wrist_x = (x_min_wrist + x_max_wrist)/2;
  wrist_y = (y_min_wrist + y_max_wrist)/2;
  wrist_z = (z_min_wrist + z_max_wrist)/2;

  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  
  double dt = 0.001;

  std::array<double, 16> new_pose = initial_pose_;
  
  new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  std::array<double, 16> buffer_pose = new_pose;

  //Cartesian rotational movement
  //Compute current angles
  x_rot = std::atan2(new_pose[6], new_pose[10]);
  y_rot = std::atan2(-new_pose[2], std::sqrt(std::pow(new_pose[6],2) + std::pow(new_pose[10], 2)));
  z_rot = std::atan2(new_pose[1], new_pose[0]);
  //ROS_INFO("%f", z_rot);
  //ROS_INFO("%f", wrist_z);

  x_rot_franka_desired = (x_rot_max_franka - x_rot_min_franka)/(x_max_wrist - x_min_wrist) * wrist_x + x_rot_min_franka - (x_rot_max_franka - x_rot_min_franka)/(x_max_wrist - x_min_wrist) * x_min_wrist;
  y_rot_franka_desired = (y_rot_max_franka - y_rot_min_franka)/(y_max_wrist - y_min_wrist) * wrist_y + y_rot_min_franka - (y_rot_max_franka - y_rot_min_franka)/(y_max_wrist - y_min_wrist) * y_min_wrist;
  z_rot_franka_desired = (z_rot_max_franka - z_rot_min_franka)/(z_max_wrist - z_min_wrist) * wrist_z + z_rot_min_franka - (z_rot_max_franka - z_rot_min_franka)/(z_max_wrist - z_min_wrist) * z_min_wrist;

  //Due to pi redundance angle on the x axis
  if(x_rot > 0)
    x_rot = M_PI - x_rot;
  else
    x_rot = - M_PI - x_rot;

    //Testing
  //x_rot_franka_desired = x_rot;
  //y_rot_franka_desired = y_rot;
  //z_rot_franka_desired = z_rot;

  x_rot_err = -(x_rot_franka_desired - x_rot);
  y_rot_err = y_rot_franka_desired - y_rot;
  z_rot_err = z_rot_franka_desired - z_rot;

  double P_rot = 15;
  double D_rot = 8;

  double x_rot_proportional = x_rot_err;
  double x_rot_derivative = - x_rot_v;

  double y_rot_proportional = y_rot_err;
  double y_rot_derivative = - y_rot_v;

  double z_rot_proportional = z_rot_err;
  double z_rot_derivative = - z_rot_v;

  x_rot_acc = x_rot_proportional * P_rot + x_rot_derivative * D_rot;
  x_rot_v += x_rot_acc * dt;

  y_rot_acc = y_rot_proportional * P_rot + y_rot_derivative * D_rot;
  y_rot_v += y_rot_acc * dt;

  z_rot_acc = z_rot_proportional * P_rot + z_rot_derivative * D_rot;
  z_rot_v += z_rot_acc * dt;

  //Compute new pose orientation
  new_pose[0] = buffer_pose[0] - z_rot_v * dt * buffer_pose[1] + y_rot_v * dt * buffer_pose[2];
  new_pose[1] = z_rot_v * dt * buffer_pose[0] + buffer_pose[1] - x_rot_v * dt * buffer_pose[2];
  new_pose[2] = - y_rot_v * dt * buffer_pose[0] + x_rot_v * dt * buffer_pose[1] + buffer_pose[2];

  new_pose[4] = buffer_pose[4] - z_rot_v * dt * buffer_pose[5] + y_rot_v * dt * buffer_pose[6];
  new_pose[5] = z_rot_v * dt * buffer_pose[4] + buffer_pose[5] - x_rot_v * dt * buffer_pose[6];
  new_pose[6] = - y_rot_v * dt * buffer_pose[4] + x_rot_v * dt * buffer_pose[5] + buffer_pose[6];

  new_pose[8] = buffer_pose[8] - z_rot_v * dt * buffer_pose[9] + y_rot_v * dt * buffer_pose[10];
  new_pose[9] = z_rot_v * dt * buffer_pose[8] + buffer_pose[9] - x_rot_v * dt * buffer_pose[10];
  new_pose[10] = - y_rot_v * dt * buffer_pose[8] + x_rot_v * dt * buffer_pose[9] + buffer_pose[10];

  //Cartesian linear motion
  if(!click_2_shift)
  {
    x_franka_desired = (x_max_franka - x_min_franka)/(x_max_omega - x_min_omega) * omega_x + x_min_franka - (x_max_franka - x_min_franka)/(x_max_omega - x_min_omega) * x_min_omega;
    y_franka_desired = (y_max_franka - y_min_franka)/(y_max_omega - y_min_omega) * omega_y + y_min_franka - (y_max_franka - y_min_franka)/(y_max_omega - y_min_omega) * y_min_omega;
    z_franka_desired = (z_max_franka - z_min_franka)/(z_max_omega - z_min_omega) * omega_z + z_min_franka - (z_max_franka - z_min_franka)/(z_max_omega - z_min_omega) * z_min_omega;
  }

  //Avoid having to move_to_start all the time to test rotational movement
  /*x_franka_desired = new_pose[12];
  y_franka_desired = new_pose[13];
  z_franka_desired = new_pose[14];*/

  x_err = x_franka_desired - new_pose[12];
  y_err = y_franka_desired - new_pose[13];
  z_err = z_franka_desired - new_pose[14];

  double P_pos = 35;
  double D_pos = 8;
  
  double x_proportional = x_err;
  double x_derivative = - x_v;
  
  double y_proportional = y_err;
  double y_derivative = - y_v;

  double z_proportional = z_err;
  double z_derivative = - z_v;

  //Here is the key
  x_acc = x_proportional * P_pos + x_derivative * D_pos;
  new_pose[12] += x_v * 0.001;
  x_v += x_acc * 0.001;

  y_acc = y_proportional * P_pos + y_derivative * D_pos;
  new_pose[13] += y_v * 0.001;
  y_v += y_acc * 0.001;

  z_acc = z_proportional * P_pos + z_derivative * D_pos;
  new_pose[14] += z_v * 0.001;
  z_v += z_acc * 0.001;

  cartesian_pose_handle_->setCommand(new_pose);

  ros::spinOnce();
}

void CartesianPoseExampleController::omegaCallback(const geometry_msgs::Vector3Stamped& msg) {
  
  omega_x = msg.vector.x;
  omega_y = msg.vector.y;
  omega_z = msg.vector.z;
  //ROS_INFO_STREAM(std::fixed << "position x = " << omega_x << " position y = " << omega_y << " position z = " << omega_z);
}

void CartesianPoseExampleController::phidgetCallback(const geometry_msgs::Vector3Stamped& msg) {
  
  wrist_x = msg.vector.x;
  wrist_y = msg.vector.y;
  wrist_z = msg.vector.z;

  //ROS_INFO_STREAM(std::fixed << "position x = " << wrist_x << " position y = " << wrist_y << " position z = " << wrist_z);
}

void CartesianPoseExampleController::clickCallback(const std_msgs::String::ConstPtr& msg){

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  std::string click_string_pressed = "Pressed";
  std::string click_string_released = "Released";

  std::string click_string_received = msg->data.c_str();

  if (click_string_received.compare(click_string_pressed) == 0) {
    click_2_shift = true;
    x_omega_at_press = omega_x;
    y_omega_at_press = omega_y;
    z_omega_at_press = omega_z;
  }

  if (click_string_received.compare(click_string_released) == 0) {
    click_2_shift = false;
    x_omega_at_release = omega_x;
    y_omega_at_release = omega_y;
    z_omega_at_release = omega_z;

    double x_shift = (x_max_franka - x_min_franka)/(x_max_omega - x_min_omega)*(x_omega_at_press - x_omega_at_release);
    double y_shift = (y_max_franka - y_min_franka)/(y_max_omega - y_min_omega)*(y_omega_at_press - y_omega_at_release);
    double z_shift = (z_max_franka - z_min_franka)/(z_max_omega - z_min_omega)*(z_omega_at_press - z_omega_at_release);

    x_min_franka = x_min_franka + x_shift;
    x_max_franka = x_max_franka + x_shift;

    y_min_franka = y_min_franka + y_shift;
    y_max_franka = y_max_franka + y_shift;;

    z_min_franka = z_min_franka + z_shift;
    z_max_franka = z_max_franka + z_shift;;
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)