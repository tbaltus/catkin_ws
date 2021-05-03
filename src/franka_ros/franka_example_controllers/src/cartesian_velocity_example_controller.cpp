// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "sensor_msgs/Joy.h"
#include "ros/service_client.h"

namespace franka_example_controllers {

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;

  falcon_subscription_ = node_handle.subscribe(
      "/falcon/joystick", 20, &CartesianVelocityExampleController::falconCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  ros::ServiceClient set_high_force_threshold_srv_ = node_handle.serviceClient<franka_msgs::SetForceTorqueCollisionBehavior>("/franka_control/set_force_torque_collision_behavior");

  var_force_threshold_.request.upper_torque_thresholds_nominal = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
  var_force_threshold_.request.upper_force_thresholds_nominal = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
  var_force_threshold_.request.lower_torque_thresholds_nominal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  var_force_threshold_.request.lower_force_thresholds_nominal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  set_high_force_threshold_srv_.call(var_force_threshold_);



  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianVelocityExampleController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  /*double time_max = 4.0;
  double v_max = 0.05;
  double angle = M_PI / 4.0;
  double cycle = std::floor(
      pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  double v_x = std::cos(angle) * v;
  double v_z = -std::sin(angle) * v;*/
  
  double max_speed = 0.035;
  double speed_x_to_reach = (max_speed-(-max_speed))/(0.17-0.07)*zpos + (-max_speed) - (max_speed-(-max_speed))/(0.17-0.07)*0.07;
  double speed_y_to_reach = (max_speed-(-max_speed))/(0.06-(-0.06))*xpos + (-max_speed) - (max_speed-(-max_speed))/(0.06-(-0.06))*(-0.06);
  double speed_z_to_reach = (max_speed-(-max_speed))/(0.06-(-0.06))*ypos + (-max_speed) - (max_speed-(-max_speed))/(0.06-(-0.06))*(-0.06);

  v_x += (speed_x_to_reach-v_x)*0.01;
  v_y += (speed_y_to_reach-v_y)*0.01;
  v_z += (speed_z_to_reach-v_z)*0.01;

  //ROS_INFO_STREAM(std::fixed << "acceleration = " << (speed_x_to_reach-v_x)*0.001/period.toSec());
  //ROS_INFO_STREAM(std::fixed << "speed = " << speed_y_to_reach);

  std::array<double, 6> command = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};//{{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityExampleController::falconCallback(const sensor_msgs::Joy& msg) {
  
  xpos = msg.axes[0];
  ypos = msg.axes[1];
  zpos = msg.axes[2];
  //ROS_INFO_STREAM(std::fixed << "position = " << xpos << "," << ypos << "," << zpos);

}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
