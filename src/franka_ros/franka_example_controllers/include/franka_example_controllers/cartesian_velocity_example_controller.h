// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "sensor_msgs/Joy.h"
#include "ros/service_client.h"
#include "franka_msgs/SetForceTorqueCollisionBehavior.h"

namespace franka_example_controllers {

class CartesianVelocityExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;


  ros::Subscriber falcon_subscription_;
  void falconCallback(const sensor_msgs::Joy& msg);
  double xpos=0.0;
  double ypos=0.0;
  double zpos=0.0;
  double v_x = 0.0;
  double v_y = 0.0;
  double v_z = 0.0;
  franka_msgs::SetForceTorqueCollisionBehavior var_force_threshold_;

};

}  // namespace franka_example_controllers
