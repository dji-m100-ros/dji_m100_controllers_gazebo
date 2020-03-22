//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <dji_m100_controllers_gazebo/dji_m100_ros_sitl_controller_gazebo.h>

#include <geometry_msgs/WrenchStamped.h>

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace dji_m100_controller_gazebo_sitl {
// For now only wrap the existing Default RobotHwSim
DJIM100HardwareSim::DJIM100HardwareSim() : DefaultRobotHWSim() {
  this->registerInterface(&body_interface_);
  body_interface_.registerAccel(&acceleration_);
  body_interface_.registerPose(&pose_);
  body_interface_.registerMotorStatus(&motor_status_);
  body_interface_.registerSensorImu(&imu_);
  body_interface_.registerTwist(&twist_);

  wrench_output_ = body_interface_.addInput<WrenchCommandHandle>("wrench");
  motor_output_ = body_interface_.addInput<MotorCommandHandle>("motor");
}

DJIM100HardwareSim::~DJIM100HardwareSim() {}

bool DJIM100HardwareSim::initSim(
    const std::string &robot_namespace, ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool default_sim_ok = DefaultRobotHWSim::initSim(
      robot_namespace, model_nh, parent_model, urdf_model, transmissions);

  this->model_ = parent_model;
  this->base_link_ = model_->GetLink();
  this->battery_handle = base_link_->Battery("linear_battery");
  this->initial_battery_level = this->battery_handle->Voltage();
#if (GAZEBO_MAJOR_VERSION >= 8)
  physics_ = model_->GetWorld()->Physics();
#else
  physics_ = model_->GetWorld()->GetPhysicsEngine();
#endif

  model_nh.param<std::string>("world_frame", world_frame_, "world");
  model_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  this->motor_status_.on = true;
  this->motor_status_.header.frame_id = base_link_frame_;

  this->enable_motors_server_ = model_nh.advertiseService(
      "enable_motors", &DJIM100HardwareSim::enableMotorsCallback, this);

  this->wrench_limiter_.init(model_nh, "wrench_limits");

  this->wrench_command_publisher_ =
      model_nh.advertise<geometry_msgs::WrenchStamped>("command/wrench", 1);
  this->motor_command_publisher_ =
      model_nh.advertise<geometry_msgs::WrenchStamped>("command/motor", 1);
  this->battery_publisher_ =
      model_nh.advertise<std_msgs::Float64>("battery", 1);

  auto plugin = this->model_->GetSDF()->GetElement("plugin");
  while (plugin->GetAttribute("name")->GetAsString() != "sitl_controller") {
    plugin = plugin->GetNextElement();
  }
  if (plugin->HasElement("allow_contact_sensing")) {
    this->contact_allowed =
        plugin->GetElement("allow_contact_sensing")->Get<bool>();
  }
  if (this->contact_allowed) {
    this->contact_subscriber_ = nh.subscribe<gazebo_msgs::ContactsState>(
        this->model_->GetName() + "/contact", 1,
        boost::bind(&DJIM100HardwareSim::contactCallback, this, _1));
  }
  return default_sim_ok;
}
void DJIM100HardwareSim::contactCallback(
    const gazebo_msgs::ContactsState::ConstPtr &contact_msg) {
  if (!motor_status_.running) {
    return;
  }
  if (pose_.position.z > 0.3) {
    took_off = true;
  }

  if (took_off) {
    int num_of_contacts = contact_msg->states.size();
    if (num_of_contacts) // Pause the simulation
    {

      this->contact = true;
      std::cout << "\033[1;31m Contact information :"
                << contact_msg->states[0].info << "\033[0m\n";
    }
  }
}
void DJIM100HardwareSim::readSim(ros::Time time, ros::Duration period) {
  DefaultRobotHWSim::readSim(time, period);
  // read state from Gazebo
  const double acceleration_time_constant = 0.1;
#if (GAZEBO_MAJOR_VERSION >= 8)
  gz_acceleration_ = ((base_link_->WorldLinearVel() - gz_velocity_) +
                      acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((base_link_->WorldLinearVel() - gz_angular_velocity_) +
       acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = base_link_->WorldPose();
  gz_velocity_ = base_link_->WorldLinearVel();
  gz_angular_velocity_ = base_link_->WorldAngularVel();
#else
  gz_acceleration_ = ((base_link_->GetWorldLinearVel() - gz_velocity_) +
                      acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((base_link_->GetWorldLinearVel() - gz_angular_velocity_) +
       acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = base_link_->GetWorldPose();
  gz_velocity_ = base_link_->GetWorldLinearVel();
  gz_angular_velocity_ = base_link_->GetWorldAngularVel();
#endif

#if (GAZEBO_MAJOR_VERSION >= 8)
  pose_.position.x = gz_pose_.Pos().X();
  pose_.position.y = gz_pose_.Pos().Y();
  pose_.position.z = gz_pose_.Pos().Z();
  pose_.orientation.w = gz_pose_.Rot().W();
  pose_.orientation.x = gz_pose_.Rot().X();
  pose_.orientation.y = gz_pose_.Rot().Y();
  pose_.orientation.z = gz_pose_.Rot().Z();
  twist_.linear.x = gz_velocity_.X();
  twist_.linear.y = gz_velocity_.Y();
  twist_.linear.z = gz_velocity_.Z();
  twist_.angular.x = gz_angular_velocity_.X();
  twist_.angular.y = gz_angular_velocity_.Y();
  twist_.angular.z = gz_angular_velocity_.Z();
  acceleration_.linear.x = gz_acceleration_.X();
  acceleration_.linear.y = gz_acceleration_.Y();
  acceleration_.linear.z = gz_acceleration_.Z();
  acceleration_.angular.x = gz_angular_acceleration_.X();
  acceleration_.angular.y = gz_angular_acceleration_.Y();
  acceleration_.angular.z = gz_angular_acceleration_.Z();
#else
  pose_.position.x = gz_pose_.pos.x;
  pose_.position.y = gz_pose_.pos.y;
  pose_.position.z = gz_pose_.pos.z;
  pose_.orientation.w = gz_pose_.rot.w;
  pose_.orientation.x = gz_pose_.rot.x;
  pose_.orientation.y = gz_pose_.rot.y;
  pose_.orientation.z = gz_pose_.rot.z;
  twist_.linear.x = gz_velocity_.x;
  twist_.linear.y = gz_velocity_.y;
  twist_.linear.z = gz_velocity_.z;
  twist_.angular.x = gz_angular_velocity_.x;
  twist_.angular.y = gz_angular_velocity_.y;
  twist_.angular.z = gz_angular_velocity_.z;
  acceleration_.linear.x = gz_acceleration_.x;
  acceleration_.linear.y = gz_acceleration_.y;
  acceleration_.linear.z = gz_acceleration_.z;
  acceleration_.angular.x = gz_angular_acceleration_.x;
  acceleration_.angular.y = gz_angular_acceleration_.y;
  acceleration_.angular.z = gz_angular_acceleration_.z;
#endif

#if (GAZEBO_MAJOR_VERSION >= 8)
  imu_.orientation.w = gz_pose_.Rot().W();
  imu_.orientation.x = gz_pose_.Rot().X();
  imu_.orientation.y = gz_pose_.Rot().Y();
  imu_.orientation.z = gz_pose_.Rot().Z();

  ignition::math::Vector3d gz_angular_velocity_body =
      gz_pose_.Rot().RotateVectorReverse(gz_angular_velocity_);
  imu_.angular_velocity.x = gz_angular_velocity_body.X();
  imu_.angular_velocity.y = gz_angular_velocity_body.Y();
  imu_.angular_velocity.z = gz_angular_velocity_body.Z();

  ignition::math::Vector3d gz_linear_acceleration_body =
      gz_pose_.Rot().RotateVectorReverse(gz_acceleration_ -
                                         model_->GetWorld()->Gravity());
  imu_.linear_acceleration.x = gz_linear_acceleration_body.X();
  imu_.linear_acceleration.y = gz_linear_acceleration_body.Y();
  imu_.linear_acceleration.z = gz_linear_acceleration_body.Z();
#else
  imu_.orientation.w = gz_pose_.rot.w;
  imu_.orientation.x = gz_pose_.rot.x;
  imu_.orientation.y = gz_pose_.rot.y;
  imu_.orientation.z = gz_pose_.rot.z;

  gazebo::math::Vector3 gz_angular_velocity_body =
      gz_pose_.rot.RotateVectorReverse(gz_angular_velocity_);
  imu_.angular_velocity.x = gz_angular_velocity_body.x;
  imu_.angular_velocity.y = gz_angular_velocity_body.y;
  imu_.angular_velocity.z = gz_angular_velocity_body.z;

  gazebo::math::Vector3 gz_linear_acceleration_body =
      gz_pose_.rot.RotateVectorReverse(gz_acceleration_ -
                                       physics_->GetGravity());
  imu_.linear_acceleration.x = gz_linear_acceleration_body.x;
  imu_.linear_acceleration.y = gz_linear_acceleration_body.y;
  imu_.linear_acceleration.z = gz_linear_acceleration_body.z;
#endif
}

void DJIM100HardwareSim::writeSim(ros::Time time, ros::Duration period) {
  // Early check for contact
  if (this->contact) {
    this->model_->GetWorld()->SetPaused(true);
    return;
  }
  DefaultRobotHWSim::writeSim(time, period);
  bool result_written = false;

  if (motor_output_->connected() && motor_output_->enabled()) {
    motor_command_publisher_.publish(motor_output_->getCommand());
    result_written = true;
  }

  if (wrench_output_->connected() && wrench_output_->enabled()) {
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = time;
    wrench.header.frame_id = base_link_frame_;

    if (motor_status_.on && motor_status_.running) {
      wrench.wrench = wrench_limiter_(wrench_output_->getCommand());

      if (!result_written) {

#if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Vector3d force(wrench.wrench.force.x,
                                       wrench.wrench.force.y,
                                       wrench.wrench.force.z);
        ignition::math::Vector3d torque(wrench.wrench.torque.x,
                                        wrench.wrench.torque.y,
                                        wrench.wrench.torque.z);
#else
        gazebo::math::Vector3 force(wrench.wrench.force.x,
                                    wrench.wrench.force.y,
                                    wrench.wrench.force.z);
        gazebo::math::Vector3 torque(wrench.wrench.torque.x,
                                     wrench.wrench.torque.y,
                                     wrench.wrench.torque.z);
#endif
        base_link_->AddRelativeForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
        base_link_->AddRelativeTorque(
            torque - base_link_->GetInertial()->CoG().Cross(force));
#else
        base_link_->AddRelativeTorque(
            torque - base_link_->GetInertial()->GetCoG().Cross(force));
#endif
      }

    } else {
      wrench.wrench = geometry_msgs::Wrench();
    }

    wrench_command_publisher_.publish(wrench);
  }
  std_msgs::Float64 battery_level;
  battery_level.data =
      (this->battery_handle->Voltage() / (double)this->initial_battery_level) *
      100.0;
  this->battery_publisher_.publish(battery_level);
}

bool DJIM100HardwareSim::enableMotorsCallback(
    hector_uav_msgs::EnableMotors::Request &req,
    hector_uav_msgs::EnableMotors::Response &res) {
  res.success = enableMotors(req.enable);
  return true;
}

bool DJIM100HardwareSim::enableMotors(bool enable) {
  motor_status_.running = enable;
  return true;
}

} // namespace dji_m100_controller_gazebo_sitl

#include <pluginlib/class_list_macros.h>
//#define PLUGINLIB_EXPORT_AMBIGUOUS_CLASS

PLUGINLIB_EXPORT_CLASS(dji_m100_controller_gazebo_sitl::DJIM100HardwareSim,
                       gazebo_ros_control::RobotHWSim)
