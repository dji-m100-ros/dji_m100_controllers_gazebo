#ifndef DJI_M100_CONTROLLER_HARDWARE_GAZEBO_H
#define DJI_M100_CONTROLLER_HARDWARE_GAZEBO_H

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>

#include <gazebo_msgs/ContactsState.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <gazebo/common/Battery.hh>
namespace dji_m100_controller_gazebo_sitl {

using namespace hector_quadrotor_interface;

class DJIM100HardwareSim : public gazebo_ros_control::DefaultRobotHWSim {
public:
  DJIM100HardwareSim();
  virtual ~DJIM100HardwareSim();

  virtual bool
  initSim(const std::string &robot_namespace, ros::NodeHandle model_nh,
          gazebo::physics::ModelPtr parent_model,
          const urdf::Model *const urdf_model,
          std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  bool enableMotorsCallback(hector_uav_msgs::EnableMotors::Request &req,
                            hector_uav_msgs::EnableMotors::Response &res);

private:
  bool enableMotors(bool enable);
  void contactCallback(const gazebo_msgs::ContactsState::ConstPtr &contact_msg);
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Accel acceleration_;
  sensor_msgs::Imu imu_;
  hector_uav_msgs::MotorStatus motor_status_;

  QuadrotorInterface body_interface_;

  WrenchCommandHandlePtr wrench_output_;
  MotorCommandHandlePtr motor_output_;

  hector_quadrotor_interface::WrenchLimiter wrench_limiter_;
  std::string base_link_frame_, world_frame_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_link_;
  gazebo::physics::PhysicsEnginePtr physics_;
  gazebo::common::BatteryPtr battery_handle;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d gz_pose_;
  ignition::math::Vector3d gz_velocity_, gz_acceleration_, gz_angular_velocity_,
      gz_angular_acceleration_;
#else
  gazebo::math::Pose gz_pose_;
  gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_,
      gz_angular_acceleration_;
#endif
  /* Only for debugging purposes. These echo the applied magnitudes and command.
   */
  ros::Publisher wrench_command_publisher_;
  ros::Publisher motor_command_publisher_;

  double initial_battery_level;
  ros::Publisher battery_publisher_;
  ros::ServiceServer enable_motors_server_;
  ros::Subscriber contact_subscriber_;

  bool contact_allowed = false;
  bool contact = false;
  bool took_off = false;
  ros::CallbackQueue callback_queue_;
  ros::NodeHandle nh;
};

} // namespace dji_m100_controller_gazebo_sitl

#endif // DJI_M100_CONTROLLER_HARDWARE_GAZEBO_H
