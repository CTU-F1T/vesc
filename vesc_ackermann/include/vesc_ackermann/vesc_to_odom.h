// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_VESC_TO_ODOM_H_
#define VESC_ACKERMANN_VESC_TO_ODOM_H_

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <teensy/drive_values.h>
#include <teensy/pwm_high.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>

namespace vesc_ackermann
{

class VescToOdom
{
public:

  VescToOdom(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  bool estop_state_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  // odometry state
  double x_, y_, yaw_;
  bool has_steering_cmd_; ///< Last servo position commanded value
  vesc_msgs::VescStateStamped::ConstPtr last_state_; ///< Last received state message
  int steering_angle_;

  // ROS services
  ros::Publisher odom_pub_;
  ros::Subscriber vesc_state_sub_;
  ros::Subscriber servo_sub_;
  ros::Subscriber estop_sub_;
  ros::Subscriber pwm_high_sub;
  boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state);
  void servoCmdCallback(const teensy::drive_values::ConstPtr& servo);
  void eStopCallback(const std_msgs::Bool::ConstPtr& estop);
  void pwmHighCallback(const teensy::pwm_high::ConstPtr& pwm_high);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_VESC_TO_ODOM_H_
