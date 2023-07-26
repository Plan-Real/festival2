#ifndef FESTIVAL_UR_CORE_NODE__FESTIVAL_HPP_
#define FESTIVAL_UR_CORE_NODE__FESTIVAL_HPP_

#include <memory>
#include <string>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/QR>

#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "festival_ur_interfaces/msg/purpose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/convert.h>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_move_group_node");

enum Status
{
  start,
  stop,
};


class FestivalNode : public rclcpp::Node {
public:
  explicit FestivalNode();

  //end of constructor
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void startPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void finishPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void initialParamSetup();


  void timercameraTf();

  std::vector<double> quaternionToEuler(double qx, double qy, double qz, double qw);

  double errorNorm(const std::vector<double> target, const std::vector<double> current, Eigen::VectorXd& error);
  void initialJointSetup();

  void secondJointSetup();

  void jointPrint(const std::vector<double> purpose_joint);

  void jointupdate();

  bool check_arrival(const std::vector<double> current, 
                     const std::vector<double> purpose, 
                     const double threshold);

  void timer_callback();


private:
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_currentpose,
                      joint_state_vel,
                      joint_group_velocities_arm,
                      joint_targetpose,
                      purpose_joint,
                      camera_targetpose,
                      camera_currentpose;

  double camera_x, camera_z, goal_x, goal_y, goal_z,
         q2, q3, q4,period;

  Eigen::VectorXd error;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobian;
  // Eigen::Matrix<double, 6,6> pinv;
  Eigen::Matrix<double, 3, 1> pinv_error;
  Status robot_status;
  
  // ros subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_stae_subscriber;

  // ros publisher
  rclcpp::Publisher<festival_ur_interfaces::msg::Purpose>::SharedPtr joint_pose_publisher;
  festival_ur_interfaces::msg::Purpose purpose_msgs;

  // ros timer
  rclcpp::TimerBase::SharedPtr timer_;

  //tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  //service server
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start,
                                                     service_finish;
  //service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr first_client,
                                                    second_client;
};

#endif