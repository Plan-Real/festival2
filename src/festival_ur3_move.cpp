#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_move_group_node");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

class MoveItPlanner : public rclcpp::Node {
public:
  MoveItPlanner(std::shared_ptr<rclcpp::Node> move_group_node_) 
    : Node("test_node"),
      move_group_arm(move_group_node_, PLANNING_GROUP_ARM)
      // joint_model_group_arm(
      //     move_group_arm.getCurrentState()->getJointModelGroup(
      //         PLANNING_GROUP_ARM)) 
  {
    RCLCPP_INFO(LOGGER, "start");
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&MoveItPlanner::timer_callback, this));
    RCLCPP_INFO(LOGGER, "start2");
  }    
  //end of constructor

  void get_info(){

  }

  // void current_state(){
  //   RCLCPP_INFO(LOGGER, "get robot current state");
  //   current_state_arm= move_group_arm.getCurrentState(10); 
  //   //최대 10초동안 상태를 받아오기 위해 대기

  //   current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
  //                                              this->joint_group_positions_arm);
  //   //position받아오기

  //   current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
  //                                              this->joint_group_velocities_arm);
  //   //velocity받아오기
  // }

  void plan_arm_joint_space()
  {

  RCLCPP_INFO(LOGGER, "planning to joint space");
  joint_group_positions_arm[0]=3.14;
  joint_group_positions_arm[1]=-1.57;
  joint_group_positions_arm[2]=0.0;
  joint_group_positions_arm[3]=-1.57;
  joint_group_positions_arm[4]=-1.57;
  joint_group_positions_arm[5]=0;
  std::vector<double> joint_group_positions_arm2(6, 0.0);
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose);



  
  // RCLCPP_INFO(LOGGER, "%d",success );
  RCLCPP_INFO(LOGGER, "planning to joint space2");
  // bool success_arm = (move_group_arm.plan(my_paln_arm)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (1)
  {
    move_group_arm.move();

  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan to joint space");
  }


  }

  void timer_callback(){
    this->timer_->cancel();
    // current_state();
    plan_arm_joint_space();
  }


private:
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm,
                      joint_group_velocities_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_paln_arm;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;

  // const moveit::core::JointModelGroup *joint_model_group_arm;

  // moveit::core::RobotStatePtr current_state_arm;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_option;
  node_option.automatically_declare_parameters_from_overrides(true);
  RCLCPP_INFO(LOGGER, "get robot current state");
  std::cout << "fucking2" << std::endl;
  auto move_group_node = rclcpp::Node::make_shared("ur_move_group_node", node_option);
  std::cout << "fucking3" << std::endl;
  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<MoveItPlanner> planner_node = std::make_shared<MoveItPlanner>(move_group_node);
  std::cout << "fucking4" << std::endl;
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
