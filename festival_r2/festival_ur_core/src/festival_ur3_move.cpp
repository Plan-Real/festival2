#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <string>
#include <math.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_move_group_node");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

enum Status
{
  start,
  stop,
};


class MoveItPlanner : public rclcpp::Node {
public:
  MoveItPlanner(std::shared_ptr<rclcpp::Node> move_group_node_) 
    : Node("test_node"),
      move_group_arm(move_group_node_, PLANNING_GROUP_ARM),
      // joint_model_group_arm(
      //     move_group_arm.getCurrentState()->getJointModelGroup(
      //         PLANNING_GROUP_ARM)),
      robot_status(stop),
      period(100)
  {
    //최대 10초동안 상태를 받아오기 위해 대
    // current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
    //                                            this->joint_group_positions_arm);
    // //position받아오기
    // current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
    //                                            this->joint_group_velocities_arm);
    // joint_group_positions_arm =  move_group_arm.getCurrentJointValues();                                
    
    //subscribe
    // auto callback = std::bind(&MoveItPlanner::topic_callback, this, std::placeholders::_1);
    // subscriber_ = this->create_subscription<std_msgs::msg::String>("topic_name", 10, callback);
    
    // service
    auto start_callback = std::bind(&MoveItPlanner::startPicServer, this, std::placeholders::_1, std::placeholders::_2);
    service_start = move_group_node_->create_service<std_srvs::srv::Trigger>("start_pic", start_callback);
    RCLCPP_INFO(LOGGER, "Ready to execute start_pic server");

    auto finish_callback = std::bind(&MoveItPlanner::finishPicServer, this, std::placeholders::_1, std::placeholders::_2);
    service_finish = move_group_node_->create_service<std_srvs::srv::Trigger>("finish_pic", finish_callback);
    RCLCPP_INFO(LOGGER, "Ready to execute finish_pic server");

    // client
    first_client = create_client<std_srvs::srv::Trigger>("start_pic");
    second_client = create_client<std_srvs::srv::Trigger>("finish_pic");

    //tf
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    //initial setting
    joint_group_velocities_arm.assign(6,0);

    initialParamSetup();
    initialJointSetup();
    jointPrint(joint_group_positions_arm);
    joint_group_velocities_arm.assign(6,0);
    // joint_group_positions_arm.assign(6,0);
    
    //timer
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(int(period)),
                                           std::bind(&MoveItPlanner::timer_callback, this));
  }    
  //end of constructor
  //   void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  // {
  //   // Do something with the message
  // }

  void startPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    //서버 -> 로봇 : 사진찍자.
    response->success = true;
    response->message = "startpic";

    robot_status=start;
    RCLCPP_INFO(LOGGER, "sending back start_pic response");
  }

  void finishPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    //서버 -> 로봇 : 다찍었다.
    response->success = true;
    response->message = "finishpic";
    
    robot_status=stop;
    initialJointSetup();
    RCLCPP_INFO(LOGGER, "sending back finish_pic response");
  }


  void initialParamSetup(){
    // move_group_arm.setGoalTolerance(0.01);
    move_group_arm.setPlannerId("RRTConnectkConfigDefault");
    camera_x=0.07165;
    camera_z=0.089;
    human_vx=-0.1;
    human_vy=0.0;
    human_vz=-0.03;
    goal_x=0;
    goal_y=0;

    // diff_humanTocamera_x = 1.0; 
    // diff_humanTocamera_y = 
    // diff_humanTocamera_z = 0.2;
  }


  void timerHumanTf()
  {
    geometry_msgs::msg::TransformStamped t;
    
    std::string fromFrameRel = "world";
    std::string toFrameRel = "human";
    try 
    {
      t = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex) 
    {
      RCLCPP_INFO(
        LOGGER, "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }
    human_vx=(t.transform.translation.x-human_pose[0])*0.5;
    human_vz=(t.transform.translation.x-human_pose[1])*0.5;
    human_vz=(t.transform.translation.x-human_pose[2])*0.5;
    human_pose[0]=t.transform.translation.x;
    human_pose[1]=t.transform.translation.y;
    human_pose[2]=t.transform.translation.z;
    //사람이없는 경우 코드를 짜야됨.
  }


  void camera_pose(const double input[])
  {
    // input[0]
    // input[1]
    // input[2]
  }
 
  void initialJointSetup(){
    joint_group_positions_arm.emplace_back(0*M_PI/180); //base
    joint_group_positions_arm.emplace_back(-90*M_PI/180);
    joint_group_positions_arm.emplace_back(60*M_PI/180);
    joint_group_positions_arm.emplace_back(-144*M_PI/180);
    joint_group_positions_arm.emplace_back(-90*M_PI/180);
    joint_group_positions_arm.emplace_back(0);
    jointPrint(joint_group_positions_arm);
    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    move_group_arm.move();
    // service call 로봇이 멈춤이다.
    // robot_staus=stop;
  }


  void jointPrint(const std::vector<double> purpose_joint){
    std::cout << " 1 : " << purpose_joint[0]
      << " 2 : " << purpose_joint[1]
      << " 3 : " << purpose_joint[2]
      << " 4 : " << purpose_joint[3]
      << " 5 : " << purpose_joint[4]
      << " 6 : " << purpose_joint[5] << std::endl;
  }

  void jointupdate(){
      q2=joint_group_positions_arm[1];
      q3=joint_group_positions_arm[2];
      q4=joint_group_positions_arm[3];
      
      joint_group_velocities_arm[0]=0;
      joint_group_velocities_arm[1]=(-(1574*goal_x*cos(q3 + q4)*sin(q2 + q3) + 1574*goal_y*cos(q3 + q4)*sin(q2 + q3) + 1574*human_vx*cos(q3 + q4)*sin(q2 + q3) + 1574*human_vz*cos(q3 + q4)*sin(q2 + q3) + 890*goal_x*sin(q2 + q3)*sin(q3 + q4) + 890*goal_y*sin(q2 + q3)*sin(q3 + q4) + 890*human_vx*sin(q2 + q3)*sin(q3 + q4) + 890*human_vz*sin(q2 + q3)*sin(q3 + q4) + 2133*goal_x*sin(q2 + q3)*sin(q3) + 2133*goal_y*sin(q2 + q3)*sin(q3) + 2133*human_vx*sin(q2 + q3)*sin(q3) + 2133*human_vz*sin(q2 + q3)*sin(q3) - 1574*goal_x*cos(q4)*sin(q2) - 1574*goal_y*cos(q4)*sin(q2) - 1574*human_vx*cos(q4)*sin(q2) - 1574*human_vz*cos(q4)*sin(q2) + 2437*goal_x*sin(q2)*sin(q3) - 890*goal_x*sin(q2)*sin(q4) - 890*goal_y*sin(q2)*sin(q4) + 2437*human_vx*sin(q2)*sin(q3) - 890*human_vx*sin(q2)*sin(q4) - 890*human_vz*sin(q2)*sin(q4))/(10000*sin(q3)));
      joint_group_velocities_arm[2]=(-(1574*goal_x*cos(q2 + q3)*cos(q3 + q4) + 1574*goal_y*cos(q2 + q3)*cos(q3 + q4) + 1574*human_vx*cos(q2 + q3)*cos(q3 + q4) + 1574*human_vz*cos(q2 + q3)*cos(q3 + q4) + 890*goal_x*cos(q2 + q3)*sin(q3 + q4) + 890*goal_y*cos(q2 + q3)*sin(q3 + q4) + 890*human_vx*cos(q2 + q3)*sin(q3 + q4) + 890*human_vz*cos(q2 + q3)*sin(q3 + q4) + 2133*goal_x*cos(q2 + q3)*sin(q3) + 2133*goal_y*cos(q2 + q3)*sin(q3) + 2133*human_vx*cos(q2 + q3)*sin(q3) + 2133*human_vz*cos(q2 + q3)*sin(q3) - 1574*goal_x*cos(q2)*cos(q4) - 1574*goal_y*cos(q2)*cos(q4) - 1574*human_vx*cos(q2)*cos(q4) - 1574*human_vz*cos(q2)*cos(q4) + 2437*goal_x*cos(q2)*sin(q3) - 890*goal_x*cos(q2)*sin(q4) - 890*goal_y*cos(q2)*sin(q4) + 2437*human_vx*cos(q2)*sin(q3) - 890*human_vx*cos(q2)*sin(q4) - 890*human_vz*cos(q2)*sin(q4))/(10000*sin(q3)));
      joint_group_velocities_arm[3]=(goal_x + goal_y + human_vx + human_vz);
      joint_group_velocities_arm[4]=0;
      joint_group_velocities_arm[5]=0;
      jointPrint(joint_group_velocities_arm);
      joint_group_positions_arm[0]+=0.0;
      joint_group_positions_arm[1]+=period*joint_group_velocities_arm[1];
      joint_group_positions_arm[2]+=period*joint_group_velocities_arm[2];
      joint_group_positions_arm[3]+=period*joint_group_velocities_arm[3];
      joint_group_positions_arm[4]+=0.0;
      jointPrint(joint_group_positions_arm);
      // joint_group_positions_arm[5]+=0.5;
  }

  // void current_state(){
  //   RCLCPP_INFO(LOGGER, "get robot current state");
  //   current_state_arm= move_group_arm.getCurrentState(10);
  //   //최대 10초동안 상태를 받아오기 위해 대기

  //   current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
  //                                              this->joint_group_positions_arm);
  //   // //position받아오기

  //   current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
  //                                              this->joint_group_velocities_arm);
  //   //velocity받아오기
  // }

  void plan_arm_joint_space()
  {
  RCLCPP_INFO(LOGGER, "planning to joint space");
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  // joint_group_positions_arm.clear();

  bool success_arm = (move_group_arm.plan(my_paln_arm)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success_arm)
  {
    move_group_arm.asyncMove();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to plan to joint space");
  }


  }

  void timer_callback(){
    // this->timer_->cancel();

    //작동모드입니다.
    if(robot_status==start)
    {
      timerHumanTf();
      jointupdate();//목표 joint의 위치 계산
      plan_arm_joint_space();//보낸것
    }
    else
    {
      //작동모드일때 추정중인 여러 변수를 초기화해야함.
    }

    
    // current_state(); //현재 로봇팔의 위치
  }


private:
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm,
                      joint_group_velocities_arm,
                      purpose_joint;

  double camera_x, camera_z, goal_x, goal_y, 
         human_pose[3], human_vx, human_vy, human_vz,
         q2, q3, q4,period;
  Status robot_status;
  
  //moveit
  moveit::planning_interface::MoveGroupInterface move_group_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_paln_arm;
  const moveit::core::JointModelGroup *joint_model_group_arm;
  moveit::core::RobotStatePtr current_state_arm;

  // ros timer
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  //tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  //service server
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_finish;

  //service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr first_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr second_client;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_option;
  node_option.automatically_declare_parameters_from_overrides(true);

  auto move_group_node = rclcpp::Node::make_shared("ur_move_group_node", node_option);
  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<MoveItPlanner> planner_node = std::make_shared<MoveItPlanner>(move_group_node);


  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
