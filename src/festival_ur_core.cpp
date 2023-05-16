#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "festival_ur_interfaces/msg/purpose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_move_group_node");

enum Status
{
  start,
  stop,
};


class FestivalNode : public rclcpp::Node {
public:
  FestivalNode(std::shared_ptr<rclcpp::Node> festival_ur_core_node_) 
    : Node("test_node"),
      joint_state_pose(6),
      joint_state_vel(6),
      joint_group_positions_arm(6),
      robot_status(stop),
      period(4000)
  {                        
    //subscribe
    auto joint_state_callback = std::bind(&FestivalNode::jointStateCallback, this, std::placeholders::_1);
    Joint_stae_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, joint_state_callback);
    
    // publisher
    joint_pose_publisher = this->create_publisher<festival_ur_interfaces::msg::Purpose>("joint_purpose", 10);
    // purpose_msgs = festival_ur_interfaces::msg::Purpose();
    // service
    service_start = festival_ur_core_node_->create_service<std_srvs::srv::Trigger>("start_pic", 
    std::bind(&FestivalNode::startPicServer, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(LOGGER, "Ready to excute start_pic server");

    service_finish = festival_ur_core_node_->create_service<std_srvs::srv::Trigger>("finish_pic", 
    std::bind(&FestivalNode::finishPicServer, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(LOGGER, "Ready to excute finish_pic server");

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
                                           std::bind(&FestivalNode::timer_callback, this));
  }    


  //end of constructor
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_pose[0]=msg->position[0];
    joint_state_pose[1]=msg->position[1];
    joint_state_pose[2]=msg->position[2];
    joint_state_pose[3]=msg->position[3];
    joint_state_pose[4]=msg->position[4];
    joint_state_pose[5]=msg->position[5];
    joint_state_vel[0]=msg->velocity[0];
    joint_state_vel[1]=msg->velocity[1];
    joint_state_vel[2]=msg->velocity[2];
    joint_state_vel[3]=msg->velocity[3];
    joint_state_vel[4]=msg->velocity[4];
    joint_state_vel[5]=msg->velocity[5];

  }

  void startPicServer(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(LOGGER, "sending back start_pic response");
    //서버 -> 로봇 : 사진찍자.
    response->success = true;
    response->message = "startpic";

    robot_status=start;
    service_start->send_response(*start_request_header, *response);
  }

  void finishPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    //서버 -> 로봇 : 다찍었다.
    response->success = true;
    response->message = "finishpic";
    
    robot_status=stop;
    initialJointSetup();
    initialParamSetup();
    RCLCPP_INFO(LOGGER, "sending back finish_pic response");
    service_finish->send_response(*finish_request_header, *response);
  }


  void initialParamSetup(){
    camera_x=0.07165;
    camera_z=0.089;
    human_vx=-0.0;
    human_vy=0.0;
    human_vz=-0.0;
    human_pose[0]=0;
    human_pose[1]=0;
    human_pose[2]=0;
    goal_x=0;
    goal_y=0;
    goal_z=0;

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
    joint_group_positions_arm[0]=(-180*M_PI/180); //base
    joint_group_positions_arm[1]=(0*M_PI/180);
    joint_group_positions_arm[2]=(-40*M_PI/180);
    joint_group_positions_arm[3]=(-150*M_PI/180);
    joint_group_positions_arm[4]=(-90*M_PI/180);
    joint_group_positions_arm[5]=(0);
    jointPrint(joint_group_positions_arm);
    purpose_msgs.joints = joint_group_positions_arm;
    purpose_msgs.time = period/1000;
    
    
    // service call 로봇이 멈춤이다.
    // robot_staus=stop;
  }

  void secondJointSetup(){
    joint_group_positions_arm[0]=(-180*M_PI/180); //base
    joint_group_positions_arm[1]=(-155*M_PI/180);
    joint_group_positions_arm[2]=(71*M_PI/180);
    joint_group_positions_arm[3]=(-87*M_PI/180);
    joint_group_positions_arm[4]=(-90*M_PI/180);
    joint_group_positions_arm[5]=(0);
    jointPrint(joint_group_positions_arm);
    purpose_msgs.joints = joint_group_positions_arm;
    purpose_msgs.time = period/1000;
    
    
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




  void timer_callback(){
    // this->timer_->cancel();
    
    //작동모드입니다.
    if(robot_status==stop)
    {

      secondJointSetup();
      // timerHumanTf();
      // jointupdate();//목표 joint의 위치 계산
      
    }
    else
    {
      initialJointSetup();
      //작동모드일때 추정중인 여러 변수를 초기화해야함.
    }
    joint_pose_publisher->publish(purpose_msgs);

    // current_state(); //현재 로봇팔의 위치
  }


private:
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_state_pose,
                      joint_state_vel,
                      joint_group_velocities_arm,
                      joint_group_positions_arm,
                      purpose_joint;

  double camera_x, camera_z, goal_x, goal_y, goal_z,
         human_pose[3], human_vx, human_vy, human_vz,
         q2, q3, q4,period;
  Status robot_status;
  
  // ros subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_stae_subscriber;

  // ros publisher
  rclcpp::Publisher<festival_ur_interfaces::msg::Purpose>::SharedPtr joint_pose_publisher;
  festival_ur_interfaces::msg::Purpose purpose_msgs;

  // ros timer
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  //tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  //service server
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_finish;
  const std::shared_ptr<rmw_request_id_t> start_request_header;
  const std::shared_ptr<rmw_request_id_t> finish_request_header;
  //service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr first_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr second_client;
};








int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto festival_ur_core_node = rclcpp::Node::make_shared("festival_ur_core_node");
  rclcpp::executors::SingleThreadedExecutor festival_executor;
  std::shared_ptr<FestivalNode> core_node = std::make_shared<FestivalNode>(festival_ur_core_node);


  festival_executor.add_node(core_node);
  festival_executor.spin();

  rclcpp::shutdown();
  return 0;
}