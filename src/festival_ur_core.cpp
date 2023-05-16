#include <memory>
#include <string>
#include <math.h>
#include <Eigen/Core>

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
  FestivalNode() 
    : Node("festival_ur_core_node"),
      joint_currentpose(6),
      joint_state_vel(6),
      joint_targetpose(6),
      joint_group_velocities_arm(6),
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
    service_start = this->create_service<std_srvs::srv::Trigger>("start_pic", 
    std::bind(&FestivalNode::startPicServer, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(LOGGER, "Ready to excute start_pic server");

    service_finish = this->create_service<std_srvs::srv::Trigger>("finish_pic", 
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

    initialParamSetup();
    initialJointSetup();
    jointPrint(joint_targetpose);
    
    //timer
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(int(period)),
                                           std::bind(&FestivalNode::timer_callback, this));
  }    


  //end of constructor
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_currentpose[0]=msg->position[0];
    joint_currentpose[1]=msg->position[1];
    joint_currentpose[2]=msg->position[2];
    joint_currentpose[3]=msg->position[3];
    joint_currentpose[4]=msg->position[4];
    joint_currentpose[5]=msg->position[5];
    joint_state_vel[0]=msg->velocity[0];
    joint_state_vel[1]=msg->velocity[1];
    joint_state_vel[2]=msg->velocity[2];
    joint_state_vel[3]=msg->velocity[3];
    joint_state_vel[4]=msg->velocity[4];
    joint_state_vel[5]=msg->velocity[5];

  }

  void startPicServer(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(LOGGER, "sending back start_pic response");
    //서버 -> 로봇 : 사진찍자.
    response->success = true;
    response->message = "startpic";

    robot_status=start;
    secondJointSetup();
  }

  void finishPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    //서버 -> 로봇 : 다찍었다.
    response->success = true;
    response->message = "finishpic";
    
    robot_status=stop;
    initialJointSetup();
    initialParamSetup();
    RCLCPP_INFO(LOGGER, "sending back finish_pic response");
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
    
    std::string fromFrameRel = "base_link";
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
    joint_targetpose[0]=(-180*M_PI/180); //base
    joint_targetpose[1]=(0*M_PI/180);
    joint_targetpose[2]=(-40*M_PI/180);
    joint_targetpose[3]=(-150*M_PI/180);
    joint_targetpose[4]=(-90*M_PI/180);
    joint_targetpose[5]=(0);
    jointPrint(joint_targetpose);
    purpose_msgs.joints = joint_targetpose;
    purpose_msgs.time = 3;
    
    
    // service call 로봇이 멈춤이다.
    // robot_staus=stop;
  }

  void secondJointSetup(){
    joint_targetpose[0]=(-180*M_PI/180); //base
    joint_targetpose[1]=(-155*M_PI/180);
    joint_targetpose[2]=(71*M_PI/180);
    joint_targetpose[3]=(-87*M_PI/180);
    joint_targetpose[4]=(-90*M_PI/180);
    joint_targetpose[5]=(0);
    jointPrint(joint_targetpose);
    purpose_msgs.joints = joint_targetpose;
    purpose_msgs.time = 3;
    
    
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
      jacobian << -(787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*sin(joint_currentpose[1]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                  (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*cos(joint_currentpose[1]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                  0,                                                                                        0,                                                            0,
                  0,                                                                                        0,                                                            0,
                  0,                                                                                        0,                                                            0,
                  1,                                                                                        1,                                                            1;
      
      joint_group_velocities_arm[0]=0;
      joint_group_velocities_arm[1]=0;
      joint_group_velocities_arm[2]=0;
      joint_group_velocities_arm[3]=0;
      joint_group_velocities_arm[4]=0;
      joint_group_velocities_arm[5]=0;
      jointPrint(joint_group_velocities_arm);
      joint_targetpose[0]+=0.0;
      joint_targetpose[1]+=period*joint_group_velocities_arm[1];
      joint_targetpose[2]+=period*joint_group_velocities_arm[2];
      joint_targetpose[3]+=period*joint_group_velocities_arm[3];
      joint_targetpose[4]+=0.0;
      jointPrint(joint_targetpose);
      // joint_targetpose[5]+=0.5;
  }

  bool check_arrival(const std::vector<double> current, 
                     const std::vector<double> purpose, 
                     const double threshold)
  {
    std::vector<double> error(6);

    for (long unsigned int i=0; i<error.size();i++){
      error[i]=std::abs(purpose[i]-current[i]);
      if(error[i]<threshold) return true;
    }
    return false;
  }


  void timer_callback(){
    // this->timer_->cancel();
    
    //작동모드입니다.
    if(robot_status==start)
    {
      if(check_arrival(joint_currentpose, joint_targetpose, 0.001))
      {
        timerHumanTf();
        jointupdate();
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Arrive");
      }

      
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
  std::vector<double> joint_currentpose,
                      joint_state_vel,
                      joint_group_velocities_arm,
                      joint_targetpose,
                      purpose_joint;

  double camera_x, camera_z, goal_x, goal_y, goal_z,
         human_pose[3], human_vx, human_vy, human_vz,
         q2, q3, q4,period;
  Eigen::Matrix<short, 6, 3> jacobian;
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
  //service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr first_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr second_client;
};








int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<FestivalNode>());
  rclcpp::shutdown();
  return 0;
}