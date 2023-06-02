#include "festival_ur_core/festival.hpp"

#include <memory>
#include <string>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/QR>

FestivalNode::FestivalNode() 
  : Node("festival_ur_core_node"),
    joint_currentpose(6),
    joint_state_vel(6),
    joint_targetpose(6),
    joint_group_velocities_arm(6),
    robot_status(start),
    camera_targetpose(6),
    camera_currentpose(6),
    error(6),
    period(100),
    jacobian(6,3)
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
void FestivalNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void FestivalNode::startPicServer(
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

void FestivalNode::finishPicServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
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


void FestivalNode::initialParamSetup(){
  camera_x=0.07165;
  camera_z=0.089;
  goal_x=0;
  goal_y=0;
  goal_z=0;
}


void FestivalNode::timercameraTf()
{
  geometry_msgs::msg::TransformStamped end, camera;
  
  std::string from_frame_base = "shoulder_link";
  std::string to_frame_end = "end";
  std::string to_frame_camera_link = "front_camera_link";
  try 
  {
    end = tf_buffer_->lookupTransform(
        to_frame_end, from_frame_base,
        tf2::TimePointZero);

  }
  catch (const tf2::TransformException & ex) 
  {
    RCLCPP_INFO(
      LOGGER, "Could not transform %s to %s: %s",
      to_frame_end.c_str(), from_frame_base.c_str(), ex.what());
    return;
  }
  try 
  {
        camera = tf_buffer_->lookupTransform(
        to_frame_camera_link, from_frame_base,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) 
  {
    RCLCPP_INFO(
      LOGGER, "Could not transform %s to %s: %s",
      to_frame_camera_link.c_str(), from_frame_base.c_str(), ex.what());
    return;
  }
  RCLCPP_INFO(LOGGER, "%lf", camera.transform.rotation.x);
  std::vector<double> camera_euler = quaternionToEuler(camera.transform.rotation.x, 
                                                        camera.transform.rotation.y, 
                                                        camera.transform.rotation.z, 
                                                        camera.transform.rotation.w);
  static bool executed = false;
  if(!executed)
  {
    camera_targetpose[3] = camera_euler[0];
    camera_targetpose[4] = camera_euler[1];
    camera_targetpose[5] = camera_euler[2];
    executed= true;
  }

  camera_targetpose[0]=end.transform.translation.x;
  camera_targetpose[1]=end.transform.translation.y;
  camera_targetpose[2]=end.transform.translation.z;

  camera_currentpose[0]=camera.transform.translation.x;
  camera_currentpose[1]=camera.transform.translation.y;
  camera_currentpose[2]=camera.transform.translation.z;
  camera_currentpose[3]=camera_euler[0];
  camera_currentpose[4]=camera_euler[1];
  camera_currentpose[5]=camera_euler[2];
  
  }

std::vector<double> FestivalNode::quaternionToEuler(double qx, double qy, double qz, double qw) {
    tf2::Quaternion quat(qx, qy, qz, qw);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    std::vector<double> euler = {roll, pitch, yaw};
    return euler;
}

double FestivalNode::errorNorm(const std::vector<double> target, const std::vector<double> current, Eigen::VectorXd& error )
{
  error << (target[0]-current[0]),
            (target[1]-current[1]),
            (target[2]-current[2]),
            (target[3]-current[3]),
            (target[4]-current[4]),
            (target[5]-current[5]);
  return error.norm();
}

void FestivalNode::initialJointSetup(){
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

void FestivalNode::secondJointSetup(){
  joint_targetpose[0]=(-180*M_PI/180); //base
  joint_targetpose[1]=(-47*M_PI/180);
  joint_targetpose[2]=(-100*M_PI/180);
  joint_targetpose[3]=(-26*M_PI/180);
  joint_targetpose[4]=(-90*M_PI/180);
  joint_targetpose[5]=(0);
  jointPrint(joint_targetpose);
  purpose_msgs.joints = joint_targetpose;
  purpose_msgs.time = 3;
  
  
  // service call 로봇이 멈춤이다.
  // robot_staus=stop;
}

void FestivalNode::jointPrint(const std::vector<double> purpose_joint){
  std::cout << " 1 : " << purpose_joint[0]
    << " 2 : " << purpose_joint[1]
    << " 3 : " << purpose_joint[2]
    << " 4 : " << purpose_joint[3]
    << " 5 : " << purpose_joint[4]
    << " 6 : " << purpose_joint[5] << std::endl;
}

void FestivalNode::jointupdate(){
    RCLCPP_INFO(LOGGER, "sending back start_pic response");
    jacobian << -(787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*sin(joint_currentpose[1]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*cos(joint_currentpose[1]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                0.0,                                                                                        0.0,                                                            0.0,
                0.0,                                                                                        0.0,                                                            0.0,
                0.0,                                                                                        0.0,                                                            0.0,
                1.0,                                                                                        1.0,                                                            1.0;
    
    RCLCPP_INFO(LOGGER, "sending back start_pic response");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    
    pinv_error=pinv*error;
    joint_targetpose[0]+=0.0;
    joint_targetpose[1]+=period*0.001*pinv_error(0,0);
    joint_targetpose[2]+=period*0.001*pinv_error(1,0);
    joint_targetpose[3]+=period*0.001*pinv_error(2,0);
    joint_targetpose[4]+=0.0;
    purpose_msgs.joints = joint_targetpose;
    purpose_msgs.time = period*0.001;
    
    // joint_targetpose[5]+=0.5;
}

bool FestivalNode::check_arrival(const std::vector<double> current, 
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


void FestivalNode::timer_callback(){
  // this->timer_->cancel();
  
  //작동모드입니다.
  if(robot_status==start)
  {
    // if(check_arrival(joint_currentpose, joint_targetpose, 0.001))
    // {
    timercameraTf();
    if(std::abs(errorNorm(camera_targetpose,camera_currentpose, error)-0.1)>0)
    {
      
      jointupdate();
      std::cout << jacobian << std::endl;
      std::cout << error << std::endl;
      jointPrint(joint_targetpose);

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
