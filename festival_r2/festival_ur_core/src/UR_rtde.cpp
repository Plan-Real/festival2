#include "festival_ur_core/UR_rtde.hpp"

using planr::festival::urcontrol;



//feedback???
//compare target_pose with current_pose
//Derivative it
URcontrol::URcontrol(const std::string & ur_ip, const double control_time)
: control_time_(control_time),

{
    init_ur(ur_ip);

}

//Convert eigen input to std::vector
void URcontrol::FollowHuman(const Eigen::Vector3d target_waypoint_vel, 
                            const double acceleration=0.5)
{

    std::vector<double> joint_speed_v;

    joint_speed_v=UpdateJointVel(target_waypoint_vel);

    steady_clock::time_point t_start = controlPtr->initPeriod();
    controlPtr->speedJ(joint_speed_v, acceleration, control_time_);
    controlPtr->waitPeriod(t_start);
}

void URcontrol::MoveJoint(const std::vector<double> target_joint,
                          const double speed=1.05,
                          const double accel=1.4,
                          const bool Asynchronous = false)
{
    controlPtr->moveJ(target_joint, speed, accel, Asynchronous);
}


void URcontrol::InitUR(const std::string & ur_ip)
{
    controlPtr=std::make_shared<RTDEControlInterface>(ur_ip);
    receivePtr=std::make_shared<RTDEReceiveInterface>(ur_ip);
}

std::vector<double> URcontrol::GetJointQ()
{
    return receivePtr->getActualQ();
}

std::vector<double> URcontrol::GetJointSpeed()
{
    return receivePtr->getActualQd();
}

double URcontrol::NormError(const std::vector<double> target, const std::vector<double> current)
{
  Eigen::VectorXd result;
  Eigen::Map<const Eigen::VectorXd> target_vec(target.data(), target.size()),
                                    current_vec(current.data(), current.size())
  result = target_vec- current_vec;
  return result.norm();
}

std::vector<double> URcontrol::UpdateJointVel(const Eigen::Vector3d target_waypoint_vel)
{
    std::vector<double> joint_currentpose, joint_vel_vector;
    Eigen::Matrix<double, 3, 6> jacobian;
    Eigen::Matrix<double, 3, 1> joint_vel;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pinv;
    
    joint_currentpose = GetJointQ();
    jacobian << -(787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*sin(joint_currentpose[1]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*cos(joint_currentpose[1]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
                0.0,                                                                                        0.0,                                                            0.0,
                0.0,                                                                                        0.0,                                                            0.0,
                0.0,                                                                                        0.0,                                                            0.0,
                1.0,                                                                                        1.0,                                                            1.0;
    
    pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    joint_vel = pinv*target_waypoint_vel;
    joint_vel_vector(joint_vel.data(), joint_vel.data()+joint_vel.size());

    return joint_vel_vector;
}
