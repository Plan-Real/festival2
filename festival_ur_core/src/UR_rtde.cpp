#include "festival_ur_core/UR_rtde.hpp"

using planr::festival::urcontrol;

URcontrol::URcontrol(const std::string & ur_ip, const double control_time)
: control_time_(control_time),

{
    init_ur(ur_ip);

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

double URcontrol::NormalizeError(const std::vector<double> target, const std::vector<double> current, Eigen::VectorXd& error )
{
  error << (target[0]-current[0]),
            (target[1]-current[1]),
            (target[2]-current[2]),
            (target[3]-current[3]),
            (target[4]-current[4]),
            (target[5]-current[5]);
  return error.norm();
}

void URcontrol::UpdateJoint()
{

}


void URcontrol::FollowHuman(Eigen::Vector3d target_waypoint_vel)
{
    std::vector<double> joint_currentpose = GetJointQ();
    jacobian << -(787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*sin(joint_currentpose[1]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*sin(joint_currentpose[1] + joint_currentpose[2]))/10000, - (787*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
            (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000 - (2437*cos(joint_currentpose[1]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000 - (2133*cos(joint_currentpose[1] + joint_currentpose[2]))/10000,   (787*sin(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/5000 - (89*cos(joint_currentpose[1] + joint_currentpose[2] + joint_currentpose[3]))/1000,
            0.0,                                                                                        0.0,                                                            0.0,
            0.0,                                                                                        0.0,                                                            0.0,
            0.0,                                                                                        0.0,                                                            0.0,
            1.0,                                                                                        1.0,                                                            1.0;


    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::Matrix<double, 3, 1> joint_speed = pinv*target_waypoint_vel;

    std::vector joint_speed_v(joint_speed.data(), joint_speed.data()+joint_speed.size());

    steady_clock::time_point t_start = rtde_control.initPeriod();
    rtde_control.speedJ(joint_speed_v, acceleration, control_time_);
    rtde_control.waitPeriod(t_start);
}