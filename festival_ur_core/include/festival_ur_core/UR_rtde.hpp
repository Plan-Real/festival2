#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>
#include <memory>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/QR>

using namespace ur_rtde;
using namespace std::chrono;


namespace planr
{
namespace festival
{
namespace urcontrol
{
class URcontrol 
{
public:
    explicit URcontrol(const std::string & ur_ip, const double control_time)
    virtual ~URcontrol(){}

    void ChangeControltime(const double control_time){ control_time_=control_time; }

    void FollowHuman(const Eigen::Vector3d target_waypoint_vel, const double acceleration=0.5);

    void MoveJoint(const std::vector<double> target_joint,
                   const double speed=1.05,
                   const double accel=1.4,
                   const bool Asynchronous = false);

private:

    void InitUR(const std::string & ur_ip);

    std::vector<double> GetJointQ();
    std::vector<double> GetJointSpeed();
    
    
    double NormError(const std::vector<double> target, 
                     const std::vector<double> current);

    std::vector<double> UpdateJointVel(const Eigen::Vector3d target_waypoint_vel);


    std::shared_ptr<RTDEControlInterface> controlPtr;
    std::shared_ptr<RTDEReceiveInterface> receivePtr;
    
    double control_time_;
}
} // namespace URcontrol
} // namespace festival
} // namespace planr
