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

private:

    void InitUR(const std::string & ur_ip);

    std::vector<double> GetJointQ();
    std::vector<double> GetJointSpeed();
    
    
    double NormalizeError(const std::vector<double> target, 
                     const std::vector<double> current, 
                     Eigen::VectorXd& error);

    void UpdateJoint();

    void FollowHuman();
    std::shared_ptr<RTDEControlInterface> controlPtr;
    std::shared_ptr<RTDEReceiveInterface> receivePtr;
    
    double control_time_;
}
} // namespace URcontrol
} // namespace festival
} // namespace planr
