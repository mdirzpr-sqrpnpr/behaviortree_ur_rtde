// robotinterface.hpp

#ifndef ROBOTINTERFACE_HPP
#define ROBOTINTERFACE_HPP

#include <string>
#include <vector>

class RobotInterface
{
public:
    RobotInterface(const std::string& robot_ip) : robot_ip_(robot_ip) {}

    virtual ~RobotInterface() = default;

    // Pure virtual functions
    virtual bool isConnected() const = 0;
    virtual bool MoveLinearToPosition(const std::vector<double>& pose, double speed, double acceleration) = 0;
    virtual bool MoveJoint(const std::vector<double>& joint_positions, double speed, double acceleration) = 0;
    virtual std::vector<double> getActualQ() const = 0;
    virtual std::vector<double> getActualTCPPosition() const = 0;
    virtual void stop() = 0;

protected:
    std::string robot_ip_;
};

#endif // ROBOTINTERFACE_HPP
