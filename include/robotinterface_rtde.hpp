// robotinterface_rtde.hpp

#ifndef ROBOTINTERFACE_RTDE_HPP
#define ROBOTINTERFACE_RTDE_HPP

#include "robotinterface.hpp"
#include <rtde_control_interface.h>
#include <rtde_receive_interface.h>
#include <memory>
#include <stdexcept>

class RobotInterface_RTDE : public RobotInterface
{
public:
    RobotInterface_RTDE(const std::string& robot_ip)
        : RobotInterface(robot_ip)
    {
        rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
    }

    bool isConnected() const override
    {
        return rtde_control_->isConnected() && rtde_receive_->isConnected();
    }

    bool MoveLinearToPosition(const std::vector<double>& pose, double speed, double acceleration) override
    {
        if (!rtde_control_->isConnected())
            throw std::runtime_error("RTDE Control Interface not connected.");

        rtde_control_->moveL(pose, speed, acceleration);
        return true;
    }

    bool MoveJoint(const std::vector<double>& joint_positions, double speed, double acceleration) override
    {
        if (!rtde_control_->isConnected())
            throw std::runtime_error("RTDE Control Interface not connected.");

        rtde_control_->moveJ(joint_positions, speed, acceleration);
        return true;
    }

    std::vector<double> getActualQ() const override
    {
        if (!rtde_receive_->isConnected())
            throw std::runtime_error("RTDE Receive Interface not connected.");

        return rtde_receive_->getActualQ();
    }

    std::vector<double> getActualTCPPosition() const override
    {
        if (!rtde_receive_->isConnected())
            throw std::runtime_error("RTDE Receive Interface not connected.");

        return rtde_receive_->getActualTCPPose();
    }

    void stop() override
    {
        if (rtde_control_->isConnected())
            rtde_control_->stopScript();
    }

private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
};

#endif // ROBOTINTERFACE_RTDE_HPP
