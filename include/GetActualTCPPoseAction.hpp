// GetActualTCPPoseAction.hpp

#ifndef GET_ACTUAL_TCP_POSE_ACTION_HPP
#define GET_ACTUAL_TCP_POSE_ACTION_HPP

#include "BehaviorTreeStatefulAction.hpp"
#include "robotinterface_rtde.hpp" // Use the new robot interface
#include <iostream>
#include <memory>
#include <vector>

class GetActualTCPPoseAction : public BehaviorTreeStatefulAction<GetActualTCPPoseAction>
{
public:
    GetActualTCPPoseAction(const std::string& name, const BT::NodeConfiguration& config)
        : BehaviorTreeStatefulAction(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("robot_ip"),
            BT::OutputPort<std::vector<double>>("pose")
        };
    }

    BT::NodeStatus onStartImpl()
    {
        // Retrieve robot IP from input port
        if (!getInput("robot_ip", robot_ip_))
        {
            throw BT::RuntimeError("GetActualTCPPoseAction missing required input [robot_ip]");
        }

        // Initialize the robot interface
        try
        {
            robot_interface_ = std::make_unique<RobotInterface_RTDE>(robot_ip_);
            if (!robot_interface_->isConnected())
            {
                throw std::runtime_error("Failed to connect to robot at " + robot_ip_);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error initializing RobotInterface_RTDE: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Get the current TCP position
        try
        {
            tcp_pose_ = robot_interface_->getActualTCPPosition();
            setOutput("pose", tcp_pose_);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get actual TCP position: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunningImpl()
    {
        // Since this action is immediate, it returns SUCCESS
        return BT::NodeStatus::SUCCESS;
    }

    void onHaltedImpl()
    {
        // No specific halt action required
    }

private:
    std::unique_ptr<RobotInterface_RTDE> robot_interface_;
    std::string robot_ip_;
    std::vector<double> tcp_pose_;
};

#endif // GET_ACTUAL_TCP_POSE_ACTION_HPP
