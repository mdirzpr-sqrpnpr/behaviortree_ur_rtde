// GetActualQAction.hpp

#ifndef GET_ACTUAL_Q_ACTION_HPP
#define GET_ACTUAL_Q_ACTION_HPP

#include "BehaviorTreeStatefulAction.hpp"
#include "robotinterface_rtde.hpp" // Updated to the new robot interface
#include <iostream>
#include <memory>
#include <vector>

class GetActualQAction : public BehaviorTreeStatefulAction<GetActualQAction>
{
public:
    GetActualQAction(const std::string& name, const BT::NodeConfiguration& config)
        : BehaviorTreeStatefulAction(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("robot_ip"),
            BT::OutputPort<std::vector<double>>("joint_positions")
        };
    }

    BT::NodeStatus onStartImpl()
    {
        // Retrieve robot IP from input port
        if (!getInput("robot_ip", robot_ip_))
        {
            throw BT::RuntimeError("GetActualQAction missing required input [robot_ip]");
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

        // Get the current joint positions
        try
        {
            joint_positions_ = robot_interface_->getActualQ();
            setOutput("joint_positions", joint_positions_);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get actual joint positions: " << e.what() << std::endl;
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
    std::vector<double> joint_positions_;
};

#endif // GET_ACTUAL_Q_ACTION_HPP
