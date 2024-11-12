#ifndef HOME_ACTION_HPP
#define HOME_ACTION_HPP

#include "BehaviorTreeStatefulAction.hpp"
#include "robotinterface_rtde.hpp"
#include "config_manager.hpp"
#include <memory>
#include <future>
#include <iostream>
#include <vector>

class HomeAction : public BehaviorTreeStatefulAction<HomeAction> {
public:
    HomeAction(const std::string& name, const BT::NodeConfiguration& config)
        : BehaviorTreeStatefulAction<HomeAction>(name, config), is_moving_(false)
    {
        // Load configuration for speed and acceleration limits
        ConfigManager& configManager = ConfigManager::getInstance();
        speed_limit_ = configManager.getSpeedLimit();
        acceleration_limit_ = configManager.getAccelerationLimit();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("robot_ip"),
            BT::InputPort<std::vector<double>>("home_joint_positions"),
            BT::InputPort<double>("speed"),
            BT::InputPort<double>("acceleration")
        };
    }

    BT::NodeStatus onStartImpl() {
        // Get input parameters
        if (!getInput("robot_ip", robot_ip_)) {
            throw BT::RuntimeError("HomeAction missing required input [robot_ip]");
        }
        if (!getInput("home_joint_positions", home_joint_positions_)) {
            throw BT::RuntimeError("HomeAction missing required input [home_joint_positions]");
        }
        if (!getInput("speed", speed_)) {
            throw BT::RuntimeError("HomeAction missing required input [speed]");
        }
        if (!getInput("acceleration", acceleration_)) {
            throw BT::RuntimeError("HomeAction missing required input [acceleration]");
        }

        // Validate joint positions
        if (home_joint_positions_.size() != 6) {
            throw BT::RuntimeError("Home joint positions must have 6 elements");
        }

        // Apply speed and acceleration limits
        double final_speed = std::min(speed_, speed_limit_);
        double final_acceleration = std::min(acceleration_, acceleration_limit_);

        // Initialize the robot interface
        try {
            robot_interface_ = std::make_unique<RobotInterface_RTDE>(robot_ip_);
            if (!robot_interface_->isConnected()) {
                throw std::runtime_error("Failed to connect to robot at " + robot_ip_);
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize RobotInterface_RTDE: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Start the moveJ command asynchronously
        is_moving_ = true;

        move_future_ = std::async(std::launch::async, [this, final_speed, final_acceleration]() {
            try {
                robot_interface_->MoveJoint(home_joint_positions_, final_speed, final_acceleration);
            } catch (const std::exception& e) {
                std::cerr << "moveJ to home position failed: " << e.what() << std::endl;
                throw;
            }
        });

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunningImpl() {
        if (move_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            try {
                move_future_.get(); // Check for exceptions
                is_moving_ = false;
                return BT::NodeStatus::SUCCESS;
            } catch (const std::exception& e) {
                std::cerr << "Exception in HomeAction: " << e.what() << std::endl;
                is_moving_ = false;
                return BT::NodeStatus::FAILURE;
            }
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHaltedImpl() {
        if (is_moving_) {
            try {
                robot_interface_->stop();
                is_moving_ = false;
                if (move_future_.valid()) {
                    move_future_.wait();
                }
            } catch (const std::exception& e) {
                std::cerr << "Failed to stop motion: " << e.what() << std::endl;
            }
        }
    }

private:
    std::unique_ptr<RobotInterface_RTDE> robot_interface_;
    std::string robot_ip_;
    std::vector<double> home_joint_positions_;
    double speed_;
    double acceleration_;
    double speed_limit_;
    double acceleration_limit_;
    bool is_moving_;
    std::future<void> move_future_;
};

#endif // HOME_ACTION_HPP
