#ifndef MOVE_L_ACTION_HPP
#define MOVE_L_ACTION_HPP

#include "BehaviorTreeStatefulAction.hpp"
#include "robotinterface_rtde.hpp"
#include "config_manager.hpp"
#include <memory>
#include <future>
#include <iostream>
#include <vector>

class MoveLAction : public BehaviorTreeStatefulAction<MoveLAction> {
public:
    MoveLAction(const std::string& name, const BT::NodeConfiguration& config)
        : BehaviorTreeStatefulAction<MoveLAction>(name, config), is_moving_(false)
    {
        // Load configuration for speed and acceleration limits
        ConfigManager& configManager = ConfigManager::getInstance();
        speed_limit_ = configManager.getSpeedLimit();
        acceleration_limit_ = configManager.getAccelerationLimit();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("robot_ip"),
            BT::InputPort<std::vector<double>>("position"),
            BT::InputPort<std::vector<double>>("orientation"),
            BT::InputPort<double>("speed"),
            BT::InputPort<double>("acceleration")
        };
    }

    BT::NodeStatus onStartImpl() {
        // Get input parameters
        if (!getInput("robot_ip", robot_ip_)) {
            throw BT::RuntimeError("MoveLAction missing required input [robot_ip]");
        }
        if (!getInput("position", position_)) {
            throw BT::RuntimeError("MoveLAction missing required input [position]");
        }
        if (!getInput("orientation", orientation_)) {
            throw BT::RuntimeError("MoveLAction missing required input [orientation]");
        }
        if (!getInput("speed", speed_)) {
            throw BT::RuntimeError("MoveLAction missing required input [speed]");
        }
        if (!getInput("acceleration", acceleration_)) {
            throw BT::RuntimeError("MoveLAction missing required input [acceleration]");
        }

        // Combine position and orientation into a single pose vector
        if (position_.size() != 3 || orientation_.size() != 3) {
            throw BT::RuntimeError("Position and orientation must have 3 elements each");
        }
        std::vector<double> pose = position_;
        pose.insert(pose.end(), orientation_.begin(), orientation_.end());

        // Limit speed and acceleration based on configuration
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

        // Start the moveL command asynchronously
        is_moving_ = true;

        move_future_ = std::async(std::launch::async, [this, pose, final_speed, final_acceleration]() {
            try {
                robot_interface_->MoveLinearToPosition(pose, final_speed, final_acceleration);
            } catch (const std::exception& e) {
                std::cerr << "moveL failed: " << e.what() << std::endl;
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
                std::cerr << "Exception in MoveLAction: " << e.what() << std::endl;
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
    std::vector<double> position_;
    std::vector<double> orientation_;
    double speed_;
    double acceleration_;
    double speed_limit_;
    double acceleration_limit_;
    bool is_moving_;
    std::future<void> move_future_;
};

#endif // MOVE_L_ACTION_HPP
