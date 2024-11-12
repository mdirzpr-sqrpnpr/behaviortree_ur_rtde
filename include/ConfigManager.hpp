#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP

#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <stdexcept>

class ConfigManager {
public:
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }

    void loadConfig(const std::string& configFilePath) {
        std::ifstream configFile(configFilePath, std::ifstream::binary);
        if (!configFile.is_open()) {
            throw std::runtime_error("Failed to open config file: " + configFilePath);
        }

        Json::Value root;
        configFile >> root;

        parseConfig(root);
    }

    // Accessors for configuration parameters
    std::vector<double> getHomeJointPositions() const {
        return home_joint_positions_;
    }

    double getSpeedLimit() const {
        return speed_limit_;
    }

    double getAccelerationLimit() const {
        return acceleration_limit_;
    }

private:
    ConfigManager() = default; // Private constructor for singleton pattern

    // Configuration parameters
    std::vector<double> home_joint_positions_;
    double speed_limit_;
    double acceleration_limit_;

    // Helper method to parse the JSON configuration
    void parseConfig(const Json::Value& root) {
        // Parse parameters
        if (root.isMember("parameters")) {
            const auto& parameters = root["parameters"];

            // Speed limit
            if (parameters.isMember("speed_limit")) {
                speed_limit_ = parameters["speed_limit"].asDouble();
            } else {
                speed_limit_ = 0.5; // Default value
            }

            // Acceleration limit
            if (parameters.isMember("acceleration_limit")) {
                acceleration_limit_ = parameters["acceleration_limit"].asDouble();
            } else {
                acceleration_limit_ = 0.5; // Default value
            }

            // Home joint positions
            if (parameters.isMember("home_joint_pos")) {
                const auto& home_joints = parameters["home_joint_pos"];
                if (!home_joints.isArray() || home_joints.size() != 6) {
                    throw std::runtime_error("home_joint_pos must be an array of 6 elements");
                }
                home_joint_positions_.clear();
                for (const auto& joint_value : home_joints) {
                    home_joint_positions_.push_back(joint_value.asDouble());
                }
            } else {
                throw std::runtime_error("home_joint_pos is missing in the config file");
            }
        } else {
            throw std::runtime_error("parameters section is missing in the config file");
        }
    }
};

#endif // CONFIG_MANAGER_HPP
