#ifndef BT_COORDINATOR_NODE_HPP
#define BT_COORDINATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <memory>

class BTCoordinatorNode : public rclcpp::Node
{
public:
    BTCoordinatorNode(const std::string &node_name, const std::string &bt_xml_file);

    // Function to register a new Action node
    template <typename CustomActionType>
    void RegisterAction(const std::string &action_name)
    {
        BT::NodeBuilder builder_action =
            [](const std::string &name, const BT::NodeConfiguration &config)
            {
                return std::make_unique<CustomActionType>(name, config);
            };
        factory_.registerBuilder<CustomActionType>(action_name, builder_action);
    }

    // Function to register a new Condition node
    template <typename CustomConditionType>
    void RegisterCondition(const std::string &condition_name)
    {
        BT::NodeBuilder builder_condition =
            [](const std::string &name, const BT::NodeConfiguration &config)
            {
                return std::make_unique<CustomConditionType>(name, config);
            };
        factory_.registerBuilder<CustomConditionType>(condition_name, builder_condition);
    }

    // Function to initialize the behavior tree by loading from the XML file
    void InitTree();

    // Function to execute the behavior tree
    void execute();

    // Make factory_ public for access in main.cpp
    BT::BehaviorTreeFactory factory_;

private:
    std::shared_ptr<BT::Tree> tree_;
    std::string bt_xml_file_; // Path to the XML file for the behavior tree
};

#endif // BT_COORDINATOR_NODE_HPP
