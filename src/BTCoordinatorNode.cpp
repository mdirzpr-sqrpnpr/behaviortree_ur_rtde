// BTCoordinatorNode.cpp

#include "BTCoordinatorNode.hpp"

BTCoordinatorNode::BTCoordinatorNode(const std::string &node_name, const std::string &bt_xml_file)
    : Node(node_name), bt_xml_file_(bt_xml_file)
{
}

void BTCoordinatorNode::InitTree()
{
    // Load the BT from XML
    tree_ = std::make_shared<BT::Tree>(factory_.createTreeFromFile(bt_xml_file_));
}

void BTCoordinatorNode::execute()
{
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree_->tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Behavior Tree executed successfully.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Behavior Tree execution failed.");
    }
}
