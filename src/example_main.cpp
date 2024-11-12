// main.cpp

#include "BTCoordinatorNode.hpp"
#include "HomeAction.hpp"
#include "MoveLAction.hpp"
#include "MoveJAction.hpp"
#include "WaitAction.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Set the path to the Behavior Tree XML file
    const std::string bt_xml_file = "bt_struct.xml";

    // Create the BTCoordinatorNode
    auto coordinator_node = std::make_shared<BTCoordinatorNode>("bt_coordinator", bt_xml_file);

    // Register the custom actions with the BT factory
    coordinator_node->RegisterAction<HomeAction>("HomeAction");
    coordinator_node->RegisterAction<MoveLAction>("MoveLAction");
    coordinator_node->RegisterAction<MoveJAction>("MoveJAction");
    coordinator_node->RegisterAction<WaitAction>("WaitAction");

    // Initialize the Behavior Tree by loading it from the XML file
    coordinator_node->InitTree();

    // Spin the node and execute the Behavior Tree
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        coordinator_node->execute();
        rclcpp::spin_some(coordinator_node);
        loop_rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
