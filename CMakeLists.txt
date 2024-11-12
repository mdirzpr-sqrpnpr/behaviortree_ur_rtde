cmake_minimum_required(VERSION 3.8)
project(bt_coordinator)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behavior_tree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(include)

# Add the main executable
add_executable(bt_coordinator_node src/example_main.cpp src/BTCoordinatorNode.cpp)

# Link libraries to the executable
target_link_libraries(bt_coordinator_node
    rclcpp::rclcpp
    behavior_tree_cpp::behaviortree_cpp
)

# Install the executable
install(TARGETS bt_coordinator_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files if necessary
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Enable testing if needed
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Finish package configuration
ament_package()
