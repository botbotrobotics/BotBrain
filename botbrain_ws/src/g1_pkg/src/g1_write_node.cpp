#include <g1_write.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create the lifecycle node
    auto node = std::make_shared<G1Write>();
    
    // Create executor
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node->get_node_base_interface());
    
    // Spin the node
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}