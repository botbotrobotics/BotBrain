#include <lifecycle_manager.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS 2 so the lifecycle manager can create its interfaces.
    rclcpp::init(argc, argv);
    
    // Instantiate the lifecycle manager node that orchestrates all state transitions.
    auto node = std::make_shared<LifecycleManager>();
    
    // Multi-threaded executor lets the lifecycle manager handle callbacks concurrently.
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node->get_node_base_interface());
    
    // Block here while the executor processes lifecycle events.
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
