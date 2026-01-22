#ifndef LIFECYCLEMANAGER_HPP
#define LIFECYCLEMANAGER_HPP

#include <mutex>
#include <thread>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <optional>
#include <unordered_map>

#include "types.hpp"
#include "graph_node.hpp"
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "bot_custom_interfaces/msg/status_array.hpp"
#include "bot_custom_interfaces/srv/state_machine.hpp"

class StateController; 

/**
 * @brief Lifecycle-aware orchestrator that loads node graphs and controls
 * state transitions for every managed node.
 *
 * Publishes aggregated status, exposes state-machine commands, and subscribes
 * to lifecycle transition events to keep the internal state controller in sync.
 */
class LifecycleManager : public rclcpp_lifecycle::LifecycleNode 
{
private:
    // ─────────────── Internal state ───────────────

    // Aggregated lifecycle snapshot used for reporting.
    State state;

    // Node graph parsed from JSON definitions.
    std::vector<NodeProfile> nodes_;

    // Controller responsible for driving state transitions.
    std::shared_ptr<StateController> controller_;    

    // ─────────────── ROS I/O ───────────────

    // Publishes aggregated node status.
    rclcpp_lifecycle::LifecyclePublisher<bot_custom_interfaces::msg::StatusArray>::SharedPtr 
        status_pub_;

    // Exposes the state-machine command service.
    rclcpp::Service<bot_custom_interfaces::srv::StateMachine>::SharedPtr state_machine_srv_;

    // Periodic status publisher timer.
    rclcpp::TimerBase::SharedPtr publisher_timer_;  

    // Callback group used to isolate lifecycle service callbacks.
    rclcpp::CallbackGroup::SharedPtr cbg_;

    // Query clients for lifecycle state per node.
    std::unordered_map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_srvs_;

    // Clients used to request lifecycle transitions (configure/activate/etc).
    std::unordered_map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_srvs_;

    // Transition-event subscriptions keyed by node name.
    std::unordered_map<std::string,
        rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> transition_subs_;

public:
    // Constructor.
    LifecycleManager();

    // Destructor.
    ~LifecycleManager() override;

    // Load parameters, node graph and create base ROS interfaces.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;

    // Activate publishers, start timers, and allow controllers to run.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;

    // Pause timers and put publishers/services back into inactive mode.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // Clean up resources
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    // Shutdown node and stop of the manager safely.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    // Request a lifecycle transition on a managed node.
    bool set_state(const std::string& node_name, const uint8_t state);

    // Query the current lifecycle state of a managed node.
    std::optional<lifecycle_msgs::msg::State> get_state(const std::string& node_name);

    // Log info message.
    void print_info(const std::string& s);
    
    // Log error message.
    void print_error(const std::string& s);

private:
    // Handle lifecycle transition events coming from managed nodes.
    void transition_callback(const std::string& node_name, 
                             const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

    // Handle external state-machine service requests.
    void command_srv_callback(
        const bot_custom_interfaces::srv::StateMachine::Request::SharedPtr req,
        bot_custom_interfaces::srv::StateMachine::Response::SharedPtr res);
  
    // Load parameters and node graph definitions.
    bool load_parameters();

    // Create ROS communications (publishers, timers, clients) after configure.
    void create_comms();

    // Periodic update hook that advances the internal state machine.
    void update_callback();

    // Publishes the aggregated status array at a fixed rate.
    void publish_callback();

    // Process pending events from nodes or the controller.
    void event_callback();
};

#endif // LIFECYCLEMANAGER.HPP 
