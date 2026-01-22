#pragma once

#include <deque>
#include <mutex>
#include <tuple>
#include <deque>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <optional>
#include <unordered_set>
#include <condition_variable>  

#include "types.hpp"
#include "graph_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

class LifecycleManager; 

/**
 * @brief Drives lifecycle transitions for all nodes.
 * - Executes bring-up/bring-down waves respecting dependencies.
 * - Dispatches external commands and queued transition events.
 * - Reports aggregated state back to the lifecycle manager.
 */
class StateController 
{
private:
    // Cached graph of nodes with dependency metadata.
    std::vector<NodeProfile> nodes_;

    // Owning lifecycle manager.
    LifecycleManager* lifecycle_;  

    // Pending lifecycle events reported by ROS callbacks.
    std::deque<PendingEvent> events_;

    // High-level commands queued by the service interface.
    std::deque<std::tuple<std::string,int>> commands_;

    // Nodes temporarily shielded from transitions.
    std::unordered_set<std::string> protected_nodes_;  

    // Protects command queue access.
    std::mutex cmd_mtx_;

    // Protects event queue access.
    std::mutex event_mtx_;

    // Guards protected_nodes_ mutations.
    std::mutex protected_mtx_;  

    // Signals event loop when new events arrive.
    std::condition_variable cv_event_;

    // Flag indicating worker threads should keep running.
    std::atomic_bool run_{false};

    // Indicates there is at least one pending command to process.
    std::atomic_bool has_command_{false};  

    // Aggregate controller state.
    std::atomic<State> state_{State::IDLE};

    // When true, event processing is temporarily blocked.
    std::atomic<bool> events_blocked_{false};

    // True when a restart sequence is queued.
    std::atomic<bool> restart_pending_{false};

    // Prevents multiple start attempts from racing.
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;
    
    // Thread executing update_loop().
    std::thread update_th_;

    // Thread executing events_loop().
    std::thread events_th_;

    // Sleep duration between update ticks.
    std::chrono::milliseconds update_period_{20};

    // Sleep duration between event-loop iterations.
    std::chrono::milliseconds events_period_{20};  

public:
    // Build controller with parsed nodes and lifecycle owner.
    StateController(const std::vector<NodeProfile> nodes, LifecycleManager* lifecycle);

    // Destructor.
    ~StateController();
    
    // Stop worker threads and flush queues.
    void stop_controller();

    // Start worker threads and unblock event loop.
    void start_controller();
 
    // Single control tick triggered by LifecycleManager.
    void update();

    // Return aggregated FSM state for reporting.
    State get_state();

    // Drain queued transition events.
    void event_manager();

    // Add transition event from lifecycle callbacks.
    void add_event(PendingEvent event);

    // Request lifecycle transition on a specific node.
    bool set_state(const std::string& name, uint8_t transition); 

    // Handle high-level commands.
    std::tuple<CommandResponse,bool> command_manager(std::string& name, Command cmd);

    // Fetch raw lifecycle state via ROS service.
    std::optional<lifecycle_msgs::msg::State> get_node_state(const std::string& node_name);
    
private:
    // Orchestrate multi-stage bring-up based on dependency graph.
    void bring_up();

    // Drive graceful tear-down when shutting down or reconfiguring.
    void bring_down();

    // Restart sequence invoked by restart commands.
    void restarting();

    // Continuous update loop (runs in its own thread).
    void update_loop();

    // Background loop that drains pending lifecycle events.
    void events_loop();
    
    // Activate a single node through its lifecycle transitions.
    bool node_activate(const NodeProfile& n);

    // Deactivate a single node through its lifecycle transitions.
    bool node_deactivate(const NodeProfile& n);

    // Verify dependencies before issuing a transition.
    bool check_dependencies(const NodeProfile& n, uint8_t transition);

    // Depth-first dependency validation used internally.
    bool check_dependencies(const NodeProfile& n, uint8_t transition,
                            std::unordered_set<std::string>& visited);
                            
    // Sleep policy used to avoid hammering the lifecycle services.
    void sleep_policy(const std::string& name);

    // Cheap lookup to ensure a dependency exists in the graph.
    bool node_exist(const std::string& dep_name);

    // Build the protected set when skipping nodes during operations.
    void build_protected_nodes(const std::string& name);
}; 