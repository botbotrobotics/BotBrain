#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Number of lifecycle transition retries before giving up on a node.
#define attempts 2

// Common wait policies used across the lifecycle manager services.
#define wait_answer_timeout std::chrono::seconds(30)
#define wait_service_timeout std::chrono::milliseconds(500)

// Global state of the controller FSM.
enum class State : uint8_t
{
    BRINGUP,
    BRINGDOWN,
    IDLE,
    AUTONOMOUS,
    TELEOP,
    ERROR,
    RESTARTING
};

// Commands exposed to external callers.
enum class Command : uint8_t
{
    ACTIVATE_NODE = 1,
    DEACTIVATE_NODE,
    RESTART_NODE,
};

enum class CommandResponse : uint8_t
{
    SUCCESS = 1,
    FAILURE,
    INVALID_NODE,
    INVALID_COMMAND,
    INVALID_STATE,
    SKIPPED
};

// High-level classification of nodes so we can orchestrate them by role.
enum class NodeClasse : uint8_t
{
    CORE = 0,
    CAMERA,
    NAVIGATION,
    ACCESSORIES,
    AUDIO,
    IA_STACK,
    PAYLOAD,
    UNKNOWN,
};

// Priority levels that influence shutdown and error reporting.
enum class ClassePriority : uint8_t
{
    TERMINAL,
    CRITICAL,
    WARNING,
    UNKNOWN
};

// Lifecycle states mirrored from rclcpp_lifecycle definitions.
enum class NodeState : uint8_t
{
    PRIMARY_STATE_UNKNOWN,	   
    PRIMARY_STATE_UNCONFIGURED,	
    PRIMARY_STATE_INACTIVE,	    
    PRIMARY_STATE_ACTIVE,	    
    PRIMARY_STATE_FINALIZED,    
};

// Configuration metadata describing each lifecycle-managed node.
struct NodeProfile
{
    std::string name;
    std::string display_name;
    NodeClasse classe{NodeClasse::CORE};
    int order{0};
    ClassePriority priority{ClassePriority::TERMINAL};
    std::vector<std::string> dependencies; 
    std::vector<std::string> dependents_of;
};

// Event queued by the lifecycle manager when a node changes state unexpectedly.
struct PendingEvent
{
    std::string node_name;
    uint8_t goal_state_id; 
};
