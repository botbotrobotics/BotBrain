#pragma once

#include <mutex>   
#include <atomic>   
#include <memory>   
#include <string>   
#include <vector>   

#include "g1_driver.hpp"

// ROS 2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ROS 2 messages and services
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <bot_custom_interfaces/srv/mode.hpp>
#include <bot_custom_interfaces/msg/names.hpp>
#include <bot_custom_interfaces/srv/arm_cmd.hpp>
#include <bot_custom_interfaces/srv/current_mode.hpp>

// Ament utilities
#include "ament_index_cpp/get_package_share_directory.hpp"

/**
 * @brief Lifecycle node that sends commands to the G1 robot.
 * - Controls locomotion (cmd_vel, FSM mode, emergency stop).
 * - Controls upper body poses (save, load, go to pose, release).
 * - Exposes ROS services and topics for external control.
 */
class G1Write : public rclcpp_lifecycle::LifecycleNode
{
private:

    // Locomotion configuration parameters
    int speed_mode_{0}; // speed profile index
    bool keep_move_{false}; // // continuous gait flag

    // G1 driver instance
    std::shared_ptr<G1Driver> g1_driver_; 

    // Callback group and subscription options (reentrant)
    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::SubscriptionOptions sub_opts_;

    // Emergency stop flag (shared between callbacks)
    std::atomic<bool> emergency_flag_{false};

    // Mutex for file operations (poses file)
    std::mutex poses_file_mutex_;
    
    // ROS publisher
    rclcpp_lifecycle::LifecyclePublisher<bot_custom_interfaces::msg::Names>::SharedPtr 
        pose_names_pub_;
    
    // ROS subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_emergency_stop_srv_;

    // ROS services
    rclcpp::Service<bot_custom_interfaces::srv::Mode>::SharedPtr set_mode_srv_;
    rclcpp::Service<bot_custom_interfaces::srv::ArmCmd>::SharedPtr set_arm_cmd_srv_;
    rclcpp::Service<bot_custom_interfaces::srv::CurrentMode>::SharedPtr get_current_mode_srv_;

    // ROS timer 
    rclcpp::TimerBase::SharedPtr arm_pose_timer_;

public:
    // Constructor.
    G1Write();

    // Destructor.
    ~G1Write();

    // Configure node (parameters, driver, publishers).
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;

    // Activate publishers, subscribers, services, timers.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;

    // Deactivate interfaces and stop motion.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // Clean up resources (services, pubs, driver).
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    // Shutdown node and stop robot safely.
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    // cmd_vel subscriber callback (velocity commands).
    void cmd_vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Set locomotion mode (FSM, start, etc.).
    void set_mode_callback(
        const std::shared_ptr<bot_custom_interfaces::srv::Mode::Request> request,
        std::shared_ptr<bot_custom_interfaces::srv::Mode::Response> response);

    // Set locomotion mode (FSM, start, etc.).
    void get_current_mode_callback(
        const std::shared_ptr<bot_custom_interfaces::srv::CurrentMode::Request> request,
        std::shared_ptr<bot_custom_interfaces::srv::CurrentMode::Response> response);

    // Set emergency stop.
    void set_emergency_stop_callback(
        const std::shared_ptr<std_srvs::srv:: SetBool::Request> request,
        std::shared_ptr<std_srvs::srv:: SetBool::Response> response);

    // Handle arm commands (save, go to, release, delete pose).
    void set_arm_cmd_callback(
        const std::shared_ptr<bot_custom_interfaces::srv::ArmCmd::Request> request,
        std::shared_ptr<bot_custom_interfaces::srv::ArmCmd::Response> response);

private:
    // Publish list of stored pose names on topic.
    void publish_pose_names();

    // ─────────────── Utils functions ───────────────

    // Load node parameters from ROS param server.
    void load_parameters();

    // Get full path of the poses file.
    std::string get_package_dir();

    // Convert FsmId to string.
    std::string fsm_id2string(FsmId fsm);

    // // Convert string to FsmId.
    static FsmId string2fsm_id(std::string s); 

    // Read all pose names from file.
    std::vector<std::string> get_pose_names(); 
    
    // Delete a pose from file by name.
    bool delete_pose_from_file(const std::string & name);

    // Save an upper body pose to file.
    bool save_pose_to_file(const std::string & name, const UpperBodyPose & pose);

    // Load pose data from file by name.
    bool get_pose_from_file(const std::string & name, UpperBodyPose & out_pose);
};