#include <cctype>     
#include <thread>      
#include <chrono>   
#include <fstream>     
#include <sstream>   
#include <algorithm>   

#include "g1_write.hpp"

G1Write::G1Write() : LifecycleNode("robot_write_node", rclcpp::NodeOptions())
{
    RCLCPP_INFO(this->get_logger(), "Node Created");
}

G1Write::~G1Write()
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "Destroying G1Write...");

        // Stop robot motion if the driver is still available
        if (g1_driver_) {(void)g1_driver_->stop_move();}

        // Explicitly reset ROS entities
        if (cmd_vel_subscription_) { cmd_vel_subscription_.reset(); }
        if (cmd_vel_subscription_){cmd_vel_subscription_.reset();}
        if (set_mode_srv_){set_mode_srv_.reset();}
        if (get_current_mode_srv_){get_current_mode_srv_.reset();}
        if (set_emergency_stop_srv_){set_emergency_stop_srv_.reset();}
        if (set_arm_cmd_srv_){set_arm_cmd_srv_.reset();}
        if (arm_pose_timer_){
            arm_pose_timer_->cancel(); 
            arm_pose_timer_.reset();
        }

        if (cbg_) cbg_.reset();
        if (g1_driver_) g1_driver_.reset();

        RCLCPP_INFO(this->get_logger(), "[G1Write] Destructor: node destroyed cleanly");
    }
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] Exception in ~G1Write(): %s", e.what());
    } 
    catch (...) 
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] Unknown exception in ~G1Write()");
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1Write::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(this->get_logger(), "[G1Write] on_configure(): starting configuration");

    try
    {
        // Declare and load parameters (only allowed in configure)
        load_parameters();
        RCLCPP_INFO(this->get_logger(), "[G1Write] Parameters loaded successfully ");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] Failed to load parameters: %s", e.what());
    }

    // Create driver instance 
    g1_driver_ = std::make_shared<G1Driver>();

    // Initialize locomotion client
    if(!g1_driver_->init_loco_client(speed_mode_, keep_move_))
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] G1Driver locomotion init FAILED");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[G1Write] G1Driver locomotion initialized successfully");
    }

    // Initialize low-level arm control
    if(!g1_driver_->init_low_level())
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] G1Driver low-level init FAILED"); 
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[G1Write] G1Driver low-level initialized successfully");
    }

    pose_names_pub_ = this->create_publisher<bot_custom_interfaces::msg::Names>
        ("pose/names", 1);

    // Reentrant callback group
    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Attach callback group to subscription options
    sub_opts_.callback_group = cbg_; 

    RCLCPP_INFO(this->get_logger(), "[G1Write] G1Driver low-level initialized successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1Write::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(this->get_logger(), "[G1Write] on_activate(): activating node"); 

    pose_names_pub_->on_activate();

    // Subscriber for velocity commands
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_out", rclcpp::QoS(10), std::bind(&G1Write::cmd_vel_subscription_callback, 
            this, std::placeholders::_1), sub_opts_);

    // Services for mode control
    set_mode_srv_ = this->create_service<bot_custom_interfaces::srv::Mode>(
        "mode", std::bind(&G1Write::set_mode_callback, this, std::placeholders::_1, 
            std::placeholders::_2), rmw_qos_profile_services_default, cbg_);

    // Services for current mode
    get_current_mode_srv_ = this->create_service<bot_custom_interfaces::srv::CurrentMode>(
        "current_mode", std::bind(&G1Write::get_current_mode_callback, this, std::placeholders::_1, 
            std::placeholders::_2), rmw_qos_profile_services_default, cbg_);

    // Services for emergency stop
    set_emergency_stop_srv_ = this->create_service<std_srvs::srv:: SetBool>(
        "emergency_stop", std::bind(&G1Write:: set_emergency_stop_callback, this, std::placeholders::_1, 
            std::placeholders::_2), rmw_qos_profile_services_default, cbg_);
    
    // Services for arm commands
    set_arm_cmd_srv_ = this->create_service<bot_custom_interfaces::srv::ArmCmd>(
        "arm_cmd", std::bind(&G1Write::set_arm_cmd_callback, this, std::placeholders::_1, 
            std::placeholders::_2), rmw_qos_profile_services_default, cbg_);

    // Periodic timer to publish available arm pose names
    arm_pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), 
        std::bind(&G1Write::publish_pose_names, this), cbg_);
 
    RCLCPP_INFO(this->get_logger(), "[G1Write] Node ACTIVATED successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1Write::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(this->get_logger(), "[G1Write] on_deactivate(): deactivating node"); 

    pose_names_pub_->on_deactivate();

    // Tear down activated entities
    if (set_mode_srv_) {set_mode_srv_.reset();}
    if (get_current_mode_srv_) {get_current_mode_srv_.reset();}
    if (set_emergency_stop_srv_) {set_emergency_stop_srv_.reset();}
    if (cmd_vel_subscription_) { cmd_vel_subscription_.reset(); }
    if (set_arm_cmd_srv_) {set_arm_cmd_srv_.reset();}
    if (arm_pose_timer_)
    {
        arm_pose_timer_->cancel(); 
        arm_pose_timer_.reset();
    }

    if (g1_driver_) {g1_driver_->stop_move();}

    RCLCPP_INFO(this->get_logger(), "[G1Write] Node DEACTIVATED successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1Write::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(this->get_logger(), "[G1Write] on_cleanup(): releasing configured resources"); 

    // Defensive cleanup
    set_mode_srv_.reset();
    get_current_mode_srv_.reset();
    set_emergency_stop_srv_.reset();
    set_arm_cmd_srv_.reset();

    // Release driver and callback group
    if (arm_pose_timer_)
    {
        arm_pose_timer_->cancel(); 
        arm_pose_timer_.reset();
    } 

    // Publisher cleanup
    pose_names_pub_.reset();

    // Release driver and callback group
    g1_driver_.reset();
    cbg_.reset();

    RCLCPP_INFO(this->get_logger(), "[G1Write] Node CLEANED UP successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1Write::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(this->get_logger(), "[G1Write] on_shutdown(): stopping node"); 

    pose_names_pub_.reset();

    if (g1_driver_) {
        g1_driver_->stop_move();
        g1_driver_.reset();
    }

    if (set_mode_srv_) {set_mode_srv_.reset();}
    if (get_current_mode_srv_) {get_current_mode_srv_.reset();}
    if (set_emergency_stop_srv_) {set_emergency_stop_srv_.reset();}
    if (set_arm_cmd_srv_) {set_arm_cmd_srv_.reset();}

    if (arm_pose_timer_)
    {
        arm_pose_timer_->cancel(); 
        arm_pose_timer_.reset();
    }

    cbg_.reset();

    RCLCPP_INFO(this->get_logger(), "[G1Write] Node SHUTDOWN completed");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void G1Write::cmd_vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Ignore velocity commands while in emergency stop condition 
    if(emergency_flag_.load()) return;

    auto saturate = [](double value, double limit) -> double 
    {
        if (value > limit)  return limit;
        if (value < -limit) return -limit;
        return value;
    };

    // Saturate incoming commands to safe limits
    double vx = saturate(msg->linear.x, 0.6);
    double vy = saturate(msg->linear.y, 0.6);
    double wz = saturate(msg->angular.z, 1.0);

    if (g1_driver_) 
    {
        g1_driver_->move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz),  
            false);   
    }
}

void G1Write::set_mode_callback(
    const std::shared_ptr<bot_custom_interfaces::srv::Mode::Request> request,
    std::shared_ptr<bot_custom_interfaces::srv::Mode::Response> response)
{

    if(emergency_flag_.load())
    {
        response->success = false;
        response->message = "Emergency is active; mode change is blocked";
        return;
    }

    if(request->mode == "start")
    {
        if(g1_driver_->start() != 0)
        {
            response->success = false;
            response->message = "Failed to start locomotion";
            return;  
        }
        else
        {
            response->success = true;
            response->message = "Locomotion started successfully";
            return;  
        }
    }

    FsmId target_id = string2fsm_id(request->mode);

    if(target_id == FsmId::ERROR)
    {
        RCLCPP_WARN(this->get_logger(),"[G1Write] set_mode_callback(): invalid mode '%s'", 
            request->mode.c_str());

        response->success = false;
        response->message = "Invalid mode";
        return;
    }

    auto rs = g1_driver_->set_fsm_id(static_cast<int>(target_id));
    if(rs != 0)
    {
        RCLCPP_WARN(this->get_logger(), "[G1Write] Failed to set mode '%s' (rc=%d)",  
            request->mode.c_str(), rs); 

        response->success = false;
        response->message = "Failed to set mode";
        return;  
    }

    const auto timeout = std::chrono::seconds(3);
    const auto start_time = std::chrono::steady_clock::now();

    int current_id = -1;
    bool success = false;

    // Wait for FSM to actually switch to requested mode (polling)
    while (std::chrono::steady_clock::now() - start_time < timeout)
    {
        auto rg = g1_driver_->get_fsm_id(current_id);
        if(rg != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (current_id == static_cast<int>(target_id))
        {
            success = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "[G1Write] FSM mode changed to '%s' successfully", 
            request->mode.c_str());    

        response->success = true;
        response->message = "Mode changed successfully"; 
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "[G1Write] Timeout waiting for FSM mode '%s'",   
            request->mode.c_str());

        response->success = false;
        response->message = "Timeout waiting for FSM mode";
    }
}

void G1Write::get_current_mode_callback(
    const std::shared_ptr<bot_custom_interfaces::srv::CurrentMode::Request> request,
    std::shared_ptr<bot_custom_interfaces::srv::CurrentMode::Response> response)
{
    (void) request;
    
    int current_id = -1;
    const int rc = g1_driver_->get_fsm_id(current_id);

    if(rc != 0)
    {
        RCLCPP_WARN(this->get_logger(), 
            "[G1Write] get_current_mode_callback(): failed to get FSM ID"); 

        response->mode = "unknown";
        return;  
    }

    const std::string mode_str = fsm_id2string(static_cast<FsmId>(current_id));

    response->mode = mode_str;
}

void G1Write::set_emergency_stop_callback(
        const std::shared_ptr<std_srvs::srv:: SetBool::Request> /*request*/,
        std::shared_ptr<std_srvs::srv:: SetBool::Response> response)
{
    if(emergency_flag_.load())
    {
        emergency_flag_.store(false);
        RCLCPP_INFO(get_logger(), "[G1Write] Emergency stop flag reset by user request");
        response->success = true;
        return;
    }

    emergency_flag_.store(true);

    if (!g1_driver_) 
    {
        RCLCPP_WARN(get_logger(), "[G1Write] Emergency stop requested but driver is null"); 

        response->success = false;
        emergency_flag_.store(false);
        return;
    }

    // 1) Stop current motion
    const int rc_stop = g1_driver_->stop_move();
    if (rc_stop != 0) 
    {
        RCLCPP_WARN(get_logger(), "[G1Write] stop_move() failed during emergency");

        response->success = false;
        emergency_flag_.store(false);
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 2) Move to a safe squat pose + damp mode
    const int rc_bal = g1_driver_->set_fsm_id(static_cast<int>
        (FsmId::BALANCE_SQUAT_SQUAT_STAND));

    if (rc_bal != 0) 
    {
        RCLCPP_WARN(get_logger(), "[G1Write] set_fsm_id(BALANCE_SQUAT_SQUAT_STAND) failed");
            
        response->success = false;
        emergency_flag_.store(false);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[G1Write] Emergency stop sequence completed successfully");
       
    response->success = true;
    return;
}

void G1Write::set_arm_cmd_callback(
    const std::shared_ptr<bot_custom_interfaces::srv::ArmCmd::Request> request,
    std::shared_ptr<bot_custom_interfaces::srv::ArmCmd::Response> response)
{
    response->success = false;
    response->message = "unknown error";

    const ArmCommand cmd = static_cast<ArmCommand>(request->command);
    const std::string & pose_name = request->name;

    switch (cmd)
    {
        case ArmCommand::SAVE_POSE:
        {
            UpperBodyPose pose;
            if (!g1_driver_->get_upper_body_pose(pose)) 
            {
                response->message = "Failed to get current upper body pose";
                break;
            }

            if (!save_pose_to_file(pose_name, pose)) 
            {
                response->message = "Failed to save pose file '" + pose_name + "'";
                break;
            }

            response->success = true;
            response->message = "Pose saved: " + pose_name; 
            break;
        }

        case ArmCommand::GO_TO_POSE:
        {
            UpperBodyPose pose;
            if(!get_pose_from_file(pose_name, pose))
            {
                response->message = "Failed to load pose from file";
                break;
            }

            if(!g1_driver_->set_arm_joints(pose))
            {
                response->message = "Failed to move arm to pose"; 
                break;
            }

            response->success = true;
            response->message = "New pose applied";
            break;
        }

        case ArmCommand::RELEASE_POSE:
        {
            if(!g1_driver_->release_arm_joints())
            {
                response->message = "Failed to release arm joint position";
                break;
            }

            response->success = true;
            response->message = "Arm joint position released";
            break;
        }

        case ArmCommand::DELETE_POSE:
        {
            if(!delete_pose_from_file(pose_name))
            {
                response->message = "Failed to delete pose '" + pose_name + "'";
                break;
            }
    
            response->success = true;
            response->message = "Pose deleted: " + pose_name;
            break;
        }

        default:
        {
            response->message = "Unknown arm command";
            break;
        }
    }
}

void G1Write::load_parameters()
{
    keep_move_ = this->declare_parameter<bool>("robot.keep_move", false);
    speed_mode_ = this->declare_parameter<int>("robot.speed_mode", 0);

    RCLCPP_INFO(this->get_logger(), "[G1Write] Parameters: keep_move=%s | speed_mode=%d", 
        keep_move_ ? "true" : "false", speed_mode_);
}

std::string G1Write::get_package_dir()
{
    // Cache path to poses file 
    static std::string poses_file;

    if (poses_file.empty())
    {
        #ifdef G1_PKG_SOURCE_DIR
                const std::string pkg_src_dir = G1_PKG_SOURCE_DIR;
                poses_file = pkg_src_dir + "/config/arm_poses.txt";
        #endif
    }

    return poses_file;
}

std::string G1Write::fsm_id2string(FsmId fsm)
{
    if(fsm == FsmId::ZERO_TORQUE) {return "zero_torque";}
    else if(fsm == FsmId::DAMPING) {return "damp";}
    else if(fsm == FsmId::LOCK_STAND) {return "preparation";}
    else if(fsm == FsmId::RUN) {return "run";}
    else if(fsm == FsmId::BALANCE_SQUAT_SQUAT_STAND) {return "squat";}
    else{return "unknown";}
}

FsmId G1Write::string2fsm_id(std::string s)
{
    if(s == "zero_torque") {return FsmId::ZERO_TORQUE;}
    else if(s == "damp") {return FsmId::DAMPING;}
    else if(s == "preparation") {return FsmId::LOCK_STAND;}
    else if(s == "run") {return FsmId::RUN;}
    else if(s == "squat") {return FsmId::BALANCE_SQUAT_SQUAT_STAND;}
    else {return FsmId::ERROR;}
}

std::vector<std::string> G1Write::get_pose_names()
{
    std::vector<std::string> names;

    const std::string path = get_package_dir();
    if (path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] get_pose_names(): failed to get poses file path");
        return names;
    }

    std::lock_guard<std::mutex> lock(poses_file_mutex_);

    std::ifstream in(path);
    if (!in.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] get_pose_names(): failed to open poses file");
        return names;
    }

    std::string line;
    while (std::getline(in, line))
    {
        auto is_space = [](unsigned char ch)
        { 
            return std::isspace(ch); 
        };

        // Trim leading whitespace
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [&](char c)
            {return !is_space(c); }));

        // Trim trailing whitespace
        line.erase(std::find_if(line.rbegin(), line.rend(), [&](char c)
            {return !is_space(c); }).base(),line.end());

        if (line.empty()) continue;

        if (line[0] == '#') continue;

        std::istringstream iss(line);
        std::string name;
        if (!(iss >> name)) continue;

        names.push_back(name);
    }

    return names;
}

bool G1Write::delete_pose_from_file(const std::string & name)
{
    if (name.empty()) return false;
    
    const std::string path = get_package_dir();
    if (path.empty()) return false;

    std::lock_guard<std::mutex> lock(poses_file_mutex_);

    std::ifstream in(path);
    if (!in.is_open()) return false;
    
    std::vector<std::string> new_lines;
    std::string line;
    bool removed = false;

    while (std::getline(in, line))
    {
        std::string trimmed = line;

        auto is_space = [](unsigned char ch){ return std::isspace(ch); };

        trimmed.erase(trimmed.begin(), std::find_if(trimmed.begin(), trimmed.end(),[&](char c)
            {return !is_space(c); }));
        trimmed.erase(std::find_if(trimmed.rbegin(), trimmed.rend(), [&](char c)
            {return !is_space(c); }).base(),trimmed.end());

        if (trimmed.empty() || trimmed[0] == '#')
        {
            new_lines.push_back(line);
            continue;
        }

        std::istringstream iss(trimmed);
        std::string pose_name_in_file;

        if (!(iss >> pose_name_in_file))
        {
            new_lines.push_back(line);
            continue;
        }

        if (pose_name_in_file == name)
        {
            removed = true;
            continue;
        }

        new_lines.push_back(line);
    }

    in.close();

    if (!removed) return false;

    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) return false;

    for (const auto & l : new_lines)
    {
        out << l << "\n";
    }

    out.close();

    return true;
}

bool G1Write::save_pose_to_file(const std::string & name, const UpperBodyPose & pose)
{
    if (name.empty()) 
    {
        RCLCPP_ERROR(this->get_logger(), "[G1Write] save_pose_to_file(): pose name is empty");
        return false;
    }

    const std::string path = get_package_dir();

    if(path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), 
            "[G1Write] save_pose_to_file(): failed to get poses file path");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(poses_file_mutex_);

        std::ofstream out(path, std::ios::app); 
        if (!out.is_open()) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "[G1Write] save_pose_to_file(): failed to open poses file");
            return false;
        }

        out << name << " "
            << pose.waist_yaw << " "
            << pose.left.shoulder_pitch << " "
            << pose.left.shoulder_roll  << " "
            << pose.left.shoulder_yaw   << " "
            << pose.left.elbow          << " "
            << pose.left.wrist_roll     << " "

            << pose.right.shoulder_pitch << " "
            << pose.right.shoulder_roll  << " "
            << pose.right.shoulder_yaw   << " "
            << pose.right.elbow          << " "
            << pose.right.wrist_roll     << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "[G1Write] Pose '%s' saved to '%s'", 
        name.c_str(), path.c_str());
    return true;
}

void G1Write::publish_pose_names()
{
    if(!pose_names_pub_) return;

    auto names = get_pose_names();

    bot_custom_interfaces::msg::Names msg;
    msg.names = names;

    pose_names_pub_->publish(msg);
}

bool G1Write::get_pose_from_file(const std::string & name, UpperBodyPose & out_pose)
{
    if (name.empty()) return false;

    const std::string path = get_package_dir();
    if (path.empty()) return false;

    std::lock_guard<std::mutex> lock(poses_file_mutex_);

    std::ifstream in(path);
    if (!in.is_open()) return false;

    std::string line;
    while (std::getline(in, line))
    {
        std::string trimmed = line;

        auto is_space = [](unsigned char ch){ return std::isspace(ch); };
        trimmed.erase(trimmed.begin(), std::find_if(trimmed.begin(), trimmed.end(), [&](char c)
            {return !is_space(c); }));
        trimmed.erase(std::find_if(trimmed.rbegin(), trimmed.rend(), [&](char c)
            {return !is_space(c); }).base(), trimmed.end());

        if (trimmed.empty() || trimmed[0] == '#')
            continue;

        std::istringstream iss(trimmed);

        std::string pose_name_in_file;
        if (!(iss >> pose_name_in_file))  continue;

        if (pose_name_in_file != name) continue;

       if (!(iss >> out_pose.waist_yaw
          >> out_pose.left.shoulder_pitch
          >> out_pose.left.shoulder_roll
          >> out_pose.left.shoulder_yaw
          >> out_pose.left.elbow
          >> out_pose.left.wrist_roll
          >> out_pose.right.shoulder_pitch
          >> out_pose.right.shoulder_roll
          >> out_pose.right.shoulder_yaw
          >> out_pose.right.elbow
          >> out_pose.right.wrist_roll))
        {
            RCLCPP_ERROR(this->get_logger(), "[G1Write] get_pose_from_file(): missing numeric values");
            return false;
        }

        return true;  
    }

    RCLCPP_WARN(this->get_logger(), "[G1Write] get_pose_from_file(): pose name does not exist");

    return false;
}