#include "lifecycle_manager.hpp"
#include "state_controller.hpp" 

LifecycleManager::LifecycleManager() : 
    LifecycleNode("state_machine_node", 
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{   
    print_info("Lifecycle node constructed.");
}

LifecycleManager::~LifecycleManager()
{
    if (controller_) controller_->stop_controller();
    print_info("State machine lifecycle node destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if(!load_parameters())
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Build controller with freshly loaded graph.
    controller_ = std::make_shared<StateController>(nodes_, this);  

    cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    status_pub_ = this->create_publisher<bot_custom_interfaces::msg::StatusArray>("state_machine/status", 1);

    print_info("Node configured successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    create_comms();

    // Periodically publish aggregated state to observers.
    publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&LifecycleManager::publish_callback, this),cbg_);
    
    state_machine_srv_ = this->create_service<bot_custom_interfaces::srv::StateMachine>(
        "state_machine/command",
        std::bind(&LifecycleManager::command_srv_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        cbg_   
    );

    if(controller_) controller_->start_controller();

    status_pub_->on_activate();

    print_info("Node activated successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    status_pub_->on_deactivate();

    if (controller_) controller_->stop_controller();
    
    print_info("Node deactivated successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (controller_) controller_->stop_controller();

    if (publisher_timer_) {
        publisher_timer_->cancel();
        publisher_timer_.reset();
    }

    transition_subs_.clear();
    change_state_srvs_.clear();
    get_state_srvs_.clear();
    nodes_.clear();
    status_pub_.reset();
  
    print_info("Node cleaned up successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleManager::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool LifecycleManager::set_state(const std::string& node_name, const uint8_t state)
{
    auto it = change_state_srvs_.find(node_name);

    if (it == change_state_srvs_.end() || !it->second) 
    {
        return false;
    }
    
    auto cli = it->second;
    
    if (!cli->wait_for_service(wait_service_timeout)) 
    {
        print_error("[" + node_name + "] service " + cli->get_service_name() + " not available.");
        return false;
    }

    print_info("[" + node_name + "] change_state service available.");

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = state;

    auto future = cli->async_send_request(req);

    if (future.wait_for(wait_answer_timeout) != std::future_status::ready) 
    {
        print_error("[" + node_name + "] change_state request timed out.");
        return false;           
      }

    const bool result = future.get()->success;

    if (!result)
    {
        print_error("[" + node_name + "] change_state(" + std::to_string(state) + ") -> FAIL");
    }
    else
    {
        print_info("[" + node_name + "] change_state(" + std::to_string(state) + ") -> OK");
    }

    return result;
}

std::optional<lifecycle_msgs::msg::State> LifecycleManager::get_state(const std::string& node_name)
{
    auto it = get_state_srvs_.find(node_name);

    if (it == get_state_srvs_.end() || !it->second) return std::nullopt;

    auto cli = it->second;

    if (!cli->wait_for_service(wait_service_timeout))  return std::nullopt;
    
    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto future = cli->async_send_request(req);

    if (future.wait_for(wait_service_timeout) != std::future_status::ready) {
        return std::nullopt;
    }

    const auto resp = future.get();

    return resp->current_state;
}

void LifecycleManager::command_srv_callback(
    const bot_custom_interfaces::srv::StateMachine::Request::SharedPtr req,
    bot_custom_interfaces::srv::StateMachine::Response::SharedPtr res)
{
    if (!controller_)
    {
        res->result = static_cast<int32_t>(CommandResponse::FAILURE);
        res->success = false;
        return;
    }

    print_info("Received state machine command.");
    
    auto [code, ok] = controller_->command_manager(req->node, static_cast<Command>(req->command));
    res->result  = static_cast<int32_t>(code);
    res->success = ok;
}

void LifecycleManager::transition_callback(const std::string& node_name,
    const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    const uint8_t goal = msg->goal_state.id;
    if (controller_)
        controller_->add_event(PendingEvent{node_name, goal});
}

bool LifecycleManager::load_parameters()
{
    std::string json_path;
    if (!this->get_parameter("nodes_json_path", json_path) || json_path.empty())
    {
        print_error("Parameter 'nodes_json_path' not set or empty. "
                    "Set it via --ros-args -p nodes_json_path:=/path/to/nodes.json");
        return false;
    }

    GraphNode graph(json_path);
    auto [nodes, ok] = graph.load_parameters();
    if (!ok)
    {
        print_error("Failed to load node graph from JSON: " + json_path);
        return false;
    } 

    nodes_ = std::move(nodes);
    return true;
}

void LifecycleManager::create_comms()
{
    // Fresh rebuild of per-node subscriptions/clients.
    transition_subs_.clear();
    change_state_srvs_.clear();
    get_state_srvs_.clear();

    for (const auto &n : nodes_) 
    {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cbg_;
    
        // Watch lifecycle transition events for this node.
        auto sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          "/" + n.name + "/transition_event",
          rclcpp::SystemDefaultsQoS(),
          [this, node_name = n.name](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
            this->transition_callback(node_name, msg);
          },
          sub_opts
        );
        transition_subs_.emplace(n.name, std::move(sub));

        // Service clients used to push lifecycle commands and query status.
        auto change_cli = this->create_client<lifecycle_msgs::srv::ChangeState>
            ("/" + n.name + "/change_state",
            rmw_qos_profile_services_default, 
            cbg_);
        change_state_srvs_.emplace(n.name, change_cli);

        auto get_cli = this->create_client<lifecycle_msgs::srv::GetState>
            ("/" + n.name + "/get_state",
            rmw_qos_profile_services_default, 
            cbg_);
        get_state_srvs_.emplace(n.name, get_cli);
    }

    print_info("Watching " + std::to_string(transition_subs_.size()) + " transition_event topics.");
}

void LifecycleManager::update_callback()
{
  if (controller_) controller_->update();
}

void LifecycleManager::publish_callback()
{
    if (!status_pub_) return;

    bot_custom_interfaces::msg::StatusArray msg;
    msg.header.stamp = this->now();

    for (const auto &n : nodes_)
    {
        bot_custom_interfaces::msg::Status item;
        item.name = n.name;
        item.display_name = n.display_name;

        auto st = get_state(n.name);  // poll current lifecycle state
        if(!st)
        {
            item.status = "ERROR";
        }
        else
        {
            item.status = st->label;
        }

        msg.containers.push_back(item);
    }

    status_pub_->publish(msg);
}

void LifecycleManager::event_callback()
{
  if (controller_) controller_->event_manager();
}

void LifecycleManager::print_info(const std::string& s) 
{
    RCLCPP_INFO(this->get_logger(), "[LifecycleManager] %s", s.c_str());
}

void LifecycleManager::print_error(const std::string& s) 
{
    RCLCPP_ERROR(this->get_logger(), "[LifecycleManager] %s", s.c_str());
}
