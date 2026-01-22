#include "state_controller.hpp"
#include "lifecycle_manager.hpp"

StateController::StateController(const std::vector<NodeProfile> nodes,  LifecycleManager* lifecycle)
    : nodes_(nodes), lifecycle_(lifecycle)
{ 
    state_ = State::BRINGUP;
    lifecycle_->print_info("State Controller created");
}

StateController::~StateController(){}

void StateController::update()
{
    const auto s = state_.load(std::memory_order_relaxed);
    switch (s)
    {
        case State::BRINGUP:
            if (running_.test_and_set()) return;
            bring_up();
            running_.clear();
            break;
        case State::BRINGDOWN:
            if (running_.test_and_set()) return;
            bring_down();
            running_.clear();
            break;
        case State::RESTARTING:
            if (running_.test_and_set()) return;
            restarting();
            running_.clear();
            break;
        case State::AUTONOMOUS:
            break;
        case State::ERROR:
            break;
        case State::IDLE:
            break;
        default:
            break;
    }
}

std::optional<lifecycle_msgs::msg::State> StateController::get_node_state(const std::string& node_name)
{
    if (!lifecycle_) return std::nullopt;
    return lifecycle_->get_state(node_name);
}

bool StateController::set_state(const std::string& name, uint8_t transition)
{
    if (!lifecycle_) return false;
    for (int attempt = 1; attempt <= attempts; ++attempt) 
    {   
        if (lifecycle_->set_state(name, transition)) 
        {
            return true;
        }
        if (attempt < attempts) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

void StateController::sleep_policy(const std::string& name)
{
    if (name == "d435i" || name == "d455" || name == "front_camera" || name == "back_camera") 
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    } else 
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool StateController::node_activate(const NodeProfile& n)
{
    // Drive the lifecycle node through configure/activate until it becomes active.
    lifecycle_->print_info("Activating node");
    
    auto cur = get_node_state(n.name);

    if (!cur) return false;

    if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return true;
    else if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) 
    {   
        lifecycle_->print_info("Configuring node...");
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return false;
        sleep_policy(n.name);
        lifecycle_->print_info("Activating node...");
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) return false;
    }
    else if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) return false;
    } 
    else
    { 
        return false;
    }
    lifecycle_->print_info("Node activated");
    return true;
}

bool StateController::node_deactivate(const NodeProfile& n)
{
    // Drive the node through deactivate/cleanup transitions so dependents shut down cleanly.
    auto cur = get_node_state(n.name);

    if (!cur) return false;

    if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) return true;
    else if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) 
    {
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) return false;
        sleep_policy(n.name);
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) return false;
    }
    else if (cur->id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if(!set_state(n.name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) return false;
    } 
    else
    {     
        return false;
    }
    return true;
}

bool StateController::node_exist(const std::string& dep_name)
{
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeProfile& x){ return x.name == dep_name; });
    if (it == nodes_.end()) return false;
    return true;
}

bool StateController::check_dependencies(const NodeProfile& n, uint8_t transition)
{
    // Recursively validate that dependencies or dependents satisfy the requested transition.
    lifecycle_->print_info("Checking dependencies");
    std::unordered_set<std::string> visited;
    return check_dependencies(n, transition, visited);
}

bool StateController::check_dependencies(const NodeProfile& n, uint8_t transition, std::unordered_set<std::string>& visited)
{   
    if(transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)
    {
        for (const auto& dep_name : n.dependencies)
        {
            if (!visited.insert(dep_name).second) continue;
            if(!node_exist(dep_name)) return false;

            auto it = std::find_if(nodes_.begin(), nodes_.end(),
                [&](const NodeProfile& x){ return x.name == dep_name; }
            );
            if (it == nodes_.end()) return false;

            const NodeProfile& dep = *it;

            // If the dependency is PAYLOAD we require it to already be active and never auto-activate it.
            if (dep.classe == NodeClasse::PAYLOAD) 
            {
                auto st = get_node_state(dep.name);
                if (!st || st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) 
                {
                    lifecycle_->print_error("PAYLOAD inactive: " + dep.name + " (blocking activation of " + n.name + ")");
                    return false;
                }
                continue; // Skip node_activate() for payloads
            }

            if (!check_dependencies(*it, transition, visited)) return false;
            if(!node_activate(*it)) return false;
        }
    }  
    else if (transition == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
    {
        for (const auto& dep_name : n.dependents_of) 
        {
            if (!visited.insert(dep_name).second) continue;
            if(!node_exist(dep_name)) return false;

            auto it = std::find_if(nodes_.begin(), nodes_.end(),
                [&](const NodeProfile& x){ return x.name == dep_name; });
            if (it == nodes_.end()) return false;

            if (!check_dependencies(*it, transition, visited)) return false;
            if(!node_deactivate(*it)) return false;
        }
    }
    else return true;

    return true;
}

void StateController::bring_up()
{
    // Activate nodes by class priority; fall back to teleop if navigation cannot start.
    lifecycle_->print_info("bringing up");

    bool is_nav = true;
    bool has_nav = false;

    for (const auto &n : nodes_) 
    {
        lifecycle_->print_info("Node name: " + n.name);

        if(n.classe == NodeClasse::PAYLOAD) continue;
        if(!is_nav && n.classe == NodeClasse::NAVIGATION) continue;

        auto cur = get_node_state(n.name);
        
        if(n.classe == NodeClasse::CORE)
        {   
            if (!cur || !check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) || !node_activate(n)) 
            {
                state_ = State::BRINGDOWN; 
                return;
            }
        }
        else if(n.classe == NodeClasse::NAVIGATION)
        {  
            has_nav = true;
            if (!cur || !check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) || !node_activate(n)) 
            {
                for (auto& n_nav : nodes_) // Disable every NAVIGATION node because we are falling back to teleop
                {
                    if (n_nav.classe != NodeClasse::NAVIGATION) continue;    
                    if(!check_dependencies(n_nav, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) continue;
                    node_deactivate(n_nav);
                }

                is_nav = false;

                continue;
            }
        }
        else
        {
            lifecycle_->print_info("Handling auxiliary classes");

            if (!cur) continue;

            lifecycle_->print_info("Current state: " + cur->label);

            if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) continue;

            if(!node_activate(n)) continue;
        }
    }

    if (is_nav && has_nav) 
    {
        state_.store(State::AUTONOMOUS, std::memory_order_relaxed);
        lifecycle_->print_info("Autonomous mode enabled");
    } 
    else 
    {
        state_.store(State::TELEOP, std::memory_order_relaxed);
        lifecycle_->print_info("Teleop mode enabled");
    }
    cv_event_.notify_all();

    {
        std::lock_guard<std::mutex> lk(event_mtx_);
        events_.clear();
    }

    events_blocked_.store(false, std::memory_order_relaxed);
}

void StateController::bring_down()
{
    // Tear nodes down in reverse order to respect dependency shutdown requirements.
    lifecycle_->print_info("bringing down");

    bool had_error = false;  

    for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) 
    {
        const auto &n = *it;

        if(n.classe == NodeClasse::PAYLOAD) continue;

        auto cur = get_node_state(n.name);
                
        if (!cur) 
        {
            lifecycle_->print_error("bring_down: missing state for " + n.name + " (get_state failed?)");
            had_error = true;
            continue;
        }

        if (!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) 
        {
            lifecycle_->print_error("bring_down: failed to resolve dependencies for " + n.name);
            had_error = true;
        }

        if (!node_deactivate(n)) 
        {
            lifecycle_->print_error("bring_down: failed to deactivate " + n.name);
            had_error = true;
            continue;
        } 
    }
    
    if (restart_pending_.load(std::memory_order_relaxed)) {
        restart_pending_.store(false, std::memory_order_relaxed);
        state_.store(State::BRINGUP, std::memory_order_relaxed);
    } 
    else 
    {
        state_.store(had_error ? State::ERROR : State::IDLE, std::memory_order_relaxed);
    }
}

// Guards against ingesting events while bring_down/bring_up is in progress.
std::atomic<bool> events_blocked_{false};

void StateController::add_event(PendingEvent event)
{
    auto s = state_.load(std::memory_order_relaxed);
    if (events_blocked_.load(std::memory_order_relaxed) ||
        (s != State::AUTONOMOUS && s != State::TELEOP)) 
    {
        return;
    }

    {
        std::unique_lock<std::mutex> lk(event_mtx_);
        events_.push_back(event);
    }
    cv_event_.notify_one();
}


void StateController::event_manager()
{
    // Process the next pending event once we know we are in a controllable state.
    PendingEvent ev;
    {
        std::unique_lock<std::mutex> lk(event_mtx_);
        if (events_.empty() || (state_.load() != State::AUTONOMOUS && state_.load() != State::TELEOP))
            return;
        ev = events_.front();
        events_.pop_front();
    }

    auto it = std::find_if(
        nodes_.begin(), nodes_.end(),
        [&](const NodeProfile& x){ return x.name == ev.node_name;});

    if (it == nodes_.end()) return;

    if (has_command_) 
    {
        std::lock_guard<std::mutex> lock(protected_mtx_);
        if (protected_nodes_.count(ev.node_name)) return;  // O(1)
    }

    const NodeProfile& n = *it;

    if (ev.goal_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
        switch (n.classe)
        {
            case NodeClasse::CORE:
            {
                state_.store(State::RESTARTING, std::memory_order_relaxed);
                return;
            }

            case NodeClasse::NAVIGATION:
            {
                bool ok = true;

                if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)){ok = false;} 
                if(ok && !check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){ok = false;} 
                if(ok && !node_activate(n)){ok = false;}

                if(!ok)
                {
                    for (auto& n_nav : nodes_) 
                    {
                        if (n_nav.classe != NodeClasse::NAVIGATION) continue;    
                        (void)check_dependencies(n_nav, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                        (void)node_deactivate(n_nav);
                    }
                    state_ = State::TELEOP;
                    return;
                }
                break;
            }

            case NodeClasse::PAYLOAD:
            {
                (void)check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                break;
            }

            default:
            {
                (void)check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                (void)check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                (void)node_activate(n);
                break;
            } 
        }
    }
    else if (ev.goal_state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        // Promote dependents back to ACTIVE when their dependency recovered.
        for (const auto& dep_name : n.dependents_of) 
        {
            auto it = std::find_if(nodes_.begin(), nodes_.end(),
                                   [&](const NodeProfile& x){ return x.name == dep_name; });

            if (it == nodes_.end()) continue;

            const NodeProfile& dep = *it;

            if (dep.classe == NodeClasse::PAYLOAD) continue;
            
            auto st = get_node_state(dep.name);

            if (!st) continue;

            (void)check_dependencies(dep,lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

            (void)node_activate(dep);
        }
    }
    else return;
}

void StateController::build_protected_nodes(const std::string& name)
{
    std::lock_guard<std::mutex> lock(protected_mtx_);

    protected_nodes_.clear();
        
    std::deque<std::string> q;
    q.push_back(name);
    protected_nodes_.insert(name);

    auto push = [&](const std::string& nm)
    {
        if (protected_nodes_.insert(nm).second) q.push_back(nm);
    };
        
    while (!q.empty()) 
    {
        std::string cur = q.front(); 
        q.pop_front();
        
        auto it = std::find_if(nodes_.begin(), nodes_.end(),
                    [&](const NodeProfile& x){ return x.name == cur; });

        if (it == nodes_.end()) continue;
        
        for (const auto& d : it->dependencies) push(d);
        for (const auto& c : it->dependents_of) push(c);
    }
}

std::tuple<CommandResponse,bool> StateController::command_manager(std::string& name, Command cmd)
{
    auto fail = [&](CommandResponse result)
    {
        has_command_ = false;
        return std::make_tuple(result, false);
    };

    auto success = [&]()
    {
        has_command_ = false;
        return std::make_tuple(CommandResponse::SUCCESS, true);
    };

    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeProfile& x){ return x.name == name; });

    if (it == nodes_.end()) 
    {
        return fail(CommandResponse::INVALID_NODE);
    }

    const NodeProfile& n = *it;  

    auto st = get_node_state(n.name);
    
    if(!st.has_value())
    {
        return fail(CommandResponse::FAILURE);
    }

    // Protect the entire dependency subgraph while executing a manual command.
    build_protected_nodes(name);
    has_command_ = true;

    switch(cmd)
    {
        case Command::ACTIVATE_NODE:
        {
            if(st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
                st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            {
                return fail(CommandResponse::INVALID_STATE);
            }

            if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            { 
                return fail(CommandResponse::FAILURE);
            }

            if(!node_activate(n))
            {  
                return fail(CommandResponse::FAILURE);
            }

            return success();
        }
        case Command::DEACTIVATE_NODE:
        {
            if(st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
                st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            { 
                return fail(CommandResponse::INVALID_STATE);
            }

            if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            { 
                return fail(CommandResponse::FAILURE);
            }

            if(!node_deactivate(n))
            {   
                return fail(CommandResponse::FAILURE);
            }

            return success();
        }
        case Command::RESTART_NODE:
        {
            // Force deactivate→cleanup and then configure→activate transitions.
            if(st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
                st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
                st->id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                return fail(CommandResponse::INVALID_STATE);
            }

            if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
            {
                return fail(CommandResponse::FAILURE);
            }

            if(!node_deactivate(n))
            {
                return fail(CommandResponse::FAILURE);
            }

            if(!check_dependencies(n, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
            {
                return fail(CommandResponse::FAILURE);
            }

            if(!node_activate(n))
            {
                return fail(CommandResponse::FAILURE);
            }

            return success();
        }
        default:
            return fail(CommandResponse::INVALID_COMMAND);
    }
}

State StateController::get_state()
{
    return state_;
}

void StateController::start_controller() 
{
    if (run_.exchange(true)) return;  // Already running
    state_.store(State::BRINGUP, std::memory_order_relaxed);
    update_th_ = std::thread(&StateController::update_loop, this);
    events_th_ = std::thread(&StateController::events_loop, this);
}

void StateController::stop_controller() 
{
    if (!run_.exchange(false)) return;   // Already stopped
    cv_event_.notify_all();              // Wake the event-processing thread
    if (update_th_.joinable()) update_th_.join();
    if (events_th_.joinable()) events_th_.join();
}

void StateController::update_loop() 
{
    while (run_.load(std::memory_order_relaxed)) 
    {
        update();
        std::this_thread::sleep_for(update_period_);
    }
}

void StateController::events_loop() 
{
    while (run_.load(std::memory_order_relaxed)) 
    {
        {
            std::unique_lock<std::mutex> lk(event_mtx_);
            // Wake whenever there are events ready for processing under allowed states.
            cv_event_.wait_for(lk, events_period_, [this]{
                const auto s = state_.load(std::memory_order_relaxed);
                const bool can_process = (s == State::AUTONOMOUS || s == State::TELEOP);
                return !run_.load(std::memory_order_relaxed) || (can_process && !events_.empty());
            });
            if (!run_.load(std::memory_order_relaxed)) break;
        }
        event_manager(); // Process a single event each cycle
    }
}

void StateController::restarting()
{
    lifecycle_->print_info("restarting: bring_down + bring_up");

    // Pause intake immediately so we can safely rebuild the graph state.
    events_blocked_.store(true, std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lk(event_mtx_);
        events_.clear();
    }
    has_command_ = false;
    {
        // Flush any protected nodes so the next command recomputes them.
        std::lock_guard<std::mutex> lock(protected_mtx_);
        protected_nodes_.clear();
    }

    restart_pending_.store(true, std::memory_order_relaxed);
    state_.store(State::BRINGDOWN, std::memory_order_relaxed);
}
