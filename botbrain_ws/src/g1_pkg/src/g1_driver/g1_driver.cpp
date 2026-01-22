#include <mutex>                                             
#include <thread>                                          
#include <chrono>    

#include "g1_driver.hpp"

G1Driver::G1Driver()
{
    // Initialize Unitree DDS ChannelFactory only once
    static std::once_flag unitree_dds_once;
    std::call_once(unitree_dds_once, []()
    {
        std::cerr << "[G1Driver] ChannelFactory Init(0)\n";
        unitree::robot::ChannelFactory::Instance()->Init(0);  
    });
}

G1Driver::~G1Driver()
{
    // Release loco client
    client_.reset();
}

bool G1Driver::init_loco_client(int speed_mode, bool keep_move)
{
    bool ok_init = false;   
    auto backoff = std::chrono::milliseconds(500);  // initial backoff
    
    try 
    {
        client_ = std::make_unique<unitree::robot::g1::LocoClient>();
        client_->SetTimeout(2.0f);

        // 1) Init() with retries
        for (int i = 1; i <= MAX_ATTEMPTS; ++i) 
        {                
            try 
            {  
                client_->Init();
                ok_init = true;
                break;
            } 
            catch (const std::exception& e) 
            {
                std::cerr << "[G1Driver] LocoClient Init() failed, attempt " << i << "/"
                    << MAX_ATTEMPTS << ": " << e.what() << "\n";
            } 
            std::this_thread::sleep_for(backoff);  
            if (backoff < std::chrono::seconds(2)) {backoff += std::chrono::milliseconds(500);}            
        }
        if (!ok_init) return false;  

        ok_init = false; 
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
        
        // 2) Continuous gait on/off with retries
        for (int i = 1; i <= MAX_ATTEMPTS; ++i) 
        {
            const int rc = continuous_gait(keep_move);
            if (rc == 0) 
            {
                ok_init = true;               
                break;                                               
            }
            std::this_thread::sleep_for(backoff);
            if (backoff < std::chrono::seconds(2)) {backoff += std::chrono::milliseconds(500);} 
        }
        if (!ok_init) return false;                                        

        std::this_thread::sleep_for(std::chrono::milliseconds(250));  

        ok_init = false;   
        
        // 3) Set speed mode with retries
        for (int i = 1; i <= MAX_ATTEMPTS; ++i) 
        {
            const int rc = set_speed_mode(speed_mode);
            if (rc == 0) {
                ok_init = true;                                           
                break;                                                
            }
            std::this_thread::sleep_for(backoff);
            if (backoff < std::chrono::seconds(2)){backoff += std::chrono::milliseconds(500);} 
        }
        if (!ok_init) return false;                   
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[G1Driver] Exception in init_loco_client: " << e.what() << "\n"; 
        return false;
    } 
    catch (...) 
    {
        std::cerr << "[G1Driver] Unknown exception in init_loco_client\n"; 
        return false;
    }
    return true;
}

bool G1Driver::init_low_level()
{
    static const std::string hg_cmd_topic   = "rt/arm_sdk";
    static const std::string hg_state_topic = "rt/lowstate";

    bool ok_init = false;   
    auto backoff = std::chrono::milliseconds(500);  

    try 
    {
        // Create publisher and subscriber
        lowcmd_publisher_.reset(
            new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(hg_cmd_topic));

        lowstate_subscriber_.reset(
            new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(hg_state_topic));

        // Init LowCmd publisher with retries
        for (int i = 1; i <= MAX_ATTEMPTS; ++i) 
        {                
            try 
            {  
                lowcmd_publisher_->InitChannel();
                ok_init = true;
                break;
            } 
            catch (const std::exception& e) 
            {
                std::cerr << "[G1Driver] Init LowCmd channel failed, attempt " << i << "/" 
                    << MAX_ATTEMPTS << ": " << e.what() << "\n";
            } 
            std::this_thread::sleep_for(backoff);  
            if (backoff < std::chrono::seconds(2)) {backoff += std::chrono::milliseconds(500);}            
        }
        if (!ok_init) return false;  

        std::this_thread::sleep_for(std::chrono::milliseconds(250));  

        ok_init = false;  

        // Init LowState subscriber with retries
        for (int i = 1; i <= MAX_ATTEMPTS; ++i) 
        {                
            try 
            {  
                lowstate_subscriber_->InitChannel(std::bind(&G1Driver::low_state_callback, this, 
                    std::placeholders::_1), 1);
                ok_init = true;
                break;
            } 
            catch (const std::exception& e) 
            {
                std::cerr << "[G1Driver] Init LowState channel failed, attempt "
                    << i << "/" << MAX_ATTEMPTS << ": " << e.what() << "\n";
            } 
            std::this_thread::sleep_for(backoff);  
            if (backoff < std::chrono::seconds(2)) {backoff += std::chrono::milliseconds(500);}            
        }
        if (!ok_init) return false; 
    }  
    catch (const std::exception& e) 
    {
        std::cerr << "[G1Driver] Exception in init_low_level: " << e.what() << "\n";
        return false;
    } 
    catch (...) 
    {
        std::cerr << "[G1Driver] Unknown exception in init_low_level\n";  
        return false;
    }
    return true;
}

int G1Driver::set_fsm_id(int fsm_id)
{
    return client_->SetFsmId(fsm_id);
}

int G1Driver::get_fsm_id(int& fsm_id)
{
    return client_->GetFsmId(fsm_id);
}

int G1Driver::get_fsm_mode(int& fsm_mode)
{
    return client_->GetFsmMode(fsm_mode); // 0 = standing, 1 = moving
}

int G1Driver::get_balance_mode(int& balance_mode)
{
    return client_->GetBalanceMode(balance_mode);
}

int G1Driver::start()
{
    return client_->Start();
}

int G1Driver::set_speed_mode(int speed_mode)
{
    return client_->SetSpeedMode(speed_mode);
}

int G1Driver::continuous_gait(bool flag)
{
    return client_->ContinuousGait(flag);
}

int G1Driver::move(float vx, float vy, float vyaw)
{
    return client_->Move(vx, -vy, vyaw);
}

int G1Driver::move(float vx, float vy, float vyaw, bool continous_move)
{
    return client_->Move(vx, -vy, vyaw, continous_move);
}

int G1Driver::stop_move()
{
    return client_->StopMove();
}

bool G1Driver::release_arm_joints()
{
    auto ms_ptr = motor_state_buffer_.GetData();
    if (!ms_ptr) 
    {
        std::cerr << "[G1Driver] release_arm_joints: MotorState not available yet\n";
        return false;
    }

    UpperBodyPose pose{};
    pose.waist_yaw = ms_ptr->q[WaistYaw];

    // left arm 
    pose.left.shoulder_pitch = ms_ptr->q[LeftShoulderPitch];
    pose.left.shoulder_roll  = ms_ptr->q[LeftShoulderRoll];
    pose.left.shoulder_yaw   = ms_ptr->q[LeftShoulderYaw];
    pose.left.elbow          = ms_ptr->q[LeftElbow];
    pose.left.wrist_roll     = ms_ptr->q[LeftWristRoll];

    // right arm
    pose.right.shoulder_pitch = ms_ptr->q[RightShoulderPitch];
    pose.right.shoulder_roll  = ms_ptr->q[RightShoulderRoll];
    pose.right.shoulder_yaw   = ms_ptr->q[RightShoulderYaw];
    pose.right.elbow          = ms_ptr->q[RightElbow];
    pose.right.wrist_roll     = ms_ptr->q[RightWristRoll];

    const float duration_sec = 2.0f;
    const float dt           = 0.02f;  // 20 ms → 50 Hz
    int steps = static_cast<int>(duration_sec / dt);
    if (steps < 1) steps = 1;

    // Ramp weight from 1.0 to 0.0
    for (int i = 0; i <= steps; ++i) 
    {
        float weight = 1.0f - static_cast<float>(i) / static_cast<float>(steps); // 1 → 0
        publish_upper_body_cmd(pose, weight);
        std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }

    weight_is_one_.store(false, std::memory_order_relaxed);

    return true;
}

bool G1Driver::get_upper_body_pose(UpperBodyPose & out_pose)
{
    auto ms_ptr = motor_state_buffer_.GetData();
    if (!ms_ptr) {
        std::cerr << "[G1Driver] get_upper_body_pose: MotorState not available yet\n";
        return false;
    }

    out_pose.waist_yaw = ms_ptr->q[WaistYaw];

    // left arm  
    out_pose.left.shoulder_pitch = ms_ptr->q[LeftShoulderPitch];
    out_pose.left.shoulder_roll  = ms_ptr->q[LeftShoulderRoll];
    out_pose.left.shoulder_yaw   = ms_ptr->q[LeftShoulderYaw];
    out_pose.left.elbow          = ms_ptr->q[LeftElbow];
    out_pose.left.wrist_roll     = ms_ptr->q[LeftWristRoll];

    // right arm                                             
    out_pose.right.shoulder_pitch = ms_ptr->q[RightShoulderPitch];
    out_pose.right.shoulder_roll  = ms_ptr->q[RightShoulderRoll];
    out_pose.right.shoulder_yaw   = ms_ptr->q[RightShoulderYaw];
    out_pose.right.elbow          = ms_ptr->q[RightElbow];
    out_pose.right.wrist_roll     = ms_ptr->q[RightWristRoll];

    return true;
}

bool G1Driver::set_arm_joints(const UpperBodyPose& target_pose_in)
{
    UpperBodyPose target_pose = target_pose_in;

    clamp_pose(target_pose); // ensure pose is inside limits

    const float duration_sec = 2.0f;
    const float dt = 0.02f;  // 20 ms → 50 Hz
    int steps = static_cast<int>(duration_sec / dt);
    if (steps < 1) steps = 1;

    // First call: ramp weight from 0.0 to 1.0
    if (!weight_is_one_.load(std::memory_order_relaxed))
    {
        for (int i = 0; i <= steps; ++i) 
        {
            float weight = static_cast<float>(i) / static_cast<float>(steps); // 0 → 1
            publish_upper_body_cmd(target_pose, weight);
            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
        }

        weight_is_one_.store(true, std::memory_order_relaxed);
        return true;
    }

    // Next calls: interpolate from current pose to target pose with weight = 1.0
    UpperBodyPose start_pose{};
    if (!get_upper_body_pose(start_pose)) return false;

    for (int i = 0; i <= steps; ++i)
    {
        float alpha = static_cast<float>(i) / static_cast<float>(steps); // 0 → 1

        UpperBodyPose cmd_pose = smooth_pose(start_pose, target_pose, alpha);
        publish_upper_body_cmd(cmd_pose, 1.0f);
        std::this_thread::sleep_for(std::chrono::duration<float>(dt));
    }

    weight_is_one_.store(true, std::memory_order_relaxed);
    return true;
}

void G1Driver::publish_upper_body_cmd(const UpperBodyPose& pose, float weight)
{
    // Clamp weight to [0, 1]
    weight = std::clamp(weight, 0.0f, 1.0f);

    unitree_hg::msg::dds_::LowCmd_ cmd{};

    cmd.mode_pr() = PR;
    cmd.mode_machine() = mode_machine_;

    // waist joint
    set_joint_cmd(cmd, G1JointIndex::WaistYaw, pose.waist_yaw);

    // left arm joints 
    set_joint_cmd(cmd, G1JointIndex::LeftShoulderPitch, pose.left.shoulder_pitch);
    set_joint_cmd(cmd, G1JointIndex::LeftShoulderRoll , pose.left.shoulder_roll);
    set_joint_cmd(cmd, G1JointIndex::LeftShoulderYaw  , pose.left.shoulder_yaw);
    set_joint_cmd(cmd, G1JointIndex::LeftElbow        , pose.left.elbow);
    set_joint_cmd(cmd, G1JointIndex::LeftWristRoll    , pose.left.wrist_roll);

    // right arm joints
    set_joint_cmd(cmd, G1JointIndex::RightShoulderPitch, pose.right.shoulder_pitch);
    set_joint_cmd(cmd, G1JointIndex::RightShoulderRoll , pose.right.shoulder_roll);
    set_joint_cmd(cmd, G1JointIndex::RightShoulderYaw  , pose.right.shoulder_yaw);
    set_joint_cmd(cmd, G1JointIndex::RightElbow        , pose.right.elbow);
    set_joint_cmd(cmd, G1JointIndex::RightWristRoll    , pose.right.wrist_roll);

    // weight joint index (kArmWeightJoint)
    cmd.motor_cmd()[kArmWeightJoint].mode() = 1;
    cmd.motor_cmd()[kArmWeightJoint].q()    = weight;
    cmd.motor_cmd()[kArmWeightJoint].dq()   = 0.0f;
    cmd.motor_cmd()[kArmWeightJoint].tau()  = 0.0f;
    cmd.motor_cmd()[kArmWeightJoint].kp()   = 0.0f;
    cmd.motor_cmd()[kArmWeightJoint].kd()   = 0.0f;

    cmd.crc() = crc3_2_core(reinterpret_cast<uint32_t*>(&cmd), (sizeof(cmd) >> 2) - 1);

    lowcmd_publisher_->Write(cmd);
}

void G1Driver::low_state_callback(const void* message) 
{
    unitree_hg::msg::dds_::LowState_ low_state =
        *(const unitree_hg::msg::dds_::LowState_ *)message;

    // Validate CRC 
    if (low_state.crc() != crc3_2_core((uint32_t *)&low_state,
        (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) return;
    
    MotorState ms_tmp;

    // Copy joint positions and velocities
    for (int i = 0; i < G1_NUM_MOTOR; ++i) 
    {
        ms_tmp.q[i]  = low_state.motor_state()[i].q();
        ms_tmp.dq[i] = low_state.motor_state()[i].dq();
    }

    motor_state_buffer_.SetData(ms_tmp);

    // Update mode_machine_ if it changed
    if (mode_machine_ != low_state.mode_machine()) 
    {
        mode_machine_ = low_state.mode_machine();
    }
}

float G1Driver::get_motor_kd(MotorType type) 
{
    switch (type) 
    {
        case GearboxS: return 1.f;
        case GearboxM: return 1.f;
        case GearboxL: return 1.f;
        default:       return 0.f;
    }
}

float G1Driver::get_motor_kp(MotorType type) 
{
    switch (type) 
    {
        case GearboxS: return 40.f;
        case GearboxM: return 40.f;
        case GearboxL: return 100.f;
        default:       return 0.f;
    }
}

void G1Driver::clamp_pose(UpperBodyPose & pose) const
{
    pose.waist_yaw = std::clamp(pose.waist_yaw, g1_limits::WAIST_YAW.min, 
        g1_limits::WAIST_YAW.max);

    // left arm limits  
    pose.left.shoulder_pitch = std::clamp(pose.left.shoulder_pitch, 
        g1_limits::L_SHOULDER_PITCH.min, g1_limits::L_SHOULDER_PITCH.max);

    pose.left.shoulder_roll = std::clamp(pose.left.shoulder_roll, 
        g1_limits::L_SHOULDER_ROLL.min, g1_limits::L_SHOULDER_ROLL.max);

    pose.left.shoulder_yaw = std::clamp(pose.left.shoulder_yaw, 
        g1_limits::L_SHOULDER_YAW.min, g1_limits::L_SHOULDER_YAW.max);

    pose.left.elbow = std::clamp(pose.left.elbow, g1_limits::L_ELBOW.min, 
        g1_limits::L_ELBOW.max);

    pose.left.wrist_roll = std::clamp(pose.left.wrist_roll, g1_limits::L_WRIST_ROLL.min, 
        g1_limits::L_WRIST_ROLL.max);

    // right arm limits          
    pose.right.shoulder_pitch = std::clamp(pose.right.shoulder_pitch, 
        g1_limits::R_SHOULDER_PITCH.min, g1_limits::R_SHOULDER_PITCH.max);

    pose.right.shoulder_roll = std::clamp(pose.right.shoulder_roll, 
        g1_limits::R_SHOULDER_ROLL.min, g1_limits::R_SHOULDER_ROLL.max);

    pose.right.shoulder_yaw = std::clamp(pose.right.shoulder_yaw,   
        g1_limits::R_SHOULDER_YAW.min, g1_limits::R_SHOULDER_YAW.max);

    pose.right.elbow = std::clamp(pose.right.elbow, g1_limits::R_ELBOW.min, 
        g1_limits::R_ELBOW.max);

    pose.right.wrist_roll = std::clamp(pose.right.wrist_roll, g1_limits::R_WRIST_ROLL.min, 
        g1_limits::R_WRIST_ROLL.max);
}

inline uint32_t G1Driver::crc3_2_core(uint32_t *ptr, uint32_t len) 
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++) 
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) 
        {
            if (CRC32 & 0x80000000) 
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else 
            {
                CRC32 <<= 1;
            }
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

inline void G1Driver::set_joint_cmd(unitree_hg::msg::dds_::LowCmd_& cmd, int idx, float q)
{
    const MotorType type = g1_motor_type_[idx];

    cmd.motor_cmd()[idx].mode() = 1;  
    cmd.motor_cmd()[idx].q()    = q;  
    cmd.motor_cmd()[idx].dq()   = 0.0f;
    cmd.motor_cmd()[idx].tau()  = 0.0f;
    cmd.motor_cmd()[idx].kp()   = get_motor_kd(type);
    cmd.motor_cmd()[idx].kd()   = get_motor_kd(type);
}

UpperBodyPose G1Driver::smooth_pose(const UpperBodyPose& a, const UpperBodyPose& b,float t)
    const
{
    auto lerp = [](float a, float b, float t) 
    {
        return a + (b - a) * t;
    };

    t = std::clamp(t, 0.0f, 1.0f);

    UpperBodyPose r{};

    r.waist_yaw = lerp(a.waist_yaw, b.waist_yaw, t);

    // left arm 
    r.left.shoulder_pitch = lerp(a.left.shoulder_pitch, b.left.shoulder_pitch, t);
    r.left.shoulder_roll  = lerp(a.left.shoulder_roll,  b.left.shoulder_roll,  t);
    r.left.shoulder_yaw   = lerp(a.left.shoulder_yaw,   b.left.shoulder_yaw,   t);
    r.left.elbow          = lerp(a.left.elbow,          b.left.elbow,          t);
    r.left.wrist_roll     = lerp(a.left.wrist_roll,     b.left.wrist_roll,     t);

    // right arm
    r.right.shoulder_pitch = lerp(a.right.shoulder_pitch, b.right.shoulder_pitch, t);
    r.right.shoulder_roll  = lerp(a.right.shoulder_roll,  b.right.shoulder_roll,  t);
    r.right.shoulder_yaw   = lerp(a.right.shoulder_yaw,   b.right.shoulder_yaw,   t);
    r.right.elbow          = lerp(a.right.elbow,          b.right.elbow,          t);
    r.right.wrist_roll     = lerp(a.right.wrist_roll,     b.right.wrist_roll,     t);

    return r;
}