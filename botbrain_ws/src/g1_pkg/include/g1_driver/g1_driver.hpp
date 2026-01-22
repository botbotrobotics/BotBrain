#ifndef G1DRIVER_HPP
#define G1DRIVER_HPP

#include <cmath>
#include <mutex>
#include <memory>
#include <thread>
#include <iomanip>
#include <fstream>
#include <unistd.h>  
#include <iostream>
#include "types.hpp"
#include <algorithm> 
#include <filesystem>
#include <shared_mutex>

// Unitree robot
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// Unitree IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

// Simple thread-safe buffer for the latest value of type T.
template <typename T>
class DataBuffer 
{
public:
    // Store a new value (overwrite the previous one).
    void SetData(const T &newData) 
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = std::make_shared<T>(newData);
    }

    /**
     * @brief Get the latest value.
     * @return Shared pointer to the value, or nullptr if empty.
     */
    std::shared_ptr<const T> GetData() const 
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return data_;
    }
private:
  std::shared_ptr<T> data_;
  mutable std::shared_mutex mutex_;
};

// Driver for Unitree G1 locomotion and upper body control.
class G1Driver
{
private:
    uint8_t mode_machine_{0};  // Last machine mode received from low state.
    std::atomic<bool> weight_is_one_{false};  // True when arm weight joint has reached 1.0.

    DataBuffer<MotorState>  motor_state_buffer_; // Latest motor state (positions/velocities).

    std::unique_ptr<unitree::robot::g1::LocoClient> client_; // High-level loco client (walking/FSM).

    // Publisher for low-level commands.
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;

    // Subscriber for low-level state.
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;

    // Motor type for each joint, used to set KP/KD.
    std::array<MotorType, G1_NUM_MOTOR> g1_motor_type_
    {
        // legs
        GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
        GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
        // waist
        GearboxM, GearboxS, GearboxS,
        // arms
        GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS,
        GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS
    };

public:
    // Initializes Unitree DDS ChannelFactory once.
    G1Driver();

    // Destructor.
    ~G1Driver();

    // ─────────────── Loco client (walking / FSM) ───────────────

    // Initialize low-level channels for arm/waist control.
    bool init_loco_client(int speed_mode, bool keep_move);

    /**
     * @brief Initialize locomotion client and set basic parameters.
     * @param speed_mode Speed mode index.
     * @param keep_move  Enable or disable continuous gait.
     * @return true on success, false on failure.
    */
    bool init_low_level();

    //Set FSM ID.
    int set_fsm_id(int fsm_id);

    // Get current FSM ID.
    int get_fsm_id(int& fsm_id);

    // Get current FSM mode (e.g. 0 = stand, 1 = move).
    int get_fsm_mode(int& fsm_mode);

    // Get current balance mode.
    int get_balance_mode(int& balance_mode);

    // Start locomotion client.
    int start();

    // Set speed mode.
    int set_speed_mode(int speed_mode);

    // Enable or disable continuous gait.
    int continuous_gait(bool flag);

    /**
     * @brief Command base velocity (continuous gait).
     *
     * @param vx   Linear velocity in X (m/s).
     * @param vy   Linear velocity in Y (m/s).
     * @param vyaw Angular velocity around Z (rad/s).
     */
    int move(float vx, float vy, float vyaw);

    // Command base velocity with continuous flag.
    int move(float vx, float vy, float vyaw, bool continous_move);

    // Stop base motion.
    int stop_move();

    // ─────────────── Upper body / arms ───────────────

    // Smoothly reduce the arm weight from 1.0 to 0.0.
    bool release_arm_joints();

    /**
     * @brief Get current upper body pose from motor state.
     * @param out_pose Filled with waist and arm joint angles.
     * @return true if motor state is available, false otherwise.
     */
    bool get_upper_body_pose(UpperBodyPose & out_pose);

    /**
     * @brief Move arms/waist to the target pose with a smooth motion.
     * @param target_pose_in Target joint angles (clamped internally).
     * @return true on success, false if motor state is missing.
     */
    bool set_arm_joints(const UpperBodyPose& target_pose_in);

    /**
     * @brief Publish a low-level command for upper body and weight joint.
     * @param pose   Target waist and arm joints.
     * @param weight Weight value in [0, 1].
     */
    void publish_upper_body_cmd(const UpperBodyPose& pose, float weight);

private:
    // Low State Callback function
    void low_state_callback(const void* message);

    // ─────────────── Utils functions ───────────────

    // Get KD gain for a motor type.
    float get_motor_kd(MotorType type);

    // Get KP gain for a motor type.
    float get_motor_kp(MotorType type);

    // Clamp pose values to joint limits.
    void clamp_pose(UpperBodyPose & pose) const;

    // Compute CRC32 for LowCmd_ / LowState_ messages.
    inline uint32_t crc3_2_core(uint32_t *ptr, uint32_t len);

    // Fill one joint command entry in LowCmd_.
    inline void set_joint_cmd(unitree_hg::msg::dds_::LowCmd_& cmd, int idx, float q);

    /**
     * @brief Linear interpolation between two poses.
     * @param a First pose.
     * @param b Second pose.
     * @param t Factor in [0, 1].
     */
    UpperBodyPose smooth_pose(const UpperBodyPose& a, const UpperBodyPose& b, float t) const;
};

#endif // G1DRIVER_HPP