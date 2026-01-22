#pragma once
#include <array>

#define MAX_ATTEMPTS 5 // max number of retry attempts

const int G1_NUM_MOTOR = 29; // number of motors stored in arrays (legs + waist + arms)
constexpr int kArmWeightJoint = 29; //index of weight joint in LowCmd (separate from G1_NUM_MOTOR)

// FSM IDs used by the loco client.
enum class FsmId : int 
{
    ZERO_TORQUE = 0,
    DAMPING = 1,
    SQUAT_POS = 2,
    SIT_POS = 3,
    LOCK_STAND = 4,
    WALK = 500,
    WALK3DOF = 501,
    LIE_DOWN_STAND_UP = 702, // lie down or stand up
    BALANCE_SQUAT_SQUAT_STAND = 706, // balance squat or squat stand
    RUN = 801,
    ERROR = -1,
};

// Preset base speed modes.
enum class Speed : int
{
    SPEED_1M_S   = 0,  // 1.0 m/s
    SPEED_2M_S   = 1,  // 2.0 m/s
    SPEED_2_7M_S = 2,  // 2.7 m/s
    SPEED_3M_S   = 3   // 3.0 m/s (modo Run)
};

// Balance mode reported by the robot.
enum class BalanceMode : int
{
    BALANCE_STANDING = 0,
    CONTINUOUS_MOVEMENT = 1,
    FORCED_STANDING = 2,
};

// FSM mode (high-level state).
enum class FmmMode : int
{
    STANDING = 0,
    MOVING = 1,
};

// Motor feedback state (position and velocity).
struct MotorState 
{
    std::array<float, G1_NUM_MOTOR> q  {};
    std::array<float, G1_NUM_MOTOR> dq {};
};

// Motor command struct used for higher-level planning.
struct MotorCommand 
{
    std::array<float, G1_NUM_MOTOR> q_target  {};
    std::array<float, G1_NUM_MOTOR> dq_target {};
    std::array<float, G1_NUM_MOTOR> kp        {};
    std::array<float, G1_NUM_MOTOR> kd        {};
    std::array<float, G1_NUM_MOTOR> tau_ff    {};
};

// Joint indices for waist and arms in the motor arrays.
enum G1JointIndex 
{
    WaistYaw = 12,

    // left arm joints
    LeftShoulderPitch = 15,
    LeftShoulderRoll  = 16,
    LeftShoulderYaw   = 17,
    LeftElbow         = 18,
    LeftWristRoll     = 19,

    // right arm joints
    RightShoulderPitch = 22,
    RightShoulderRoll  = 23,
    RightShoulderYaw   = 24,
    RightElbow         = 25,
    RightWristRoll     = 26,
};

// Mode for PR/AB control.
enum PRorAB { PR = 0, AB = 1 };

struct ArmPose
{
    float shoulder_pitch;  // shoulder pitch [rad]
    float shoulder_roll;  // shoulder roll [rad]
    float shoulder_yaw;  // shoulder yaw [rad]
    float elbow;   // elbow joint [rad]
    float wrist_roll;  // wrist roll [rad]
};

struct UpperBodyPose
{
    float waist_yaw;   // waist yaw [rad]

    ArmPose left;     // left arm pose
    ArmPose right;    // right arm pose
};

// Type of gearbox for each motor.
enum MotorType { GearboxS = 0, GearboxM = 1, GearboxL = 2 };

// High-level arm commands for application logic.
enum ArmCommand
{
    SAVE_POSE = 1,
    GO_TO_POSE = 2,
    RELEASE_POSE = 3,
    SEND_JOINT_POS = 4,
    DELETE_POSE = 5,
};

struct JointLimit
{
    float min; // minimum joint angle [rad]
    float max; // maximum joint angle [rad]
};
namespace g1_limits
{
    inline constexpr JointLimit WAIST_YAW       { -2.5f,  2.5f  };  // orig: [-2.618 ,  2.618 ]

    inline constexpr JointLimit L_SHOULDER_PITCH{ -2.9f,  2.5f  };  // orig: [-3.0892,  2.6704]
    inline constexpr JointLimit L_SHOULDER_ROLL { -1.4f,  2.1f  };  // orig: [-1.5882,  2.2515]
    inline constexpr JointLimit L_SHOULDER_YAW  { -2.5f,  2.5f  };  // orig: [-2.618 ,  2.618 ]
    inline constexpr JointLimit L_ELBOW         { -0.9f,  2.0f  };  // orig: [-1.0472,  2.0944]
    inline constexpr JointLimit L_WRIST_ROLL    { -1.8f,  1.8f  };  // orig: [-1.9722,  1.9722]

    inline constexpr JointLimit R_SHOULDER_PITCH{ -2.9f,  2.5f  };  // orig: [-3.0892,  2.6704]
    inline constexpr JointLimit R_SHOULDER_ROLL { -2.1f,  1.4f  };  // orig: [-2.2515,  1.5882]
    inline constexpr JointLimit R_SHOULDER_YAW  { -2.5f,  2.5f  };  // orig: [-2.618 ,  2.618 ]
    inline constexpr JointLimit R_ELBOW         { -0.9f,  2.0f  };  // orig: [-1.0472,  2.0944]
    inline constexpr JointLimit R_WRIST_ROLL    { -1.8f,  1.8f  };  // orig: [-1.9722,  1.9722]
} 