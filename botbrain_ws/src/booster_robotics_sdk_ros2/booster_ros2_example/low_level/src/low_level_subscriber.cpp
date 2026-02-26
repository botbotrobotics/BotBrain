#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "booster_interface/msg/low_state.hpp"

using std::placeholders::_1;

class LowLevelSubscriber : public rclcpp::Node {
public:
    LowLevelSubscriber() :
        Node("low_level_subscriber") {
        subscription_ = this->create_subscription<booster_interface::msg::LowState>(
            "/low_state", 10,
            std::bind(&LowLevelSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const booster_interface::msg::LowState &msg) const {
        RCLCPP_INFO(this->get_logger(), "Received imu x, y, z: %f %f %f", msg.imu_state.acc[0], msg.imu_state.acc[1], msg.imu_state.acc[2]);
    }
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr subscription_;
};

int main(int arc, char *argv[]) {
    rclcpp::init(arc, argv);
    rclcpp::spin(std::make_shared<LowLevelSubscriber>());
    rclcpp::shutdown();
    return 0;
}
