#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("rpc_client_node");
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client =
        node->create_client<booster_interface::srv::RpcService>(
            "booster_rpc_service");

    auto request =
        std::make_shared<booster_interface::srv::RpcService::Request>();
    auto req_msg = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kSwitchHandEndEffectorControlMode,
        booster::robot::b1::SwitchHandEndEffectorControlModeParameter>(true);

    auto req_msg_close = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kSwitchHandEndEffectorControlMode,
        booster::robot::b1::SwitchHandEndEffectorControlModeParameter>(false);

    auto req_move_msg = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kMove,
        booster::robot::b1::MoveParameter>(0.5, 0.0, 0.0);

    auto req_move_msg_zero = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kMove,
        booster::robot::b1::MoveParameter>(0.0, 0.0, 0.0);

    booster::robot::Posture tar_posture;
    tar_posture.position_ = booster::robot::Position(0.35, 0.25, 0.1);
    tar_posture.orientation_ = booster::robot::Orientation(0., 0., 0.);

    auto req_move_hand_end_effector_msg = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kMoveHandEndEffector,
        booster::robot::b1::MoveHandEndEffectorParameter>(
        tar_posture, 2000, booster::robot::b1::HandIndex::kLeftHand);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }

    while (rclcpp::ok()) {
        // change to hand end effector control mode
        request->msg = req_msg;
        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        std::this_thread::sleep_for(2s);

        // start to move
        request->msg = req_move_msg;
        result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        std::this_thread::sleep_for(2s);

        // move hand end effector
        request->msg = req_move_hand_end_effector_msg;
        result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        std::this_thread::sleep_for(20s);

        // stop moving
        request->msg = req_move_msg_zero;
        result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        std::this_thread::sleep_for(2s);

        // change to hand end effector control mode
        request->msg = req_msg_close;
        result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        std::this_thread::sleep_for(10s);
    }

    rclcpp::shutdown();
    return 0;
}