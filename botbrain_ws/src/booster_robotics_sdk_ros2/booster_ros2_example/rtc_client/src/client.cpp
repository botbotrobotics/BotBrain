#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/message_utils.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "booster/robot/ai/api.hpp"
#include "include/prompt.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

booster_interface::msg::BoosterApiReqMsg GenerateMsg(const int64_t api, const std::string &body) {
  booster_interface::msg::BoosterApiReqMsg msg;
  msg.api_id = api;
  msg.body = body;
  return msg;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node>  rtc_node =
        rclcpp::Node::make_shared("rtc_client_node");
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr rpc_client =
        rtc_node->create_client<booster_interface::srv::RpcService>(
            "booster_rtc_service");


  // Start Ai Chat
  std::vector<std::string> interrupt_keywords;
  interrupt_keywords.push_back("apple");
  interrupt_keywords.push_back("banana");
  booster::robot::StartAiChatParameter param;
  param.llm_config_.system_prompt_ = kDefaultSystemPrompt;
  param.llm_config_.welcome_msg_ = kWelcomeMsg;
  param.interrupt_mode_ = false;
  param.asr_config_.interrupt_keywords_ = interrupt_keywords;
  param.asr_config_.interrupt_speech_duration_ = 200;
  param.tts_config_.ignore_bracket_text_ = {3};
  param.tts_config_.voice_type_ = "zh_female_shuangkuaisisi_emo_v2_mars_bigtts";
  param.enable_face_tracking_ = false;
  std::string body = param.ToJson().dump();
  auto req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = GenerateMsg((int64_t)booster::robot::AiApiId::kStartAiChat, body);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending API ID: %ld, Body: %s", req->msg.api_id,
             req->msg.body.c_str());

  auto future = rpc_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rtc_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RtcService error ");
	 return -1;
  } 

  booster_interface::msg::BoosterApiRespMsg response = future.get()->msg;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "StartVoiceSpeak: %ld", response.status);

  std::this_thread::sleep_for(2s);

  // Speak
  booster::robot::SpeakParameter speak_param;
  speak_param.msg_ = "I love robot";
  body = speak_param.ToJson().dump();
  req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = GenerateMsg((int64_t)booster::robot::AiApiId::kSpeak, body);

   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending API ID: %ld, Body: %s", req->msg.api_id,
             req->msg.body.c_str());

  future = rpc_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rtc_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RtcService error ");
         return -1;
  }

  response = future.get()->msg;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak: %ld", response.status);

  std::this_thread::sleep_for(2s);

  // Stop Voice Chat
  req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = GenerateMsg((int64_t)booster::robot::AiApiId::kStopAiChat, ""); 
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending API ID: %ld, Body: %s", req->msg.api_id,
              req->msg.body.c_str());

  future = rpc_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rtc_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RtcService error ");
         return -1;
  }

  response = future.get()->msg;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "StopVoiceSpeak: %ld", response.status);



  std::this_thread::sleep_for(2s);
  // Start Face Tracking
  req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = GenerateMsg((int64_t)booster::robot::AiApiId::kStartFaceTracking, "");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending API ID: %ld, Body: %s", req->msg.api_id,
              req->msg.body.c_str());

  future = rpc_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rtc_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RtcService error ");
         return -1;
  }

  response = future.get()->msg;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "StartFaceTracking: %ld", response.status);

  std::this_thread::sleep_for(2s);

  // Stop Face Tracking  
  req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = GenerateMsg((int64_t)booster::robot::AiApiId::kStopFaceTracking, "");

 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending API ID: %ld, Body: %s", req->msg.api_id,
             req->msg.body.c_str());

  future = rpc_client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rtc_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RtcService error ");
         return -1;
  }

   response = future.get()->msg;
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "StopFaceTracking: %ld", response.status);

  rclcpp::shutdown();
  return 0;
}
