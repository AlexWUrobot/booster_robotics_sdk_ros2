#include "booster_interface/srv/rpc_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

#include <booster/robot/rpc/request_header.hpp>
#include <booster/robot/rpc/response_header.hpp>
#include <booster/robot/rpc/error.hpp>

#include <iostream>

namespace booster {
namespace robot {

void AiClient::Init() {
    rtc_node_ = rclcpp::Node::make_shared("rtc_client_node");
    rpc_client_ = rtc_node_->create_client<booster_interface::srv::RpcService>(kServiceRtc);
}


int32_t AiClient::SendApiRequestWithResponse(AiApiId api_id, const std::string &param) {
    auto req = std::make_shared<booster_interface::srv::RpcService::Request>();
    req->msg = GenerateMsg(api_id, body);

    auto future = rpc_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(rtc_node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
         return -100;
    }

    booster_interface::msg::BoosterApiRespMsg response = future.get()->msg;
    return response.status;
}

booster_interface::msg::BoosterApiReqMsg AiClient::GenerateMsg(const int64_t api, const std::string &body) {
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = api;
    msg.body = body;
    return msg;
}


}
} // namespace booster::robot::b1
