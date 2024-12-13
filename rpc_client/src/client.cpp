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
    auto req_msg = booster_interface::CreateSwitchHandEndEffectorControlModeMsg(true);
    // auto req_msg = booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kDamping);
    // auto req_msg = booster_interface::CreateLieDownMsg();
    request->msg = req_msg;

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
        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s",
                        result.get()->msg.body.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call rpc service");
        }
        // std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    return 0;
}