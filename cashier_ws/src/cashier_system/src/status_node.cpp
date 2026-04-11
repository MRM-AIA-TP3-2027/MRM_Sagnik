#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "cashier_system/srv/get_status.hpp"

#include <iostream>
#include <thread>
#include <map>
#include <sstream>

using GetStatus = cashier_system::srv::GetStatus;

class StatusNode : public rclcpp::Node {
public:
    StatusNode() : Node("status_node"), total_bill_(0) {

        // SUBSCRIBER
        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill_topic",
            10,
            std::bind(&StatusNode::callback, this, std::placeholders::_1)
        );

        // SERVICE
        service_ = this->create_service<GetStatus>(
            "get_status",
            std::bind(&StatusNode::handle_service, this,
                      std::placeholders::_1,
                      std::placeholders::_2)
        );

        // CLIENT
        client_ = this->create_client<GetStatus>("get_status");

        // KEYBOARD THREAD
        input_thread_ = std::thread(&StatusNode::keyboard_listener, this);
    }

    ~StatusNode() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    rclcpp::Service<GetStatus>::SharedPtr service_;
    rclcpp::Client<GetStatus>::SharedPtr client_;
    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;

    int total_bill_;
    std::map<std::string, int> inventory_;

    std::thread input_thread_;

    // SERVICE CALLBACK
    void handle_service(
        const std::shared_ptr<GetStatus::Request> request,
        std::shared_ptr<GetStatus::Response> response)
    {
        (void)request;

        std::stringstream ss;

        ss << "\n=== SYSTEM STATUS ===\n";
        ss << "Total Bill: " << total_bill_ << "\n";
        ss << "Inventory:\n";

        for (auto &item : inventory_) {
            ss << item.first << " : " << item.second << "\n";
        }

        response->status = ss.str();
    }

    // SUBSCRIBER CALLBACK
    void callback(const cashier_system::msg::Bill::SharedPtr msg)
    {
        inventory_[msg->item_name] += msg->quantity;
        total_bill_ += msg->quantity * msg->price_per_unit;
    }

    // KEYBOARD LISTENER
    void keyboard_listener() {
        while (rclcpp::ok()) {
            char key;
            std::cout << "Press 's' to view system status: ";
            std::cin >> key;

            if (key == 's') {

                auto request = std::make_shared<GetStatus::Request>();
                request->request = "status";

                while (!client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        return;
                    }
                    std::cout << "Waiting for service...\n";
                }

                auto future = client_->async_send_request(request);

                if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
                {
                    std::cout << future.get()->status << std::endl;
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StatusNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
