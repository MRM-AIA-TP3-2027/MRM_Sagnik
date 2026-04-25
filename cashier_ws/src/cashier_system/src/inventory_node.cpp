#include "rclcpp/rclcpp.hpp"

#include "cashier_system/msg/bill.hpp"
#include <map>

class InventoryNode : public rclcpp::Node {
public:
    InventoryNode() : Node("inventory_node"), total_income_(0.0) {
        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill_topic", 10,
            std::bind(&InventoryNode::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const cashier_system::msg::Bill::SharedPtr msg) {
        inventory_[msg->item_name] -= msg->quantity;
        total_income_ += msg->quantity * msg->price_per_unit;

        std::cout << "Item: " << msg->item_name << std::endl;
        std::cout << "Remaining: " << inventory_[msg->item_name] << std::endl;
        std::cout << "Total Income: " << total_income_ << std::endl;
        std::cout << "------------------------" << std::endl;
    }

    std::map<std::string, int> inventory_;
    float total_income_;
    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventoryNode>());
    rclcpp::shutdown();
    return 0;
}
