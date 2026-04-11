#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"

class BillGenerator : public rclcpp::Node {
public:
    BillGenerator() : Node("bill_generator") {
        publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill_topic", 10);
    }

    void run() {
        while (rclcpp::ok()) {
            cashier_system::msg::Bill msg;

            std::cout << "Item: ";
            std::cin >> msg.item_name;

            std::cout << "Quantity: ";
            std::cin >> msg.quantity;

            std::cout << "Price: ";
            std::cin >> msg.price_per_unit;

            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Bill sent");
        }
    }

private:
    rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BillGenerator>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
