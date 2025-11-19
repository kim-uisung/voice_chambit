#include <rclcpp/rclcpp.hpp>
#include "voice_alert_pkg/voice_alert_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoiceAlertNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
