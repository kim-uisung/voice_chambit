#ifndef VOICE_ALERT_NODE_HPP_
#define VOICE_ALERT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <map>

class VoiceAlertNode : public rclcpp::Node
{
public:
    VoiceAlertNode();
    
private:
    void direction_callback(const std_msgs::msg::String::SharedPtr msg);
    void play_audio(const std::string& direction);
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_sub_;
    std::map<std::string, std::string> audio_files_;
    
    std::string last_direction_;
    rclcpp::Time last_play_time_;
};

#endif
