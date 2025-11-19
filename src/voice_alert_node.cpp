#include "voice_alert_pkg/voice_alert_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

VoiceAlertNode::VoiceAlertNode() : Node("voice_alert_node")
{
    // 패키지 내 audio 폴더 경로
    std::string pkg_path = ament_index_cpp::get_package_share_directory("voice_alert_pkg");
    
    // 음성 파일 경로 매핑
    audio_files_["left"] = pkg_path + "/audio/left.wav";
    audio_files_["front"] = pkg_path + "/audio/front.wav";
    audio_files_["right"] = pkg_path + "/audio/right.wav";
    
    // Subscriber 생성
    direction_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/obstacle_direction", 10,
        std::bind(&VoiceAlertNode::direction_callback, this, std::placeholders::_1)
    );
    
    last_play_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Voice Alert Node Started");
}

void VoiceAlertNode::direction_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string direction = msg->data;
    
    // 중복 방지: 같은 방향 연속 재생 제한
    auto now = this->now();
    double time_diff = (now - last_play_time_).seconds();
    
    if (direction == last_direction_ && time_diff < 3.0) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Obstacle detected: %s", direction.c_str());
    play_audio(direction);
    
    last_direction_ = direction;
    last_play_time_ = now;
}

void VoiceAlertNode::play_audio(const std::string& direction)
{
    if (audio_files_.find(direction) == audio_files_.end()) {
        RCLCPP_WARN(this->get_logger(), "Unknown direction: %s", direction.c_str());
        return;
    }
    
    std::string audio_path = audio_files_[direction];
    
    // aplay로 재생 (백그라운드)
    std::string command = "aplay " + audio_path + " &";
    int result = system(command.c_str());
    
    if (result != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to play audio: %s", audio_path.c_str());
    }
}
