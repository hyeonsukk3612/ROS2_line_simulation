#include <rclcpp/rclcpp.hpp>                     // ROS2 C++ 노드 라이브러리
#include <geometry_msgs/msg/vector3.hpp>          // geometry_msgs의 Vector3 메시지 타입

class MotorCmdSubscriber : public rclcpp::Node {
public:
    MotorCmdSubscriber() : Node("motor_cmd_subscriber") { // 노드 생성자, 이름 지정
        // topic_dxlpub 토픽 구독 (큐 크기 10)
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "topic_dxlpub", 10,
            std::bind(&MotorCmdSubscriber::cmd_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "topic_dxlpub 구독 시작!"); // 로그 출력
    }

private:
    void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { // 메시지 콜백 함수
        double left_speed = msg->x;  // 왼쪽 바퀴 속도
        double right_speed = msg->y; // 오른쪽 바퀴 속도
        // 받은 값을 콘솔에 출력 (또는 실제 모터 제어 함수에 전달)
        RCLCPP_INFO(this->get_logger(), "받은 모터 속도: left=%.1f, right=%.1f", left_speed, right_speed);

        // 실제 Dynamixel 제어 함수 예시 (dxl.cpp/dxl.hpp 연동 시)
        // dxl.setVelocity(left_speed, right_speed);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_; // 구독자 포인터
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2 초기화
    rclcpp::spin(std::make_shared<MotorCmdSubscriber>()); // 노드 실행 및 콜백 대기
    rclcpp::shutdown(); // ROS2 종료
    return 0;
}