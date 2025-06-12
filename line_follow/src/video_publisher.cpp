#include <rclcpp/rclcpp.hpp>                               // ROS2 C++ 라이브러리
#include <sensor_msgs/msg/image.hpp>                       // ROS2 이미지 메시지 타입
#include <cv_bridge/cv_bridge.h>                           // OpenCV와 ROS2 이미지 변환용 라이브러리
#include <opencv2/opencv.hpp>                              // OpenCV 라이브러리
using namespace std;                                       // 표준 네임스페이스 사용
using namespace cv;                                        // OpenCV 네임스페이스 사용

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {           // 노드 생성자, 노드 이름은 "video_publisher"
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video/image_raw", 10); // "/video/image_raw" 토픽으로 이미지 발행
        video_path_ = "/home/linux/ros2_ws/src/line_follow/include/line_follow/7_lt_ccw_100rpm_in.mp4"; // 비디오 파일 경로
        cap_.open(video_path_, CAP_FFMPEG);                // FFMPEG로 비디오 파일 열기
        if (!cap_.isOpened()) {                            // 비디오 파일 열기 실패 시
            RCLCPP_ERROR(this->get_logger(), "Could not open video file: %s", video_path_.c_str());
            rclcpp::shutdown();                            // 노드 종료
            return;
        }
        timer_ = this->create_wall_timer(                  // 33ms마다 timer_callback 함수 호출
            std::chrono::milliseconds(33),
            std::bind(&VideoPublisher::timer_callback, this)
        );
    }
private:
    void timer_callback() {                                // 타이머 콜백 함수
        Mat frame;
        cap_ >> frame;                                     // 비디오에서 한 프레임 읽기
        if (frame.empty()) {                               // 프레임이 비었으면(비디오 종료)
            RCLCPP_INFO(this->get_logger(), "Video ended.");
            rclcpp::shutdown();                            // 노드 종료
            return;
        }
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(); // OpenCV 이미지를 ROS2 이미지 메시지로 변환
        publisher_->publish(*msg);                         // 이미지 메시지 발행
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; // 이미지 발행자
    rclcpp::TimerBase::SharedPtr timer_;                   // 타이머
    VideoCapture cap_;                                     // 비디오 캡처 객체
    string video_path_;                                    // 비디오 파일 경로
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                              // ROS2 초기화
    rclcpp::spin(std::make_shared<VideoPublisher>());      // 노드 실행(이벤트 루프)
    rclcpp::shutdown();                                    // ROS2 종료
    return 0;
}
