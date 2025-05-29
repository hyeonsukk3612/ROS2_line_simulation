#include <rclcpp/rclcpp.hpp>            // ROS2 C++ 라이브러리
#include <sensor_msgs/msg/image.hpp>    // 이미지 메시지 타입
#include <cv_bridge/cv_bridge.h>        // OpenCV와 ROS2 이미지 변환
#include <opencv2/opencv.hpp>           // OpenCV 라이브러리
#include <std_msgs/msg/int32.hpp>       // 정수형 메시지 타입
using namespace std;
using namespace cv;

class LineTracer : public rclcpp::Node {
public:
    LineTracer() : Node("line_tracer") {
        // 이미지 토픽 구독자 생성 (/video/image_raw에서 이미지 수신)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video/image_raw", 10, std::bind(&LineTracer::image_callback, this, std::placeholders::_1));
        // 결과 토픽 발행자 생성 (/line/result로 라인 중심 x좌표 발행)
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/line/result", 10);
        // 라인 추적 기준점 초기화 (처음엔 가운데)
        mainPoint_ = Point(-1, -1);
        // OpenCV 창 생성 (원본 영상, 처리 결과 영상)
        namedWindow("Original Video", WINDOW_NORMAL);
        namedWindow("Line Tracer Result", WINDOW_NORMAL);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROS2 이미지를 OpenCV 이미지로 변환
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            RCLCPP_INFO(this->get_logger(), "Received image");
        } catch (cv_bridge::Exception& e) {
            // 변환 실패 시 에러 출력
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;
        Mat gray, binary;
        // 1. 그레이스케일 변환
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        // 2. 밝기 평균 맞추기 (옵션)
        gray += Scalar(100) - mean(gray);
        // 3. 이진화
        threshold(gray, binary, 128, 255, THRESH_BINARY);
        // 4. ROI 설정 (하단 90픽셀만 사용)
        Rect roi(0, max(0, binary.rows - 90), binary.cols, 90);
        binary = binary(roi);

        // 5. 처음 프레임: 가운데 기준점 설정
        if (mainPoint_.x == -1) {
            mainPoint_ = Point(binary.cols / 2, binary.rows - 1);
        }

        // 6. 연결된 컴포넌트 분석 (레이블링)
        Mat label, stats, centroids;
        int num = connectedComponentsWithStats(binary, label, stats, centroids);
        int closestIndex = -1;
        double minDist = binary.cols;
        // 7. 기준점 근처 라인만 추적
        for (int i = 1; i < num; i++) {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > 100) { // 일정 크기 이상만
                Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
                double dist = abs(center.x - mainPoint_.x);
                if (dist < minDist && dist < 100) { // 오차범위(100픽셀 이내)
                    minDist = dist;
                    closestIndex = i;
                }
            }
        }
        int result = -1;
        if (closestIndex >= 0) {
            Point center(cvRound(centroids.at<double>(closestIndex, 0)), cvRound(centroids.at<double>(closestIndex, 1)));
            mainPoint_ = center;
            result = center.x;
            RCLCPP_INFO(this->get_logger(), "Line found at x: %d", result);
        }
        // 8. 결과 발행 (라인 중심 x좌표)
        auto msg_result = std_msgs::msg::Int32();
        msg_result.data = result;
        publisher_->publish(msg_result);

        // 9. 결과 이미지 시각화 (ROI 영역)
        Mat output = binary.clone();
        cvtColor(output, output, COLOR_GRAY2BGR);

        // 10. 모든 라인(객체) 파란색, 추적 라인 빨간색으로 표시
        for (int i = 1; i < num; i++) {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > 100) {
                Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
                Rect object(stats.at<int>(i, CC_STAT_LEFT), stats.at<int>(i, CC_STAT_TOP),
                            stats.at<int>(i, CC_STAT_WIDTH), stats.at<int>(i, CC_STAT_HEIGHT));
                if (i == closestIndex) {
                    rectangle(output, object, Scalar(0, 0, 255), 2); // 추적 라인: 빨간색
                    circle(output, center, 5, Scalar(0, 0, 255), -1);
                } else {
                    rectangle(output, object, Scalar(255, 0, 0), 2); // 기타 객체: 파란색
                    circle(output, center, 5, Scalar(255, 0, 0), -1);
                }
            }
        }

        // 11. 원본 영상 출력 (자르기 없이)
        imshow("Original Video", frame);
        // 12. 라인 트레이서 결과 출력 (ROI 영역)
        imshow("Line Tracer Result", output);
        waitKey(1); // OpenCV 창이 멈추지 않게 하기 위해 필요
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; // 이미지 구독자
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;          // 결과 발행자
    Point mainPoint_;                                                       // 추적 기준점
};

int main(int argc, char **argv) {
    // ROS2 노드 초기화 및 실행
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTracer>());
    rclcpp::shutdown();
    return 0;
}