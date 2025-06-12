#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/vector3.hpp> // Vector3 메시지 추가
using namespace std;
using namespace cv;

class LineTracer : public rclcpp::Node {
public:
    LineTracer() : Node("line_tracer") {
        // 이미지 토픽 구독자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/video/image_raw", 10, std::bind(&LineTracer::image_callback, this, std::placeholders::_1));
        // 결과 토픽 발행자 생성 (Vector3 메시지로 변경)
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10);
        mainPoint_ = Point(-1, -1);
        namedWindow("Original Video", WINDOW_NORMAL);
        namedWindow("Line Tracer Result", WINDOW_NORMAL);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            RCLCPP_INFO(this->get_logger(), "Received image");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;
        Mat gray, binary;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        gray += Scalar(100) - mean(gray);
        threshold(gray, binary, 128, 255, THRESH_BINARY);
        Rect roi(0, max(0, binary.rows - 90), binary.cols, 90);
        binary = binary(roi);

        if (mainPoint_.x == -1) {
            mainPoint_ = Point(binary.cols / 2, binary.rows - 1);
        }

        Mat label, stats, centroids;
        int num = connectedComponentsWithStats(binary, label, stats, centroids);
        int closestIndex = -1;
        double minDist = binary.cols;
        for (int i = 1; i < num; i++) {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > 100) {
                Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
                double dist = abs(center.x - mainPoint_.x);
                if (dist < minDist && dist < 100) {
                    minDist = dist;
                    closestIndex = i;
                }
            }
        }
        int result = -1;
        if (closestIndex >= 0) {
            Point center(cvRound(centroids.at<double>(closestIndex, 0)), cvRound(centroids.at<double>(i, 1)));
            mainPoint_ = center;
            result = center.x;
            RCLCPP_INFO(this->get_logger(), "Line found at x: %d", result);
        }
        
        // 라인 중심 x좌표를 바탕으로 left, right 속도 계산
        double left_speed = 100.0; // 예시: 기본값
        double right_speed = 100.0;
        if (result != -1) { // 라인 감지 시
            double center_x = result;
            double mid_x = binary.cols / 2.0;
            double offset = (center_x - mid_x) / mid_x; // -1~1 범위로 정규화
            left_speed = 100.0 + offset * 50.0; // 예시: 좌우 속도 조절
            right_speed = 100.0 - offset * 50.0;
        }

        // Vector3 메시지로 발행
        auto msg_result = geometry_msgs::msg::Vector3();
        msg_result.x = left_speed;  // 왼쪽 바퀴 속도
        msg_result.y = right_speed; // 오른쪽 바퀴 속도
        publisher_->publish(msg_result);

        // 시각화 부분은 그대로
        Mat output = binary.clone();
        cvtColor(output, output, COLOR_GRAY2BGR);
        for (int i = 1; i < num; i++) {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > 100) {
                Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
                Rect object(stats.at<int>(i, CC_STAT_LEFT), stats.at<int>(i, CC_STAT_TOP),
                            stats.at<int>(i, CC_STAT_WIDTH), stats.at<int>(i, CC_STAT_HEIGHT));
                if (i == closestIndex) {
                    rectangle(output, object, Scalar(0, 0, 255), 2);
                    circle(output, center, 5, Scalar(0, 0, 255), -1);
                } else {
                    rectangle(output, object, Scalar(255, 0, 0), 2);
                    circle(output, center, 5, Scalar(255, 0, 0), -1);
                }
            }
        }
        imshow("Original Video", frame);
        imshow("Line Tracer Result", output);
        waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_; // Vector3 발행자로 변경
    Point mainPoint_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTracer>());
    rclcpp::shutdown();
    return 0;
}
