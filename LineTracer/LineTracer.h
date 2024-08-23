#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <string>
#include <map>
#ifdef __cplusplus
extern "C" {
#endif

/*関数宣言*/
struct PID {
    double Kp, Ki, Kd;
    double previous_error, integral;
};
static double pid_control(PID &pid, double error);
static void motor_cntrol(double left_motor_speed , double right_motor_speed);
static cv::Mat Capture(void);
static std::tuple<cv::Mat, cv::Mat> RectFrame(const cv::Mat& frame);
static cv::Mat createMask(const cv::Mat& hsv, const std::string& color);
static cv::Mat Morphology(const cv::Mat& mask);
static std::tuple<int, int, cv::Mat> ProcessContours(const cv::Mat& morphed);
static void PIDMotor(PID &pid);
static void Show(void);

extern bool follow;
extern uint8_t scene;
extern int cX, cY;
extern double BASE_SPEED;
constexpr int frame_center = 180; 
#ifdef __cplusplus
}
#endif
