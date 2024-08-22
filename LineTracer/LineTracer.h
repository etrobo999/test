#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/*関数宣言*/
struct PID {
    double Kp, Ki, Kd;
    double previous_error, integral;
};
static double pid_control(PID &pid, double error);
static void Capture(void); 
static void motor_cntrol(double left_motor_speed , double right_motor_speed);
static cv::Mat Capture(void);
static cv::Mat RectFrame(const cv::Mat& frame);
static cv::Mat createMask(const cv::Mat& hsv, const std::string& color);
static cv::Mat Morphology(const cv::Mat& mask);
static void PIDMotor(PID &pid);
static void Show(void);

extern bool follow;
extern uint8_t scene;
extern int cX, cY;
extern double BASE_SPEED;
/* カラーセンサの輝度設定 */
constexpr int WHITE_BRIGHTNESS = 180;
constexpr int BLACK_BRIGHTNESS = 10;

/*カメラの閾値設定*/
constexpr int THRESHOLDVALUE = 25;
constexpr int MAXBINARYVALUE = 255;

/*カメラのトリミング*/
constexpr int TRIMY = 240;
constexpr int TRIMH = 60;

constexpr int frame_center = 180; 

constexpr int ALLB_Y1 = 0;
constexpr int ALLB_Y2 = TRIMH;

/* ステアリング操舵量の係数 */
constexpr float STEERING_COEF = 0.2F;

#ifdef __cplusplus
}
#endif
