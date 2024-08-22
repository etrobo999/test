#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/* PID構造体の宣言 */
struct PID {
    double Kp, Ki, Kd;
    double previous_error, integral;
};

/* C言語用の関数宣言（C++の関数には不要） */
void motor_cntrol(double left_motor_speed, double right_motor_speed);

#ifdef __cplusplus
}
#endif

double pid_control(PID &pid, double error);
void controlMotorsWithPID(PID &pid);
void Capture(void); 
cv::Mat Capture(void);
cv::Mat RectFrame(const cv::Mat& frame);
cv::Mat createMask(const cv::Mat& hsv, const std::string& color);
cv::Mat Morphology(const cv::Mat& mask);
void PIDMotor(PID &pid);
void Show(void);

extern bool follow;
extern uint8_t scene;
extern int cX, cY;
extern double BASE_SPEED;

/* カラーセンサの輝度設定 */
constexpr int WHITE_BRIGHTNESS = 180;
constexpr int BLACK_BRIGHTNESS = 10;

/* カメラの閾値設定 */
constexpr int THRESHOLDVALUE = 25;
constexpr int MAXBINARYVALUE = 255;

/* カメラのトリミング */
constexpr int TRIMY = 240;
constexpr int TRIMH = 60;

constexpr int frame_center = 180; 

constexpr int ALLB_Y1 = 0;
constexpr int ALLB_Y2 = TRIMH;

/* ステアリング操舵量の係数 */
constexpr float STEERING_COEF = 0.2F;
