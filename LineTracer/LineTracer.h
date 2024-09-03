#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <chrono>
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
struct CameraSettings {
    int frame_width;
    int frame_height;
    int format;
    int fps;
};

void applyGrayWorldWhiteBalance(cv::Mat& src);                                    //ホワイトバランス補正関数
static double pid_control(PID &pid, double error);                                //PIDの誤差計算関数
static void motor_cntrol(double left_motor_speed , double right_motor_speed);     //モータの速度設定関数                                            //旧画像取得関数
//void Hsv(cv::Mat& hsv);                                                           //画像のトリミング
//void RectFrame(cv::Mat& rectframe);                                               //HSV変換関数
static std::tuple<cv::Mat, cv::Mat> RectFrame(const cv::Mat& frame);    
static void createMask(const cv::Mat& hsv, const std::string& color);             //マスク変換関数
static cv::Mat Morphology(const cv::Mat& mask);                                   //モルフォロジー変換関数
static std::tuple<int, int> ProcessContours(const cv::Mat& morphed);              //追従座標計算関数
static void PIDMotor(PID &pid);                                                   //PID走行関数
static void Show(const cv::Mat& showfreme);                                       //画像表示関数
static void set_speed(double BASE_SPEED);
static bool detectCheck(const cv::Mat& morphed, int min_area);                    //輪郭検知関数

extern std::chrono::high_resolution_clock::time_point start_time1;                //経過時間の箱1
extern std::chrono::high_resolution_clock::time_point start_time2;                //経過時間の箱2
extern std::chrono::high_resolution_clock::time_point start_time3;                //経過時間の箱3
static void startTimer(int timer_id);                                             //時間の計測開始関数
static float getTime(int timer_id);                                               //時間の取得関数

extern std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_bounds;
extern bool follow, resetting, resize_on;
extern uint8_t scene;
extern int cX, cY;
extern double left_speed, right_speed;

extern int frame_center; 
#ifdef __cplusplus
}
#endif
