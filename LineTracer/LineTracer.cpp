#include <opencv2/opencv.hpp>
#include "app.h"
#include "LineTracer.h"
#include <stdio.h>
#include <iostream>
#include <bitset>

/* 関数プロトタイプ宣言 */
struct PID {
    double Kp, Ki, Kd;
    double previous_error, integral;
};
double pid_control(PID &pid, double error);
static void Capture(void); 
static void motor_cntrol(double left_motor_speed , double right_motor_speed);

PID pid = {0.18, 0.01, 0.01, 0, 0}; 

cv::VideoCapture camera;

/* ライントレースタスク(50msec周期で関数コールされる) */
void tracer_task(intptr_t unused) {
    Capture();
    /* タスク終了 */
    ext_tsk();
}

static void Capture(void){
    int retry_count = 0;
    const int max_retries = 5;
    if (!camera.isOpened()) {
        camera.open(0);
        return;
    }
    cv::Mat frame, hsv, mask, morphed;
    while (retry_count < max_retries) {
        camera >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Frame is empty. Retrying..." << std::endl;
            retry_count++;
            continue;
        }
        break;
    }
    /*std::cout << "Cols: " << frame.cols << ", Rows: " << frame.rows << std::endl;*/
    frame = frame(cv::Rect(140, TRIMY, 500, TRIMH));

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 特定の色範囲を検出（黒いラインを検出）
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), mask);

    // モルフォロジー変換（開操作と閉操作）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, morphed, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(morphed, morphed, cv::MORPH_CLOSE, kernel);

    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // 最大の輪郭を選択
        auto largest_contour = std::max_element(contours.begin(),  contours.end(),
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                return cv::contourArea(a) < cv::contourArea(b);
            }
        );

        // 重心を計算
        cv::Moments M = cv::moments(*largest_contour);
        int cX = static_cast<int>(M.m10 / M.m00);
        int cY = static_cast<int>(M.m01 / M.m00);
        cv::circle(frame, cv::Point(cX, cY), 5, cv::Scalar(255, 0, 0), -1);

        // エラーベースのPID制御
        double error = frame_center - cX;
        double control = pid_control(pid, error);
        double BASE_SPEED = 80.0;
        double left_motor_speed = BASE_SPEED;
        double right_motor_speed = BASE_SPEED;
        // フィードバック制御のためのモータ制御（仮想）
        if (control > 0) {
            left_motor_speed -= control * 2;
        } else if (control < 0) {
            right_motor_speed += control * 2;
        } else {
            left_motor_speed -= control;
            right_motor_speed += control;

        }
        // モータ速度を表示（実際のロボットではここでモータ制御関数を呼び出す）
        std::cout << "Left Motor: " << left_motor_speed << ", Right Motor: " << right_motor_speed << std::endl;
        motor_cntrol(left_motor_speed , right_motor_speed);
    
    }
    cv::imshow("Frame", frame);
    cv::waitKey(1);
    return;
    
}

/* 走行モータ制御 */
static void motor_cntrol(double left_motor_speed , double right_motor_speed){
    // モータ速度を0から100の範囲に制限
    left_motor_speed = std::max(std::min(left_motor_speed, 100.0), -100.0);
    right_motor_speed = std::max(std::min(right_motor_speed, 100.0), -100.0);

    // 実際のモータ制御関数をここで呼び出す
    ev3_motor_set_power(left_motor, left_motor_speed);
    ev3_motor_set_power(right_motor, right_motor_speed);
    return;
}



double pid_control(PID &pid, double error) {
    pid.integral += error;
    double derivative = error - pid.previous_error;
    pid.previous_error = error;
    return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

