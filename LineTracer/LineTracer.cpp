#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include "app.h"
#include "LineTracer.h"
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <sched.h>
#include <signal.h>
#include <pthread.h>
#include <mutex>
#include <condition_variable>

using namespace std;
using namespace cv;

raspicam::RaspiCam_Cv Camera;

/*PIDインスタンス生成*/
PID straightpid = {0.08, 0, 0.012, 0, 0}; //ストレートPID
PID Bcurvetpid = {0.14, 0, 0, 0, 0}; //急カーブPID
PID Mcurvetpid = {0.11, 0.004, 0, 0, 0}; //ちょうどいいカーブPID
PID Scurvetpid = {0.10, 0.002, 0, 0, 0}; //ゆっくりカーブPID
PID gacurvetpid = {0.16, 0.0005, 0, 0, 0}; //ゆっくりカーブPID

/*rectの値初期化*/
//int rect_x = 100;
//int rect_y = 180;
//int rect_width = 440;
//int rect_height = 160;

int rect_x = 100;
int rect_y = 110;
int rect_width = 440;
int rect_height = 230;


/*cameraの初期設定*/
CameraSettings camera_settings = {640, 480, CV_8UC3, 40};


/*使用する変数の宣言*/
std::chrono::high_resolution_clock::time_point start_time1;
std::chrono::high_resolution_clock::time_point start_time2;
std::chrono::high_resolution_clock::time_point start_time3;

std::mutex mtx;
std::mutex mtx2;
std::mutex mtx3;
std::mutex mtx4;
std::condition_variable frame_ready_var;
std::condition_variable wb_var;
std::condition_variable display_var;
std::condition_variable contour_var;

/*使用する（かもしれない）cv::MATの変数宣言*/
Mat orizin_frame, frame, rectframe, hsv, mask, mask1, mask2, morphed, morphed1, morphed2, result_frame;

/*使用する変数の初期化*/
uint8_t scene = 1;
uint8_t _scene = 0;
int frame_center = 220;
int cX = 0;
int cY = 0;
double left_motor_factor = 1.06209;
double right_motor_factor = 1.0;
double left_speed = 0.0;
double right_speed = 0.0;
bool display_show = true;

//センサーの値を入れる変数
bool touch_sensor_bool = false;
int left_motor_counts = 0;
int right_motor_counts = 0;
int _left_motor_counts = 0;
int _right_motor_counts = 0;
int16_t gyro_counts = 0;
int16_t _gyro_counts = 0;

// 追従方向の変数[true = 左] [false = 右]
bool follow = false;
bool resize_on = false;

// スレッドの操作のための変数
bool resetting = false;
bool frame_ready = false;
bool wb_ready = false;
bool display_ready = false;
bool contour_ready = false;

// 連続して検知された回数をカウントする変数
int detection_count = 0;
int no_detection_count = 0;
int stop_count = 0;

//////////////////////////////////////////////////////////////////////
////////　　　     特殊なcamera処理　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

void* opencv_thread_func(void* arg) {
    // シグナルマスクの設定
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    while (true) {
        // カメラ初期化 (設定を使用)
        Camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_settings.frame_width);
        Camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_settings.frame_height);
        Camera.set(cv::CAP_PROP_FORMAT, camera_settings.format);
        Camera.set(cv::CAP_PROP_FPS, camera_settings.fps);
        if (!Camera.open()) {
            cerr << "Error: !Camera.open" << endl;
            pthread_exit(NULL);
        }

        while (true) {
            //startTimer(2);
            Camera.grab();
            Mat temp_frame;
            Camera.retrieve(temp_frame);

            if (temp_frame.empty()) {
                cerr << "frame.empty" << endl;
                continue;
            }
            // 取得したフレームを共有変数にコピー
            {
                std::lock_guard<std::mutex> lock(mtx);
                temp_frame.copyTo(frame);
                frame_ready = true;
            }
            // メインタスクにフレームが準備できたことを通知
            frame_ready_var.notify_one();

            // ここでカメラ設定が変更されたかをチェック
            if (resetting) {
                Camera.release();  // カメラをリリース
                resetting = false;
                break;  // 内側のループを抜けて再初期化へ
            }
            //cout << "camera "  << getTime(2) <<endl;
        }
    }

    pthread_exit(NULL);
}

/*void* white_balance_thread_func(void* arg) {
    // シグナルマスクの設定
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    while (true) {
        Mat temp_frame1;
        
        // フレームが準備されるまで待機
        {
            std::unique_lock<std::mutex> lock(mtx2);
            frame_ready_var.wait(lock, [] { return frame_ready; });
            startTimer(3);
            temp_frame1 = orizin_frame.clone(); // フレームをコピーしてローカルで処理
        }
        
        if (resize_on) {
            cv::resize(temp_frame1, temp_frame1, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        }
        
        applyGrayWorldWhiteBalance(temp_frame1);

        // 処理したフレームを戻す
        {
            std::lock_guard<std::mutex> lock(mtx2);
            temp_frame1.copyTo(frame);
            frame_ready = false;
            wb_ready = true;
        }
        cout << "WB "  << getTime(3) <<endl;
        
        // 次の処理をメインスレッドに通知
        wb_var.notify_one();
    }

    pthread_exit(NULL);
}
*/

// scene を更新する関数
void* contour_thread_func(void* arg) {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    int min_area = 1500;

    while (true) {
        // contour_var が通知されるまで待機
        {
            std::unique_lock<std::mutex> lock(mtx2);
            contour_var.wait(lock, [] { return contour_ready; });  // notify_oneで再開される
            morphed1 = Morphology(mask1);//青色モル
        }

        // 輪郭検知処理
        bool is_right_side, is_left_side;
        std::tie(is_right_side, is_left_side) = detectRectangleAndPosition(mask1, min_area);
        // 左右の検知結果によってシーンを更新
        if (is_right_side) {
            follow = false;
            left_motor_counts += ev3_motor_get_counts(left_motor);
            right_motor_counts += ev3_motor_get_counts(right_motor);
            reset_left_motor();
            reset_right_motor();
            reset_gyro_sensor();
            _scene = scene; 
            //scene = 38;
        } else if (is_left_side) {
            follow = true;
            left_motor_counts = ev3_motor_get_counts(left_motor);
            right_motor_counts = ev3_motor_get_counts(right_motor);
            reset_left_motor();
            reset_right_motor();
            reset_gyro_sensor();
            _scene = scene; 
            scene = 51;
        }

        // 処理が終わったら contour_ready をリセット
        contour_ready = false;
    }

    pthread_exit(NULL);
}

void* display_thread_func(void* arg) {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    while (true) {
        // フレームが表示できるまで待機
        Mat temp_frame1;
        {
            std::unique_lock<std::mutex> lock(mtx4);
            display_var.wait(lock, [] { return display_ready; });
            result_frame.copyTo(temp_frame1);
        }

        //temp_frame1 = result_frame.clone();
        cv::imshow("temp_frame1", temp_frame1);
        cv::waitKey(1);

        {
            display_ready = false;
        }
    }

    pthread_exit(NULL);
}

//////////////////////////////////////////////////////////////////////
////////　　　         メイン処理　  　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

void* main_thread_func(void* arg) {
    set_cpu_affinity(0);
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    pthread_t opencv_thread;
//    pthread_t white_balance_thread;
    pthread_t contour_thread;
    pthread_t display_thread;

    // OpenCVスレッドを作成
    if (pthread_create(&opencv_thread, NULL, opencv_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create OpenCV thread" << endl;
        pthread_exit(NULL);
    }
    
/*          // ホワイトバランス処理スレッドを作成
    if (pthread_create(&white_balance_thread, NULL, white_balance_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create White Balance thread" << endl;
        pthread_exit(NULL);
    }
*/
    // 輪郭検知のマルチ処理
    if (pthread_create(&contour_thread, NULL, contour_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create contour thread" << endl;
        pthread_exit(NULL);
    }


    // 画面表示スレッドを作成
    if (pthread_create(&display_thread, NULL, display_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create Display thread" << endl;
        pthread_exit(NULL);
    }

/*    // モータのキャリブレーション

    {
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        motor_cntrol(50, 50);  // 両方のモータを同じ速度で動かす
        cv::waitKey(6000);
        motor_cntrol(0, 0);
        left_motor_counts = ev3_motor_get_counts(left_motor);
        right_motor_counts = ev3_motor_get_counts(right_motor);
        if (left_motor_counts > right_motor_counts) {
            // 左モータの回転数が多い場合、右モータの出力を上げる
            right_motor_factor = (double)left_motor_counts / right_motor_counts;
        } else if (right_motor_counts > left_motor_counts) {
            // 右モータの回転数が多い場合、左モータの出力を上げる
            left_motor_factor = (double)right_motor_counts / left_motor_counts;
        }
        std::cout << "Calibration complete. Left factor: " << left_motor_factor << ", Right factor: " << right_motor_factor << std::endl;
    }
*/
    bool ext = true;

    while (ext) {
        std::unique_lock<std::mutex> lock(mtx3);
        frame_ready_var.wait(lock, [] { return frame_ready; });
        //wb_ready = false;
        switch (scene) {
//////////////////////////////////////////////////////////////////////
////////　　　　　　スタート処理　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 1: //画面表示・ボタンでスタート
            startTimer(1);
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black");
            //bitwise_not(mask2, mask2);
            morphed = Morphology(mask2);
            tie(cX, cY) = Follow_1(morphed);
            console_PL();
            cout << getTime(1) << endl;
            if(ev3_touch_sensor_is_pressed(touch_sensor)){
                scene++;
            };
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            //std::cout << ev3_gyro_sensor_get_angle(gyro_sensor) << std::endl;
            //std::cout << ev3_touch_sensor_is_pressed(touch_sensor) << std::endl;
            break;
        case 2:
            rect_x = 100;
            rect_y = 180;
            rect_width = 440;
            rect_height = 160;
            reset_left_motor();
            reset_right_motor();
            reset_gyro_sensor();
            console_PL();
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            scene = 11;
            break;
        case 3:
            startTimer(1);
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "white");
            //bitwise_not(mask, mask);
            morphed = Morphology2(mask);
            tie(cX, cY) = Follow_2(morphed);
            console_PL();
            cout << getTime(1) << endl;
            if(ev3_touch_sensor_is_pressed(touch_sensor)){
                scene++;
            };
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            //std::cout << ev3_gyro_sensor_get_angle(gyro_sensor) << std::endl;
            //std::cout << ev3_touch_sensor_is_pressed(touch_sensor) << std::endl;
            break;
        case 4:
            rect_x = 100;
            rect_y = 180;
            rect_width = 440;
            rect_height = 160;
            reset_left_motor();
            reset_right_motor();
            reset_gyro_sensor();
            console_PL();
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            scene = 31;
            break;
        case 5:
            std::cout << ev3_gyro_sensor_reset(gyro_sensor) << std::endl;
            cv::waitKey(30);
            std::cout << ev3_motor_reset_counts(right_motor) << std::endl;
            cv::waitKey(30);
            std::cout << ev3_motor_reset_counts(left_motor) << std::endl;
            cv::waitKey(30);
            break;
        case 6:
            startTimer(1);
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black");
            //bitwise_not(mask2, mask2);
            morphed = Morphology(mask2);
            tie(cX, cY) = Follow_4(morphed);
            console_PL();
            cout << getTime(1) << endl;
            if(ev3_touch_sensor_is_pressed(touch_sensor)){
                scene++;
            };
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            //std::cout << ev3_gyro_sensor_get_angle(gyro_sensor) << std::endl;
            //std::cout << ev3_touch_sensor_is_pressed(touch_sensor) << std::endl;
            break;
        case 7:
            rect_x = 100;
            rect_y = 180;
            rect_width = 440;
            rect_height = 160;
            reset_left_motor();
            reset_right_motor();
            reset_gyro_sensor();
            console_PL();
            std::cout << "gyro " << ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            scene = 11;
            break;
        case 8:
        case 9:
        case 10:
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　難所前ライントレース　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 11: //設定の読み込み
            startTimer(2);
            startTimer(1);
            set_speed(70.0);
            follow = !follow;
            scene++;
            break;
        case 12: //第一ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(straightpid);
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 6350){
                scene++;
            }
            console_PL();
            break;
        case 13: //設定の読み込み
            startTimer(1);
            set_speed(55.0);
            scene++;
            break;
        case 14: //第一急カーブ
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(Bcurvetpid);         
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 7600){
                scene++;
            }
            console_PL();
            break;
        case 15: //設定の読み込み
            startTimer(1);
            set_speed(55.0);
            follow = !follow;
            scene++;
            break;
        case 16: //第二ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(straightpid);
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 11700){
                scene++;
            }
            console_PL();
            break;
        case 17://設定の読み込み
            follow = !follow;
            startTimer(1);
            set_speed(50.0);
            scene++;
            break;
        case 18: //第二急カーブ
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(Bcurvetpid);
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 12500){
                scene++;
            }
            console_PL();
            break;
        case 19://設定の読み込み
            follow = !follow;
            startTimer(1);
            set_speed(60.0);
            scene++;
            break;
        case 20: //第三ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(straightpid);
            if(getTime(1) >=1){
                scene = 21;
            }
            console_PL();
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　第一難所　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 21://設定の読み込み
            set_speed(45.0);
            scene++;
            break;
        case 22://シーン1
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black"); //Mask,Mask1
            morphed = Morphology(mask);
            morphed1 = Morphology(mask1); //青色モル
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(straightpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            console_PL();
            break;
        case 23://設定の読み込み
            set_speed(45.0);
            follow = !follow;
            scene++;
            std::cout << follow << std::endl;
            break;
        case 24://シーン2
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black"); //Mask,Mask1
            morphed = Morphology(mask);
            morphed1 = Morphology(mask1); //青色モル
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(Bcurvetpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            console_PL();
            break;
        case 25://設定の読み込み
            set_speed(45.0);
            follow = !follow;
            scene++;
            std::cout << follow << std::endl;
            break;
        case 26://シーン3
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black"); //Mask,Mask1
            morphed = Morphology(mask);
            morphed1 = Morphology(mask1); //青色モル
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(Bcurvetpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            console_PL();
            break;
        case 27://設定の読み込み
            set_speed(45.0);
            follow = !follow;
            scene++;
            std::cout << follow << std::endl;
            break;
        case 28://シーン4
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black"); //Mask,Mask1
            morphed = Morphology(mask);
            morphed1 = Morphology(mask1); //青色モル
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(Bcurvetpid);
            if(detectCheck(morphed1,2000)){
                motor_cntrol(0,0);
                scene = 31;
            }
            console_PL();
            break;
        case 29://設定の読み込み
            startTimer(1);
            set_speed(45.0);
            follow = !follow;
            scene++;
            std::cout << follow << std::endl;
            break;
        case 30:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            PIDMotor(straightpid);
            console_PL();
            if(getTime(1) >=3){
                scene++;
                motor_cntrol(0,0);
            }
            
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　第三難所　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 31://設定の読み込み
            rect_x = 100;
            rect_y = 110;
            rect_width = 440;
            rect_height = 230;
            set_speed(40.0);
            scene++;
            std::cout << "Case 31" << std::endl;
            break;
        case 32:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black");
            morphed = Morphology(mask2);
            tie(cX, cY) = Follow_4(morphed);
            PIDMotor(Bcurvetpid);
            console_PL();
            break;
        case 33:
            motor_cntrol(50,50);
            if (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 1300) {
                {
                    set_speed(45.0);
                    scene++;
                }
            }
            console_PL();
            break;
        case 34:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_white"); //Mask,Mask1
            contour_ready = true;
            contour_var.notify_one();
            //bitwise_not(mask2, mask2);//白黒反転
            morphed = Morphology2(mask2);//白色モル
            tie(cX, cY) = Follow_2(morphed);
            PIDMotor(Bcurvetpid);
            console_PL();
            while (contour_ready) {
                cv::waitKey(10);
            }
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) + left_motor_counts + right_motor_counts >= 4200){
                {
                    motor_cntrol(0,0);
                    set_speed(-45.0);
                    scene++;
                }
            }
            break;
        case 35:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_white"); //Mask,Mask1
            //bitwise_not(mask2, mask2);//白黒反転
            morphed = Morphology2(mask2);//白色モル
            tie(cX, cY) = Follow_2(morphed);
            PIDMotor(Bcurvetpid);
            console_PL();
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) + left_motor_counts + right_motor_counts <= 1300){
                {
                    motor_cntrol(0,0);
                    set_speed(45.0);
                    reset_gyro_sensor();
                    scene++;
                }
            }
            break;
        case 36:
            gyro_counts = ev3_gyro_sensor_get_angle(gyro_sensor);
            if (gyro_counts < 85){
                motor_cntrol(50,-50);
            } else if (gyro_counts >= 85){
                {
                    motor_cntrol(0,0);
                    reset_left_motor();
                    reset_right_motor();
                    left_motor_counts = 0;
                    right_motor_counts = 0;
                    frame_center = 100;
                    scene++;
                }
            }
            console_PL();
            std::cout << "gyro " << gyro_counts<< std::endl;
            break;
        case 37:
             tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_white"); //Mask,Mask1
            morphed = Morphology2(mask2);//白色モル
            bitwise_not(morphed, morphed);
            tie(cX, cY) = Follow_2(morphed);
            PIDMotor(Bcurvetpid);
            console_PL();
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 1800){
                {
                    motor_cntrol(0,0);
                    reset_gyro_sensor();
                    scene++;
                }
            }
            break;
        case 38:
            gyro_counts = ev3_gyro_sensor_get_angle(gyro_sensor);
            if (gyro_counts > -80){
                motor_cntrol(-50,50);
            } else if (gyro_counts <= -80){
                {
                    motor_cntrol(0,0);
                    reset_left_motor();
                    reset_right_motor();
                    left_motor_counts = 0;
                    right_motor_counts = 0;
                    frame_center = 320;
                    scene++;
                }
                
            }
            console_PL();
            std::cout << "gyro " << gyro_counts<< std::endl;
            break;
        case 39:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_white"); //Mask,Mask1
            contour_ready = true;
            contour_var.notify_one();
            //bitwise_not(mask2, mask2);//白黒反転
            morphed = Morphology2(mask2);//白色モル
            tie(cX, cY) = Follow_2(morphed);
            PIDMotor(Bcurvetpid);
            console_PL();
            while (contour_ready) {
                cv::waitKey(10);
            }
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) + left_motor_counts + right_motor_counts >= 1600){
                {
                    motor_cntrol(0,0);
                    set_speed(-50.0);
                    scene++;
                }
            }
            break;
        case 40:
            break;
        case 41:
            break;
        case 42:
            std::cout << "Case 42" << std::endl;
            break;
        case 43:
            std::cout << "Case 43" << std::endl;
            break;
        case 44:
            std::cout << "Case 44" << std::endl;
            break;
        case 45:
            std::cout << "Case 45" << std::endl;
            break;
        case 46:
            std::cout << "Case 46" << std::endl;
            break;
        case 47:
            std::cout << "Case 47" << std::endl;
            break;
        case 48:
            std::cout << "Case 48" << std::endl;
            break;
        case 49:
            std::cout << "Case 49" << std::endl;
            break;
        case 50:
            std::cout << "Case 50" << std::endl;
            ext = false;
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　例外特殊ケース　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 51:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue"); //Mask,Mask1
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_3(morphed);
            console_PL();
            PIDMotor(Bcurvetpid);
            if (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 500){
                scene++;
            }
            break;
        case 52:
            motor_cntrol(-50,-60);
            console_PL();
            if (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) <= 30){
                motor_cntrol(0,0);
                scene = _scene;
            }
            break;
        case 53:
            if (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) > 0){
                motor_cntrol(-50,-50);
                while (true) {
                    if (ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) <= 0){
                        scene++;
                        break;
                    }
                    cv::waitKey(30);
                    console_PL();
                }
            }
            break;
        case 54:
            gyro_counts = ev3_gyro_sensor_get_angle(gyro_sensor);
            if (gyro_counts < 0){
                motor_cntrol(50,-50);
                while (true) {
                    if (gyro_counts >= 0){
                        scene = _scene;
                        motor_cntrol(0,0);
                        reset_left_motor();
                        reset_right_motor();
                        break;
                    }
                    cv::waitKey(30);
                    gyro_counts = ev3_gyro_sensor_get_angle(gyro_sensor);
                    console_PL();
                    std::cout << "gyro " << gyro_counts<< std::endl;
                }
            }
            break;
        case 55:
            std::cout << "Case 55" << std::endl;
            break;
        case 56:
            std::cout << "Case 56" << std::endl;
            break;
        case 57:
            std::cout << "Case 57" << std::endl;
            break;
        case 58:
            std::cout << "Case 58" << std::endl;
            break;
        case 59:
            std::cout << "Case 59" << std::endl;
            break;
        case 60:
            std::cout << "Case 60" << std::endl;
            break;
        default:
            std::cout << "Default case" << std::endl;
            break;
        }
        display_ready = true;
        display_var.notify_one();
    }
    pthread_exit(NULL);
}


void tracer_task(intptr_t unused) {
    pthread_t main_thread;
    if (pthread_create(&main_thread, NULL, main_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create Main thread" << endl;
        pthread_exit(NULL);
    }
    ext_tsk(); // タスクを終了
}


/* ホワイトバランス補正 */
void applyGrayWorldWhiteBalance(Mat& src) {
    vector<Mat> channels(3);
    split(src, channels);

    // 各チャンネルの最小値と最大値を取得
    double min_r, max_r, min_g, max_g, min_b, max_b;
    minMaxLoc(channels[2], &min_r, &max_r);
    minMaxLoc(channels[1], &min_g, &max_g);
    minMaxLoc(channels[0], &min_b, &max_b);

    // 各チャンネルを正規化してスケーリング (範囲を0～255に再スケーリング)
    channels[2] = (channels[2] - min_r) * (255.0 / (max_r - min_r));
    channels[1] = (channels[1] - min_g) * (255.0 / (max_g - min_g));
    channels[0] = (channels[0] - min_b) * (255.0 / (max_b - min_b));

    // チャンネルを再結合
    merge(channels, src);
}

static tuple<Mat, Mat>  RectFrame(const Mat& frame) {
    Mat rectframe, hsv;
    rectframe = frame.clone();    
    rectframe = rectframe(Rect(rect_x, rect_y, rect_width, rect_height));
    medianBlur(rectframe, hsv, 5);
    cvtColor(hsv, hsv, COLOR_BGR2HSV);
    return make_tuple(rectframe, hsv);
}

/* フレームのトリミング 
void RectFrame(Mat& rectframe) {
    rectframe = rectframe(Rect(120, 180, 400, 160));
}

/* HSV変換 
void Hsv(Mat& hsv) {
    cvtColor(hsv, hsv, COLOR_BGR2HSV);
};*/

/* マスク変換 */
static void createMask(const Mat& hsv, const std::string& color) {
    if (color == "red") {
        inRange(hsv, color_bounds["red_low"].first, color_bounds["red_low"].second, mask1);
        inRange(hsv, color_bounds["red_high"].first, color_bounds["red_high"].second, mask2);
        mask = mask1 | mask2;  // 両方のマスクを統合
    } else if (color == "blue_black") {
        inRange(hsv, color_bounds["blue"].first, color_bounds["blue"].second, mask1);
        inRange(hsv, color_bounds["black"].first, color_bounds["black"].second, mask2);
        mask = mask1 | mask2;  // 両方のマスクを統合
    } else if (color == "blue_white") {
        inRange(hsv, color_bounds["blue"].first, color_bounds["blue"].second, mask1);
        inRange(hsv, color_bounds["white"].first, color_bounds["white"].second, mask2);
        mask = mask1 | mask2;  // 両方のマスクを統合
    // 青と黒のマスクを統合
    } else {
        inRange(hsv, color_bounds[color].first, color_bounds[color].second, mask);
    }
}


/* モルフォロジー変換 */
static Mat Morphology(const Mat& mask) {
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, morphed, MORPH_OPEN, kernel);
    morphologyEx(morphed, morphed, MORPH_CLOSE, kernel);    
    return morphed;  // モルフォロジー変換後の画像を返す
}

static Mat Morphology2(const Mat& mask) {
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, morphed, MORPH_OPEN, kernel);
    bitwise_not(morphed, morphed);
    dilate(morphed, morphed, kernel);
    morphologyEx(morphed, morphed, MORPH_CLOSE, kernel);   
    bitwise_not(morphed, morphed); 
    return morphed;  // モルフォロジー変換後の画像を返す
}


/*ライン切り替え追従関数*/
static std::tuple<int, int> Follow_1(cv::Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    const double min_contour_area = 3000.0; // ピクセル数

    // 最大の輪郭と2番目に大きい輪郭を見つける
    std::vector<cv::Point>* largest_contour = nullptr;
    std::vector<cv::Point>* second_largest_contour = nullptr;
    double largest_area = min_contour_area;
    double second_largest_area = min_contour_area;

    for (auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= largest_area) {
            second_largest_area = largest_area;
            second_largest_contour = largest_contour;

            largest_area = area;
            largest_contour = &contour;
        } else if (area >= second_largest_area) {
            second_largest_area = area;
            second_largest_contour = &contour;
        }
    }

    int cX = 0, cY = 0;
    std::vector<cv::Point>* target_contour = nullptr;

    // 輪郭が1つしかない場合、その輪郭を重心で分割し、2つの輪郭として扱う
    if (largest_contour && second_largest_contour == nullptr) {
        stop_count = 0;
        // 重心を計算
        cv::Moments M = cv::moments(*largest_contour);
        int cx = static_cast<int>(M.m10 / M.m00);

        // 最大の輪郭以外を黒く塗りつぶす
        Mat mask = Mat::zeros(morphed.size(), CV_8UC1);  // 真っ黒なマスクを作成
        std::vector<std::vector<cv::Point>> fillContours = {*largest_contour};
        
        // 最大の輪郭を白で塗りつぶす
        drawContours(mask, fillContours, -1, Scalar(255), FILLED);

        // morphedにmaskを適用し、最大の輪郭だけを残す
        morphed.setTo(Scalar(0));  // morphed全体を黒くする
        drawContours(morphed, fillContours, -1, Scalar(255), FILLED);  // 最大輪郭のみ白で描画

        // 垂直な線を描画（疑似的に2つの輪郭があるようにする）
        cv::line(morphed, cv::Point(cx, 0), cv::Point(cx, morphed.rows), cv::Scalar(0), 2);

        // 再度輪郭を抽出
        contours.clear();
        findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        std::cout << "Number of contours found: " << contours.size() << std::endl;

        largest_contour = nullptr;
        second_largest_contour = nullptr;

        if (contours.size() >= 2) {
            largest_contour = &contours[0];
            second_largest_contour = &contours[1];
        } else if (contours.size() == 1) {
            largest_contour = &contours[0];
            second_largest_contour = nullptr;  // 2つ目の輪郭がない場合はnull
        }
    }

    // 2つの輪郭が存在する場合の通常処理
    if (largest_contour && second_largest_contour) {
        stop_count = 0;
        cv::Moments M1 = cv::moments(*largest_contour);
        cv::Moments M2 = cv::moments(*second_largest_contour);
        int cX1 = static_cast<int>(M1.m10 / M1.m00);
        int cX2 = static_cast<int>(M2.m10 / M2.m00);

        if (follow) {
            target_contour = (cX1 < cX2) ? largest_contour : second_largest_contour;
        } else {
            target_contour = (cX1 > cX2) ? largest_contour : second_largest_contour;
        }

        // 選択された輪郭の重心を計算
        cv::Moments M = cv::moments(*target_contour);
        cX = static_cast<int>(M.m10 / M.m00);
        cY = static_cast<int>(M.m01 / M.m00);
    } else {
        stop_count++;
    }

    // 重心を描画
    result_frame = morphed.clone(); // 描画用にフレームをコピー
    cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);


    // 結果をタプルで返す (重心のx座標, y座標, 描画済みフレーム)
    return std::make_tuple(cX, cY);
}

static std::tuple<int, int> Follow_4(Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::cout << "Number of contours found: " << contours.size() << std::endl;

    const double min_contour_area = 2000.0; // ピクセル数
    // 最大の輪郭と2番目に大きい輪郭を見つける
    std::vector<cv::Point>* best_contour = nullptr;
    double best_score = std::numeric_limits<double>::max(); // 中心に近い輪郭を優先するため、最初は大きな値を設定

    for (auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= min_contour_area) {
            // 重心を計算
            cv::Moments M = cv::moments(contour);
            int cX = static_cast<int>(M.m10 / M.m00);

            // 中心に近いほど優先 (x座標の距離の二乗)
            double distance_to_center = std::pow(cX - frame_center, 2);

            // 優先度を調整（左右優先の切り替え）
            if (!follow && cX > frame_center) {
                distance_to_center *= 2; // 右側なら優先度を下げる
            } else if (follow && cX < frame_center) {
                distance_to_center *= 2; // 左側なら優先度を下げる
            }

            // 中心に近い輪郭を選択
            if (distance_to_center < best_score) {
                best_score = distance_to_center;
                best_contour = &contour;
            }
        }
    }

    int cX = 0, cY = 0;
    if (best_contour) {
        stop_count = 0;

        // 選択された輪郭の重心を計算
        cv::Moments M = cv::moments(*best_contour);
        cX = static_cast<int>(M.m10 / M.m00);
        cY = static_cast<int>(M.m01 / M.m00);

        // 重心を描画
        result_frame = morphed.clone(); // 描画用にフレームをコピー
        cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
    } else {
        stop_count++;
    }

    // 結果をタプルで返す (重心のx座標, y座標)
    return std::make_tuple(cX, cY);
}

/*最大の輪郭追従関数*/
static std::tuple<int, int> Follow_2(const Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::cout << "Number of contours found: " << contours.size() << std::endl;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        std::cout << "Contour " << i << " area: " << area << std::endl;
    }

    const double min_contour_area = 3000.0; // ピクセル数

    // 最大の輪郭を見つける
    std::vector<cv::Point>* largest_contour = nullptr;
    double largest_area = min_contour_area;

    for (auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= largest_area) {
            largest_area = area;
            largest_contour = &contour;
        }
    }

    int cX = 0, cY = 0;
    // 有効な輪郭が少なくとも1つある場合に処理を行う
    if (largest_contour) {
        stop_count = 0;

        // 最大の輪郭の重心を計算
        cv::Moments M = cv::moments(*largest_contour);
        cX = static_cast<int>(M.m10 / M.m00);
        cY = static_cast<int>(M.m01 / M.m00);
    } else {
        stop_count++;
    }

    result_frame = morphed.clone(); // 描画用にフレームをコピー
    cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
    
    // 結果をタプルで返す (重心のx座標, y座標, 描画済みフレーム)
    return std::make_tuple(cX, cY);
}

/*青ペットボトル検知*/
std::tuple<bool, bool> detectRectangleAndPosition(const Mat& morphed, int min_area) {
    // 輪郭を格納するためのベクタ
    std::vector<std::vector<cv::Point>> contours;

    // 画像から輪郭を抽出する
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    bool large_rectangle_detected = false;
    bool is_right_side = false;
    bool is_left_side = false;

    for (const auto& contour : contours) {
        // 輪郭の面積を計算
        double area = contourArea(contour);

        // 面積が閾値以上の輪郭のみを対象にする
        if (area >= min_area) {
            // 輪郭をポリゴン近似し、頂点数を確認する
            std::vector<cv::Point> approx;
            approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

            // 頂点の数が4つであれば四角形とみなす
            if (approx.size() == 4) {
                // 重心を計算して、右側か左側かを判定する
                cv::Moments M = moments(contour);
                int cX = static_cast<int>(M.m10 / M.m00);  // 重心のX座標を計算

                // 画面の中心を基準に左右どちらかを判定
                if (cX > morphed.cols / 2) {
                    is_right_side = true;
                } else {
                    is_left_side = true;
                }

                // 四角形が見つかったら、続行せずにループを抜ける
                break;
            }
        }
    }

    // 四角形が検出され、右側・左側のどちらかを返す
    return std::make_tuple(is_right_side, is_left_side);
}

/* 四角形の輪郭を追従し、最大の輪郭を二つまで保持 */
static std::tuple<int, int> Follow_3(const Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::cout << "Number of contours found: " << contours.size() << std::endl;

    const double min_contour_area = 2500.0; // ピクセル数

    // 最大の輪郭と2番目に大きい輪郭を見つける
    std::vector<cv::Point>* largest_contour = nullptr;
    std::vector<cv::Point>* second_largest_contour = nullptr;
    double largest_area = min_contour_area;
    double second_largest_area = min_contour_area;

    // 輪郭を走査し、四角形のみを対象にする
    for (auto& contour : contours) {
        double area = contourArea(contour);
        std::vector<cv::Point> approx;

        // ポリゴン近似で輪郭を四角形として認識できるか確認
        approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

        // 四角形（頂点が4つ）のみを対象とする
        if (approx.size() == 4 && area >= min_contour_area) {
            if (area >= largest_area) {
                second_largest_area = largest_area;
                second_largest_contour = largest_contour;

                largest_area = area;
                largest_contour = &contour;
            } else if (area >= second_largest_area) {
                second_largest_area = area;
                second_largest_contour = &contour;
            }
        }
    }

    int cX = 0, cY = 0;
    // 有効な輪郭が少なくとも1つある場合に処理を行う
    if (largest_contour) {
        stop_count = 0;
        std::vector<cv::Point>* target_contour;

        // 2つの輪郭が存在し、followがtrueならxが小さい方、falseならxが大きい方を選ぶ
        if (second_largest_contour) {
            cv::Moments M1 = cv::moments(*largest_contour);
            cv::Moments M2 = cv::moments(*second_largest_contour);
            int cX1 = static_cast<int>(M1.m10 / M1.m00);
            int cX2 = static_cast<int>(M2.m10 / M2.m00);

            if (follow) {
                target_contour = (cX1 < cX2) ? largest_contour : second_largest_contour;
            } else {
                target_contour = (cX1 > cX2) ? largest_contour : second_largest_contour;
            }
        } else {
            // 輪郭が1つしかない場合、それを使用
            target_contour = largest_contour;
        }

        // 選択された輪郭の重心を計算
        cv::Moments M = cv::moments(*target_contour);
        cX = static_cast<int>(M.m10 / M.m00);
        cY = static_cast<int>(M.m01 / M.m00);
    } else {
        stop_count++;
    }

    // 結果フレームに重心を描画
    result_frame = morphed.clone();  // 描画用にフレームをコピー
    cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);

    // 結果をタプルで返す (重心のx座標, y座標, 描画済みフレーム)
    return std::make_tuple(cX, cY);
}

/*PID制御関数*/
static void PIDMotor(PID &pid) {
    // エラーベースのPID制御
    double error = frame_center - cX;
    double control = pid_control(pid, error);

    // モータ速度の初期化
    double left_motor_speed = left_speed;
    double right_motor_speed = right_speed;

    if (control > 0) {
        // 右に曲がる場合、左モータを減速し、右モータを加速
        left_motor_speed -= control;
        right_motor_speed += control;
    } else if (control < 0) {
        // 左に曲がる場合、右モータを減速し、左モータを加速
        left_motor_speed -= control;
        right_motor_speed += control;
    }

    // 停止条件が満たされた場合、モータを停止
    if (stop_count >= 30) {
        left_motor_speed = 0.0;
        right_motor_speed = 0.0;
}
    // モータ速度を表示
    std::cout << "Left Motor: " << left_motor_speed << ", Right Motor: " << right_motor_speed << std::endl;
    motor_cntrol(left_motor_speed, right_motor_speed);

}

static void PIDMotorR(PID &pid) {
    // エラーベースのPID制御
    double error = frame_center - cX;
    double control = pid_control(pid, error);

    // モータ速度の初期化
    double left_motor_speed = left_speed;
    double right_motor_speed = right_speed;

    if (control > 0) {
        // 右に曲がる場合、左モータを減速し、右モータを加速
        left_motor_speed += control;
        right_motor_speed -= control;
    } else if (control < 0) {
        // 左に曲がる場合、右モータを減速し、左モータを加速
        left_motor_speed += control;
        right_motor_speed -= control;
    }

    // 停止条件が満たされた場合、モータを停止
    if (stop_count >= 30) {
        left_motor_speed = 0.0;
        right_motor_speed = 0.0;
}
    // モータ速度を表示
    std::cout << "Left Motor: " << left_motor_speed << ", Right Motor: " << right_motor_speed << std::endl;
    motor_cntrol(left_motor_speed, right_motor_speed);

}


/* 走行モータ制御 */
static void motor_cntrol(double left_motor_speed , double right_motor_speed){
    // 実際のモータ制御関数をここで呼び出す
    left_motor_speed = left_motor_speed * left_motor_factor;
    right_motor_speed = right_motor_speed * right_motor_factor;
    while (true) {
        if (ev3_motor_set_power(left_motor, left_motor_speed) == 0) {
            break;
        }
        cv::waitKey(20);  // 20ms待機
    }
    while (true) {
        if (ev3_motor_set_power(right_motor, right_motor_speed) == 0) {
            break;
        }
        cv::waitKey(20);  // 20ms待機
    }
    return;
}

/* 青検知 */
static bool detectCheck(const Mat& morphed, int min_area) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 一定以上のサイズの輪郭を持つかどうかをチェック
    bool large_contour_detected = false;
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area >= min_area) {
            large_contour_detected = true;
            break;
        }
    }
    if (detection_count < 5){
        if (large_contour_detected){
            detection_count++;
        }else{
            detection_count = 0;
        }
    } else if (detection_count >= 5){
        if (!large_contour_detected){
            no_detection_count++;
        }else{
            no_detection_count = 0;
        }
    }
    if(no_detection_count >=5 ){
        detection_count = 0;
        no_detection_count = 0;
        return true;
    }
    return false;
}

/* 誤差計算 */
static double pid_control(PID &pid, double error) {
    pid.integral += error;
    double derivative = error - pid.previous_error;
    pid.previous_error = error;
    return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

/* 速度の設定 */
static void set_speed(double BASE_SPEED){
    left_speed = BASE_SPEED;
    right_speed = BASE_SPEED;
}

/* コンソール出力 */
void console_PL(){
    std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
    std::cout << "left " << ev3_motor_get_counts(left_motor) << " right " << ev3_motor_get_counts(right_motor) << std::endl;
    std::cout << "Case " << static_cast<int>(scene) << std::endl;
}

/* マスク値 */
std::map<std::string, std::pair<Scalar, Scalar>> color_bounds = {
    {"black", {Scalar(0, 0, 0), Scalar(180, 255, 50)}},  // 黒色
    {"blue", {Scalar(100, 100, 0), Scalar(140, 255, 255)}},  // 青色
    {"red_low", {Scalar(0, 100, 100), Scalar(10, 255, 255)}},  // 赤色（低範囲）
    {"red_high", {Scalar(160, 100, 100), Scalar(180, 255, 255)}},  // 赤色（高範囲）
    {"yellow", {Scalar(20, 100, 100), Scalar(30, 255, 255)}},  // 黄色
    {"green", {Scalar(40, 50, 50), Scalar(80, 255, 255)}},  // 緑色
    {"white", {Scalar(0, 0, 110), Scalar(180, 150, 255)}}
};
/* スタートタイマー */
static void startTimer(int timer_id) {
    if (timer_id == 1) {
        start_time1 = std::chrono::high_resolution_clock::now();
    } else if (timer_id == 2) {
        start_time2 = std::chrono::high_resolution_clock::now();
    } else if (timer_id == 3) {
        start_time3 = std::chrono::high_resolution_clock::now();
    }
}

// 経過時間を取得する関数 (秒単位, 小数点以下2桁)
static float getTime(int timer_id) {
    auto end_time = std::chrono::high_resolution_clock::now();
    
    if (timer_id == 1) {
        return std::chrono::duration_cast<std::chrono::duration<float>>(end_time - start_time1).count();
    } else if (timer_id == 2) {
        return std::chrono::duration_cast<std::chrono::duration<float>>(end_time - start_time2).count();
    } else if (timer_id == 3) {
        return std::chrono::duration_cast<std::chrono::duration<float>>(end_time - start_time3).count();
    } else {
        std::cerr << "Error: Invalid timer ID " << timer_id << std::endl;
        return 0.0f;
    }
}

// スレッドにCPUコアをセットする関数
void set_cpu_affinity(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);  // core_idで指定されたコアにスレッドを固定

    pthread_t current_thread = pthread_self();  // 現在のスレッドIDを取得
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

    if (result != 0) {
        std::cerr << "Error setting CPU affinity for core " << core_id << std::endl;
    } else {
        std::cout << "CPU affinity set to core " << core_id << std::endl;
    }
}

// 右モータのリセット関数
void reset_right_motor() {
    while (true) {
        ev3_motor_reset_counts(right_motor);
        if (ev3_motor_get_counts(right_motor) == 0) {
            break;
        }
        cv::waitKey(30);  // 30ms待機
    }
}

// 左モータのリセット関数
void reset_left_motor() {
    while (true) {
        ev3_motor_reset_counts(left_motor);
        if (ev3_motor_get_counts(left_motor) == 0) {
            break;
        }
        cv::waitKey(30);  // 30ms待機
    }
}

// ジャイロセンサーのリセット関数
void reset_gyro_sensor() {
    while (true) {
        ev3_gyro_sensor_reset(gyro_sensor);
        if (ev3_gyro_sensor_get_angle(gyro_sensor) == 0) {
            break;
        }
        cv::waitKey(30);  // 30ms待機
    }
}