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
PID straightpid = {0.055, 0, 0.006, 0, 0}; //ストレートPID
PID Bcurvetpid = {0.11, 0.006, 0, 0, 0}; //急カーブPID
PID Mcurvetpid = {0.08, 0.003, 0, 0, 0}; //ちょうどいいカーブPID
PID Scurvetpid = {0.07, 0.004, 0, 0, 0}; //ゆっくりカーブPID

/*rectの値初期化*/
int rect_x = 0;
int rect_y = 0;
int rect_width = 640;
int rect_height = 340;

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

/*使用する（かもしれない）cv::MATの変数宣言*/
Mat orizin_frame, frame, rectframe, hsv, mask, mask1, mask2, morphed, morphed1, morphed2, result_frame;

/*使用する変数の初期化*/
uint8_t scene = 1;
int frame_center = 220;
int cX = 0;
int cY = 0;
double left_speed = 0.0;
double right_speed = 0.0;

//センサーの値を入れる変数
bool touch_sensor_bool = false;
int left_motor_counts = 0;
int right_motor_counts = 0;
int gyro_counts = 0;

// 追従方向の変数[true = 右] [false = 左]
bool follow = true;
bool resize_on = false;

// スレッドの操作のための変数
bool resetting = false;
bool frame_ready = false;
bool wb_ready = false;
bool display_ready = false;

// 連続して検知された回数をカウントする変数
int detection_count = 0;
int no_detection_count = 0;
int stop_count = 0;

//////////////////////////////////////////////////////////////////////
////////　　　     特殊なcamera処理　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

void* opencv_thread_func(void* arg) {
    set_cpu_affinity(0);
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
            startTimer(2);
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
                temp_frame.copyTo(orizin_frame);
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
            cout << "camera "  << getTime(2) <<endl;
        }
    }

    pthread_exit(NULL);
}

void* white_balance_thread_func(void* arg) {
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
            temp_frame1 = result_frame.clone();
        }

        // 表示処理
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
    set_cpu_affinity(1);
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    pthread_t opencv_thread;
    pthread_t white_balance_thread;
    pthread_t display_thread;

    // OpenCVスレッドを作成
    if (pthread_create(&opencv_thread, NULL, opencv_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create OpenCV thread" << endl;
        pthread_exit(NULL);
    }
    
    // ホワイトバランス処理スレッドを作成
    if (pthread_create(&white_balance_thread, NULL, white_balance_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create White Balance thread" << endl;
        pthread_exit(NULL);
    }

    // 画面表示スレッドを作成
    if (pthread_create(&display_thread, NULL, display_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create Display thread" << endl;
        pthread_exit(NULL);
    }

    bool ext = true;
    
    while (ext) {
        std::unique_lock<std::mutex> lock(mtx3);
        wb_var.wait(lock, [] { return wb_ready; });
        wb_ready = false;
        switch (scene) {
//////////////////////////////////////////////////////////////////////
////////　　　　　　スタート処理　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 1: //画面表示・ボタンでスタート
            ev3_motor_reset_counts(left_motor);
            ev3_motor_reset_counts(right_motor);
            ev3_gyro_sensor_reset(gyro_sensor);
            startTimer(1);
            ev3_gyro_sensor_reset(gyro_sensor);
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            cout <<getTime(1)<<endl;
            std::cout << ev3_gyro_sensor_get_angle(gyro_sensor) << std::endl;
            std::cout << ev3_touch_sensor_is_pressed(touch_sensor) << std::endl;
            std::cout << ev3_motor_get_counts(left_motor) << std::endl;
            std::cout << ev3_motor_get_counts(right_motor) << std::endl;
            std::cout << "Case 1" << std::endl;
            scene++;
            break;
        case 2:
            startTimer(1);
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "white");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_2(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            if(ev3_touch_sensor_is_pressed(touch_sensor)){
                scene = 11;
            };
            cout <<getTime(1)<<endl;
            std::cout << ev3_motor_get_counts(left_motor) << std::endl;
            std::cout << ev3_motor_get_counts(right_motor) << std::endl;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
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
            scene++;
            break;
        case 12: //第一ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 6350){
                scene++;
            }
            std::cout << "Case 12" << std::endl;
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
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Bcurvetpid);         
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 7500){
                scene++;
            }
            std::cout << "Case 14" << std::endl;
            break;
        case 15: //設定の読み込み
            startTimer(1);
            set_speed(55.0);
            scene++;
            break;
        case 16: //第二ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 11600){
                scene++;
            }
            std::cout << "Case 16" << std::endl;
            break;
        case 17://設定の読み込み
            follow = false;
            startTimer(1);
            set_speed(50.0);
            scene++;
            break;
        case 18: //第二急カーブ
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Bcurvetpid);
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            if(ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor) >= 12700){
                scene++;
            }
            std::cout << "Case 18" << std::endl;
            break;
        case 19://設定の読み込み
            follow = true;
            startTimer(1);
            set_speed(60.0);
            scene++;
            break;
        case 20: //第三ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            PIDMotor(straightpid);
            if(getTime(1) >=0.5){
                scene = 21;
            }
            std::cout << "Case 20" << std::endl;
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　第一難所　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 21://設定の読み込み
            set_speed(50.0);
            scene++;
            break;
        case 22://シーン1
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "blue_black"); //Mask,Mask1
            morphed = Morphology(mask);
            morphed1 = Morphology(mask1); //青色モル
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            std::cout << "Case 22" << std::endl;
            break;
        case 23://設定の読み込み
            set_speed(50.0);
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
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Mcurvetpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            std::cout << "Case 24" << std::endl;
            break;
        case 25://設定の読み込み
            set_speed(50.0);
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
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Scurvetpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            std::cout << "Case 26" << std::endl;
            break;
        case 27://設定の読み込み
            set_speed(50.0);
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
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Mcurvetpid);
            if(detectCheck(morphed1,2000)){
                scene++;
            }
            std::cout << "Case 28" << std::endl;
            break;
        case 29://設定の読み込み
            set_speed(50.0);
            follow = !follow;
            scene++;
            std::cout << follow << std::endl;
            break;
        case 30:
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY) = Follow_1(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);         
            if(getTime(1) >=2){
                scene++;
                ev3_motor_set_power(left_motor, 0);
                ev3_motor_set_power(right_motor, 0);
            }
            std::cout << "Case 30" << std::endl;
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　第三難所　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 31://設定の読み込み
            ev3_motor_reset_counts(left_motor);
            ev3_motor_reset_counts(right_motor);
            ev3_gyro_sensor_reset(gyro_sensor);
            rect_x = 0;
            rect_y = 0;  
            rect_width = 640;
            rect_height = 480;
            scene++;
            std::cout << "Case 31" << std::endl;
            break;
        case 32:
            std::cout <<ev3_gyro_sensor_get_angle(gyro_sensor)<< std::endl;
            std::cout <<ev3_motor_get_counts(left_motor)<< std::endl;
            std::cout <<ev3_motor_get_counts(right_motor)<< std::endl;
            std::cout << "Case 32" << std::endl;
            break;
        case 33:\
            tie(rectframe, hsv) = RectFrame(frame);
            createMask(hsv, "white"); //Mask,Mask1
            morphed = Morphology(mask1);//白色モル
            morphed1 = Morphology(mask2); //青色モル
            tie(cX, cY) = Follow_2(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Mcurvetpid);
            std::cout << "Case 33" << std::endl;
            break;
        case 34:
            std::cout << "Case 34" << std::endl;
            break;
        case 35:
            std::cout << "Case 35" << std::endl;
            break;
        case 36:
            std::cout << "Case 36" << std::endl;
            break;
        case 37:
            std::cout << "Case 37" << std::endl;
            break;
        case 38:
            std::cout << "Case 38" << std::endl;
            break;
        case 39:
            std::cout << "Case 39" << std::endl;
            break;
        case 40:
            std::cout << "Case 40" << std::endl;
            break;
        case 41:
            std::cout << "Case 41" << std::endl;
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
    Mat resizeframe, rectframe, hsv;
    resizeframe = frame.clone();

    
    rectframe = resizeframe(Rect(rect_x, rect_y, rect_width, rect_height));
    //rectframe = resizeframe(Rect(80, 200, 480, 140));
    cvtColor(rectframe, hsv, COLOR_BGR2HSV);
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
        inRange(hsv, color_bounds["white"].first, color_bounds["white"].second, mask1);
        inRange(hsv, color_bounds["blue"].first, color_bounds["blue"].second, mask2);
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


/*ライン切り替え追従関数*/
static std::tuple<int, int> Follow_1(const Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::cout << "Number of contours found: " << contours.size() << std::endl;

    for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    std::cout << "Contour " << i << " area: " << area << std::endl;
    }

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
        // 重心を描画
    }else{
        stop_count++;
    }
    result_frame = morphed.clone(); // 描画用にフレームをコピー
    cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
    // 結果をタプルで返す (重心のx座標, y座標, 描画済みフレーム)
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


/* 走行モータ制御 */
static void motor_cntrol(double left_motor_speed , double right_motor_speed){
    // 実際のモータ制御関数をここで呼び出す
    ev3_motor_set_power(left_motor, left_motor_speed);
    ev3_motor_set_power(right_motor, right_motor_speed);
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


/* マスク値 */
std::map<std::string, std::pair<Scalar, Scalar>> color_bounds = {
    {"black", {Scalar(0, 0, 0), Scalar(180, 255, 30)}},  // 黒色
    {"blue", {Scalar(100, 100, 0), Scalar(140, 255, 255)}},  // 青色
    {"red_low", {Scalar(0, 100, 100), Scalar(10, 255, 255)}},  // 赤色（低範囲）
    {"red_high", {Scalar(160, 100, 100), Scalar(180, 255, 255)}},  // 赤色（高範囲）
    {"yellow", {Scalar(20, 100, 100), Scalar(30, 255, 255)}},  // 黄色
    {"green", {Scalar(40, 50, 50), Scalar(80, 255, 255)}},  // 緑色
    {"white", {Scalar(0, 0, 180), Scalar(180, 255, 255)}}  // 白色
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