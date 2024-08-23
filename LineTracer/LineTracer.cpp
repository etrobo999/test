#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
#include "app.h"
#include "LineTracer.h"
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <signal.h>
#include <pthread.h>
#include <mutex>
#include <condition_variable>

using namespace std;
using namespace cv;

raspicam::RaspiCam_Cv Camera;

/*PIDインスタンス生成*/
PID straightpid = {0.1, 0, 0, 0, 0}; //ストレートPID
PID Bcurvetpid = {0.18, 0, 0, 0, 0}; //急カーブPID
PID Mcurvetpid = {0.14, 0, 0, 0, 0}; //ちょうどいいカーブPID
PID Scurvetpid = {0.12, 0, 0, 0, 0}; //ゆっくりカーブPID


/*使用する変数の宣言*/
std::chrono::high_resolution_clock::time_point start_time1;
std::chrono::high_resolution_clock::time_point start_time2;
std::chrono::high_resolution_clock::time_point start_time3;

std::mutex mtx;
std::condition_variable condition_var;
Mat frame, rectframe, hsv, mask, morphed, result_frame;

/*使用する変数の初期化*/
uint8_t scene = 1;
int cX = 0;
int cY = 0;
double BASE_SPEED = 0.0;
bool follow = true;
bool frame_ready = false;


/*なんかすごいcameraの処理*/
void* opencv_thread_func(void* arg) {
    // シグナルマスクの設定
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR2);  // ASPカーネルが使用するシグナルをマスク
    sigaddset(&set, SIGPOLL);  // その他のカーネルシグナルをマスク
    sigaddset(&set, SIGALRM);  // タイマーシグナルをマスク
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    // カメラ初期化 (一度だけ行う)
    Camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    Camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    Camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(cv::CAP_PROP_FPS, 24);
    Camera.set(cv::CAP_PROP_AUTO_WB, 1);
    if (!Camera.open()) {
        cerr << "Error: !Camera.open" << endl;
        pthread_exit(NULL);
    }

    while (true) {
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
        condition_var.notify_one();
    }

    pthread_exit(NULL);
}

/*走行時の根本プログラム*/
void tracer_task(intptr_t unused) {
    pthread_t opencv_thread;

    // OpenCVスレッドを作成
    if (pthread_create(&opencv_thread, NULL, opencv_thread_func, NULL) != 0) {
        cerr << "Error: Failed to create OpenCV thread" << endl;
        return;
    }
    bool ext = true;
    
    while (ext){
        std::unique_lock<std::mutex> lock(mtx);
        condition_var.wait(lock, [] { return frame_ready; });
        switch (scene) {

//////////////////////////////////////////////////////////////////////
////////　　　　　　スタート処理　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 1:
            startTimer(2);
            scene++;
            break;
        case 2: //画面表示・ボタンでスタート
            tie(rectframe, hsv) = RectFrame(frame);
            mask = createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY, result_frame) = ProcessContours(morphed);
            cout << "Centroid: (" << cX << ", " << cY << ")" <<endl;
            if(ev3_touch_sensor_is_pressed(touch_sensor)){
                scene = 3;
            };
            cout << getTime(2) <<endl;
            startTimer(2);
            break;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
            cout << getTime(2) <<endl;
            startTimer(2);
            scene = 11;
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　難所前ライントレース　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 11: //設定の読み込み
            startTimer(1);
            follow = true;
            BASE_SPEED = 95.0;
            scene++;
            break;
        case 12: //第一ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            mask = createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY, result_frame) = ProcessContours(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);         
            if(getTime(1) >=10){
                scene++;
            }
            std::cout << "Case 12" << std::endl;
            cout << getTime(2) <<endl;
            startTimer(2);
            break;
        case 13: //設定の読み込み
            startTimer(1);
            BASE_SPEED = 70.0;
            scene++;
            break;
        case 14: //第一急カーブ
            tie(rectframe, hsv) = RectFrame(frame);
            mask = createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY, result_frame) = ProcessContours(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Bcurvetpid);         
            if(getTime(1) >=4){
                scene++;
            }
            cout << getTime(2) <<endl;
            startTimer(2);
            std::cout << "Case 14" << std::endl;
            break;
        case 15: //設定の読み込み
            startTimer(1);
            BASE_SPEED = 95.0;
            scene++;
            break;
        case 16: //第二ストレート
            tie(rectframe, hsv) = RectFrame(frame);
            mask = createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY, result_frame) = ProcessContours(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(straightpid);
            if(getTime(1) >=6){
                scene++;
            }
            cout << getTime(2) <<endl;
            startTimer(2);
            std::cout << "Case 16" << std::endl;
            break;
        case 17://設定の読み込み
            startTimer(1);
            BASE_SPEED = 70.0;
            scene++;
            break;
        case 18: //第二急カーブ
            tie(rectframe, hsv) = RectFrame(frame);
            mask = createMask(hsv, "black");
            morphed = Morphology(mask);
            tie(cX, cY, result_frame) = ProcessContours(morphed);
            std::cout << "Centroid: (" << cX << ", " << cY << ")" << std::endl;
            PIDMotor(Bcurvetpid);
            if(getTime(1) >=4){
                scene++;
            }
            cout << getTime(2) <<endl;
            startTimer(2);
            std::cout << "Case 18" << std::endl;
            break;
        case 19:
        case 20:
            scene = 21;
            std::cout << "Case 20" << std::endl;
            break;

//////////////////////////////////////////////////////////////////////
////////　　　　　　　　第一難所　　　　　　　　　　/////////////////////
//////////////////////////////////////////////////////////////////////

        case 21:
            std::cout << "Case 21" << std::endl;
            break;
        case 22:
            std::cout << "Case 22" << std::endl;
            break;
        case 23:
            std::cout << "Case 23" << std::endl;
            break;
        case 24:
            std::cout << "Case 24" << std::endl;
            break;
        case 25:
            std::cout << "Case 25" << std::endl;
            break;
        case 26:
            std::cout << "Case 26" << std::endl;
            break;
        case 27:
            std::cout << "Case 27" << std::endl;
            break;
        case 28:
            std::cout << "Case 28" << std::endl;
            break;
        case 29:
            std::cout << "Case 29" << std::endl;
            break;
        case 30:
            std::cout << "Case 30" << std::endl;
            break;
        case 31:
            std::cout << "Case 31" << std::endl;
            break;
        case 32:
            std::cout << "Case 32" << std::endl;
            break;
        case 33:
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
        frame_ready = false;
    }
    /* タスク終了 */
    ext_tsk(); // タスクを終了
}

/* フレームの取得処理 
static Mat Capture(void){
    Mat frame;
    Camera.grab();
    Camera.retrieve(frame);
    if (frame.empty()){
        cerr << "frame.empty" << endl;
        return Mat(); // 空の Mat オブジェクトを返す
    }
    return frame; // 獲得したフレームを返す
}                                                   */


/* フレームのトリミング＆HSV変換 */
static tuple<Mat, Mat>  RectFrame(const Mat& frame) {
    Mat rectframe, hsv;
    rectframe = frame(Rect(140, 240, 360, 60));
    cvtColor(rectframe, hsv, COLOR_BGR2HSV);
    return make_tuple(rectframe, hsv);
}

/* マスク変換 */
static Mat createMask(const Mat& hsv, const std::string& color) {
    Mat mask;

    if (color == "red") {
        Mat mask1, mask2;
        inRange(hsv, color_bounds["red_low"].first, color_bounds["red_low"].second, mask1);
        inRange(hsv, color_bounds["red_high"].first, color_bounds["red_high"].second, mask2);
        mask = mask1 | mask2;  // 両方のマスクを統合
    } else if (color == "blue_black") {
        Mat mask1, mask2;
        inRange(hsv, color_bounds["blue"].first, color_bounds["blue"].second, mask1);
        inRange(hsv, color_bounds["black"].first, color_bounds["black"].second, mask2);
        mask = mask1 |         mask = mask1 | mask2;  // 両方のマスクを統合
;  // 青と黒のマスクを統合
    } else {
        inRange(hsv, color_bounds[color].first, color_bounds[color].second, mask);
    }

    return mask;
}


/* モルフォロジー変換 */
static Mat Morphology(const Mat& mask) {
    Mat morphed;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, morphed, MORPH_OPEN, kernel);
    morphologyEx(morphed, morphed, MORPH_CLOSE, kernel);    
    return morphed;  // モルフォロジー変換後の画像を返す
}


/*追従関数*/
static std::tuple<int, int, Mat> ProcessContours(const Mat& morphed) {
    // 輪郭を抽出
    std::vector<std::vector<cv::Point>> contours;
    findContours(morphed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::cout << "Number of contours found: " << contours.size() << std::endl;

    for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    std::cout << "Contour " << i << " area: " << area << std::endl;
    }

    const double min_contour_area = 500.0; // ピクセル数

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
        result_frame = rectframe.clone(); // 描画用にフレームをコピー
        cv::circle(result_frame, cv::Point(cX, cY), 5, cv::Scalar(255, 0, 0), -1);
        cv::imshow("result_frame", result_frame);
        cv::waitKey(1);
    }

    // 結果をタプルで返す (重心のx座標, y座標, 描画済みフレーム)
    return std::make_tuple(cX, cY, result_frame);
}


/*PID制御関数*/
static void PIDMotor(PID &pid) {
    // エラーベースのPID制御
    double error = frame_center - cX;
    double straight_control = pid_control(pid, error);

    // モータ速度の初期化
    double left_motor_speed = BASE_SPEED;
    double right_motor_speed = BASE_SPEED;

    // フィードバック制御のためのモータ制御
    if (straight_control > 0) {
        left_motor_speed -= straight_control * 2;
    } else if (straight_control < 0) {
        right_motor_speed += straight_control * 2;
    } else {
        left_motor_speed -= straight_control;
        right_motor_speed += straight_control;
    }

    // モータ速度を表示
    std::cout << "Left Motor: " << left_motor_speed << ", Right Motor: " << right_motor_speed << std::endl;
    
    // 実際のモータ制御関数を呼び出す
    motor_cntrol(left_motor_speed, right_motor_speed);
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


/* 画像の表示 */
static void Show(const Mat& showfreme){
    cv::imshow("showfreme", showfreme);
    cv::waitKey(1);
    return;
}

/* 誤差計算 */
static double pid_control(PID &pid, double error) {
    pid.integral += error;
    double derivative = error - pid.previous_error;
    pid.previous_error = error;
    return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}


std::map<std::string, std::pair<Scalar, Scalar>> color_bounds = {
    {"black", {Scalar(0, 0, 0), Scalar(180, 255, 50)}},  // 黒色
    {"blue", {Scalar(100, 150, 0), Scalar(140, 255, 255)}},  // 青色
    {"red_low", {Scalar(0, 100, 100), Scalar(10, 255, 255)}},  // 赤色（低範囲）
    {"red_high", {Scalar(160, 100, 100), Scalar(180, 255, 255)}},  // 赤色（高範囲）
    {"yellow", {Scalar(20, 100, 100), Scalar(30, 255, 255)}},  // 黄色
    {"green", {Scalar(40, 50, 50), Scalar(80, 255, 255)}}  // 緑色
};

// 時間のカウント開始
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