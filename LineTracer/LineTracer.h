#pragma once
#include <opencv2/opencv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

/* 下記の項目は各ロボットに合わせて変えること */

/* カラーセンサの輝度設定 */
constexpr int WHITE_BRIGHTNESS = 180;
constexpr int BLACK_BRIGHTNESS = 10;

/*カメラの閾値設定*/
constexpr int THRESHOLDVALUE = 25;
constexpr int MAXBINARYVALUE = 255;

/*カメラのトリミング*/
constexpr int TRIMY = 270;
constexpr int TRIMH = 40;

constexpr int frame_center = 180; 

constexpr int ALLB_Y1 = 0;
constexpr int ALLB_Y2 = TRIMH;

/* ステアリング操舵量の係数 */
constexpr float STEERING_COEF = 0.2F;

#ifdef __cplusplus
}
#endif
