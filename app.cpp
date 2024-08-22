#include "app.h"
#include "ScenarioList.h"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
using ev3api::ColorSensor;
using ev3api::TouchSensor;
using ev3api::Motor;
using ev3api::Clock;

// Device objects
// オブジェクトを静的に確保する
ColorSensor gColorSensor(PORT_2);
TouchSensor gTouchSensor(PORT_1);
Motor       gLeftWheel(PORT_C);
Motor       gRightWheel(PORT_B);
Clock       gClock;

// オブジェクトの定義
static Starter            *gStarter;
static ScenarioList       *gScenarioList;  // グローバル変数名に "g" を追加
static Drive              *gDrive;

/**
 * EV3システム生成
 */
static void user_system_create() {
    // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
    tslp_tsk(2U * 1000U);

    // オブジェクトの作成
    gStarter          = new Starter(gTouchSensor);
    gDrive            = new Drive(gLeftWheel, gRightWheel);
    gScenarioList     = new ScenarioList();  // ScenarioListのインスタンスを作成

    // 初期化完了通知
    ev3_led_set_color(LED_ORANGE);
}

/**
 * EV3システム破棄
 */
static void user_system_destroy() {
    gLeftWheel.reset();
    gRightWheel.reset();
    delete gStarter;
    delete gDrive;
    delete gScenarioList;  // "g" プレフィックスを付けた変数名を使用
}

/**
 * メインタスク
 */
void main_task(intptr_t unused) {
    user_system_create();  // センサやモータの初期化処理

    // 周期ハンドラ開始
    sta_cyc(CYC_TRACER);

    slp_tsk();  // バックボタンが押されるまで待つ

    // 周期ハンドラ停止
    stp_cyc(CYC_TRACER);

    user_system_destroy();  // 終了処理

    ext_tsk();
}

/**
 * ライントレースタスク
 */
void tracer_task(intptr_t exinf) {
    if (gScenarioList->isCompleted()) {
        wup_tsk(MAIN_TASK); 
    } else {
        gScenarioList->executeScenario();  // 修正されたクラス名でメソッドを呼び出す
    }

    ext_tsk();
}