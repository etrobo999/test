﻿/******************************************************************************
 *  app.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Task main_task
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "app.h"
#include "ScenarioList.h"
#include <iostream>

// using宣言
//using ev3api::ColorSensor;
using ev3api::TouchSensor;
//using ev3api::Motor;
//using ev3api::Clock;

// Device objects
// オブジェクトを静的に確保する
//ColorSensor gColorSensor(PORT_2);
TouchSensor gTouchSensor(PORT_1);
//Motor       gLeftWheel(PORT_C);
//Motor       gRightWheel(PORT_B);
//Clock       gClock;

// オブジェクトの定義
static Starter         *gStarter;
static ScenarioList    *gScenarioList;  // グローバル変数名に "g" を追加

/**
 * EV3システム生成
 */
static void user_system_create() {
    // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
    tslp_tsk(2U * 1000U);

    // オブジェクトの作成
    gStarter         = new Starter(gTouchSensor);
    gScenarioList     = new ScenarioList(gStarter);  // ScenarioListのインスタンスを作成

    // 初期化完了通知
    ev3_led_set_color(LED_ORANGE);
}

/**
 * EV3システム破棄
 */
static void user_system_destroy() {
//    gLeftWheel.reset();
//    gRightWheel.reset();

    delete gStarter;
    delete gScenarioList;  // "g" プレフィックスを付けた変数名を使用
}

/**
 * メインタスク
 */
void main_task(intptr_t unused) {
    user_system_create();  // センサやモータの初期化処理
    std::cout << "1" << std::endl;
    // 周期ハンドラ開始
    sta_cyc(CYC_TRACER);
    std::cout << "2" << std::endl;
    slp_tsk();  // バックボタンが押されるまで待つ
    std::cout << "3" << std::endl;
    // 周期ハンドラ停止
    stp_cyc(CYC_TRACER);
    std::cout << "4" << std::endl;
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