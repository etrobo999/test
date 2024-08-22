#include "DoubleLoopScenario.h"
#include "Drive.h"
#include <iostream>  // コンソール出力用
#include <thread>    // スリープ用
#include <chrono>    // スリープ用

// 外部のグローバル変数を利用する
extern Drive* gDrive;

DoubleLoopScenario::DoubleLoopScenario() : sceneNo(1), isComplete(false) {
}

void DoubleLoopScenario::execute() {
    switch (sceneNo) {
        case 1:
            std::cout << "Executing Scene 1: Moving straight" << std::endl;
            // シーン1の処理: 左右のベース速度を個別に設定して直進
            gDrive->setBaseSpeed(50.0, 50.0);
            gDrive->setMotorSpeeds(50.0, 50.0);
            std::this_thread::sleep_for(std::chrono::seconds(10));  // 10秒間スリープ
            update();  // シーン1が完了したら更新処理を呼び出す
            break;
        case 2:
            std::cout << "Executing Scene 2: Turning left" << std::endl;
            // シーン2の処理: 左に旋回
            gDrive->setBaseSpeed(40.0, 20.0);
            gDrive->setMotorSpeeds(40.0, 20.0);
            std::this_thread::sleep_for(std::chrono::seconds(10));  // 10秒間スリープ
            update();  // シーン2が完了したら更新処理を呼び出す
            break;
        case 3:
            std::cout << "Executing Scene 3: Scenario completed" << std::endl;
            markCompletion();
            break;
        default:
            std::cout << "Default case: No more scenes" << std::endl;
            break;
    }
}

void DoubleLoopScenario::update() {
        sceneNo += 1;  // 次のシーンへ移行
}

void DoubleLoopScenario::markCompletion() {
    isComplete = true;  // 完了フラグを設定
}

bool DoubleLoopScenario::checkCompletion() const {
    return isComplete;  // シーンの完了フラグを返す
}