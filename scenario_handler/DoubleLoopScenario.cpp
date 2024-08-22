#include "DoubleLoopScenario.h"
#include <iostream>  // コンソール出力用

DoubleLoopScenario::DoubleLoopScenario() 
    : Scenario2(){
}
void DoubleLoopScenario::execute() {
    switch (sceneNo) {
        case 1:
            std::cout << "Executing Scene 1: Moving straight" << std::endl;
            // シーン1の処理: 左右のベース速度を個別に設定して直進
            update();  // シーン1が完了したら更新処理を呼び出す
            break;
        case 2:
            std::cout << "Executing Scene 2: Turning left" << std::endl;
            // シーン2の処理: 左に旋回
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
