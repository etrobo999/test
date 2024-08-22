#include "ScenarioList.h"
#include <iostream>
ScenarioList::ScenarioList(const Starter* starter) : scenarioNo(0), mStarter(starter), scenarioComplete(false) {
}

void ScenarioList::executeScenario() {
    switch (scenarioNo) {
        case 0:
            std::cout << "1-1" << std::endl;
            if (mStarter->isPushed()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
        case 1:
            std::cout << "1-2" << std::endl;
            doubleLoopScenario.execute(); 
            if (doubleLoopScenario.checkCompletion()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
        case 2:
            std::cout << "1-2" << std::endl;
            markCompletion();
            break;
        default:
            break;
    }
}

void ScenarioList::incrementScenarioNo() {
    scenarioNo++;  // シナリオ番号を1つ進める
}

void ScenarioList::markCompletion() {
    scenarioComplete = true;  // 完了フラグを設定
}

bool ScenarioList::isCompleted() const {
    return scenarioComplete;
}