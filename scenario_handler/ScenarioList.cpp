#include "ScenarioList.h"
ScenarioList::ScenarioList(const Starter* starter) : scenarioNo(0), mStarter(starter), scenarioComplete(false) {
}

void ScenarioList::executeScenario() {
    switch (scenarioNo) {
        case 0:
            if (mStarter->isPushed()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
        case 1:
            doubleLoopScenario.execute(); 
            if (doubleLoopScenario.checkCompletion()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
        case 2:
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