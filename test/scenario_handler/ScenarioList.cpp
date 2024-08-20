#include "ScenarioList.h"
#include "DoubleLoopScenario.h"

extern Starte* gStarte;
ScenarioList::ScenarioList() : ScenarioNo(0), ScenarioComplete(false) {
}

void ScenarioList::executeScenario() {
    switch (ScenarioNo) {
        case 0:
             if (gStarte->isPushed()) {
                    incrementScenarioNo();  // 次のシナリオへ移行
            }
        case 1:
            scenarioBox = std::make_unique<DoubleLoopScenario>();  // ダブルループシナリオクラスを生成
            incrementScenarioNo();
            break;
        case 2:
            scenarioBox->execute();  // ダブルループシナリオを実行
            if (scenarioBox->checkCompletion()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
        case 3:

        default:
            break;
    }
}

void ScenarioList::incrementScenarioNo() {
    ScenarioNo++;  // シナリオ番号を1つ進める
}

void ScenarioList::ScenariomarkCompletion() {
    ScenarioComplete = true;  // 完了フラグを設定
}

bool ScenarioList::ScenarioCompletion() const {
    return ScenarioComplete;
}