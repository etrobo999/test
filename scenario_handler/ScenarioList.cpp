#include "ScenarioList.h"
#include "DoubleLoopScenario.h"

extern Starter* gStarter;

ScenarioList::ScenarioList() : scenarioNo(0), scenarioComplete(false) {
}

void ScenarioList::executeScenario() {
    switch (scenarioNo) {
        case 0:
            if (gStarter->isPushed()) {
                incrementScenarioNo();  // 次のシナリオへ移行
            }
            break;
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
            // シナリオの処理が完了した後の処理を追加するか、空であることを明示
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