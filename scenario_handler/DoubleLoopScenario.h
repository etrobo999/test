#pragma once

#include "Scenario.h"

class DoubleLoopScenario : public Scenario {
public:
    DoubleLoopScenario();
    void execute() override;  // Execute double loop scenario
    void update() override;   // Update scene number
    void markCompletion();  // 完了フラグを設定する関数
    bool checkCompletion() const;  // シーンが完了したかを確認する
private:
    int sceneNo;  // シーン番号
    bool isComplete;  // シーンが完了したかどうかのフラグ
};