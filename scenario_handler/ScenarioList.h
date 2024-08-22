#pragma once

#include <memory>

// 前方宣言
class Scenario;

class ScenarioList {
public:
    ScenarioList();
    void executeScenario();  // Execute the current scenario
    void incrementScenarioNo();  // Increment the current scenario number
    void markCompletion();  // 完了フラグを設定する関数
    bool isCompleted() const;  // シーンが完了したかどうかを確認する

private:
    unsigned int scenarioNo;  // Current scenario number
    std::unique_ptr<Scenario> scenarioBox;  // Current scenario instance
    bool scenarioComplete;  // シーンが完了したかどうかのフラグ
};