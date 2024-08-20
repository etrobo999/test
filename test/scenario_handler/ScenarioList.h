#pragma once

#include <memory>

class ScenarioList {
public:
    ScenarioList();
    void executeScenario();  // Execute the current scenario
    void incrementScenarioNo();  // Increment the current scenario number (public function)
    void ScenariomarkCompletion();  // 完了フラグを設定する関数
    bool ScenarioCompletion() const;
private:
    int ScenarioNo ;  // Current scenario number
    std::unique_ptr<Scenario> scenarioBox;  // Current scenario instance (scenarioBox)
    bool ScenarioComplete;  // シーンが完了したかどうかのフラグ
};