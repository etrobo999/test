#pragma once

class Scenario {
public:
    virtual void execute() = 0;  // Execute scenario based on the current scene number
    virtual void update() = 0;   // Update the scene number
    virtual void markCompletion();  // 完了フラグを設定する関数
    bool checkCompletion() const;  // シーンが完了したかを確認する
    virtual ~Scenario() = default;

protected:
    int sceneNo;  // シーン番号
    bool isComplete;  // シーンが完了したかどうかのフラグ
};