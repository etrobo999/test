#pragma once

class Scenario {
public:
    Scenario() : sceneNo(0), isComplete(false) {}  // コンストラクタで初期化
    virtual void execute() = 0;  // Execute scenario based on the current scene number
    virtual void update() = 0;   // Update the scene number
    virtual void markCompletion() { isComplete = true; }  // 完了フラグを設定する関数
    virtual bool checkCompletion() const { return isComplete; }  // シーンが完了したかを確認する
    virtual ~Scenario() = default;

protected:
    unsigned int sceneNo;  // シーン番号
    bool isComplete;  // シーンが完了したかどうかのフラグ
};
