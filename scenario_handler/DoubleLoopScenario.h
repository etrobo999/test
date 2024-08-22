#pragma once
#include "Scenario2.h"
#include "Drive.h"

class DoubleLoopScenario : public Scenario2 {
public:
    DoubleLoopScenario();
    void execute() override;  // Execute double loop scenario
};