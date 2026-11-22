// 第三／第四關機構測試：ID4/5/6 步進與 Pin2/3/5 伺服。
#include <Arduino.h>
#include "test_controller.h"

void setup() { Stage4Test::begin(); }
void loop() { Stage4Test::loop(); }
