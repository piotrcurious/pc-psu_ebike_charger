#!/bin/bash
set -e

echo "Compiling and running Iteration Test..."
g++ -o tests/test_iteration tests/test_iteration.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_iteration | grep "TEST PASSED"

echo "Compiling and running Safety Test..."
g++ -o tests/test_safety tests/test_safety.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_safety | grep "PASSED" | wc -l | grep -q "4" && echo "Safety Test Passed"

echo "Compiling and running Robustness Test..."
g++ -o tests/test_robustness tests/test_robustness.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_robustness | grep "PASSED" | wc -l | grep -q "2" && echo "Robustness Test Passed"

echo "Compiling and running Unit Test..."
g++ -o tests/test_unit tests/test_unit.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_unit | grep "PASSED" | wc -l | grep -q "15" && echo "Unit Test Passed"

echo "Compiling and running Advanced Test..."
g++ -o tests/test_advanced tests/test_advanced.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_advanced | grep "PASSED" | wc -l | grep -q "4" && echo "Advanced Test Passed"

echo "Compiling and running TFT Test..."
g++ -o tests/test_tft tests/test_tft.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Wire -I. -DUNIT_TEST -DESP32_WROOM_TFT
./tests/test_tft | grep "PASSED" | wc -l | grep -q "1" && echo "TFT Test Passed"

echo "Compiling and running Performance Test..."
g++ -o tests/test_performance tests/test_performance.cpp tests/mock_arduino/Arduino.cpp -Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -I. -DUNIT_TEST
./tests/test_performance | grep "PASSED" | wc -l | grep -q "2" && echo "Performance Test Passed"

echo "ALL TESTS PASSED"
