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
./tests/test_robustness | grep "PASSED" | wc -l | grep -q "1" && echo "Robustness Test Passed"

echo "ALL TESTS PASSED"
