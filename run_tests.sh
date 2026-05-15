#!/bin/bash
set -e
INC="-Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -Itests"
echo "Compiling tests..."
g++ $INC tests/test_bulk.cpp tests/mock_arduino/Arduino.cpp -o tests/run_bulk
g++ $INC tests/test_cv.cpp tests/mock_arduino/Arduino.cpp -o tests/run_cv
echo "Running Bulk Simulation..."
./tests/run_bulk | grep "Complete"
echo "Running CV Simulation..."
./tests/run_cv | grep "Complete"
rm tests/run_bulk tests/run_cv
echo -e "\nTests completed successfully."
