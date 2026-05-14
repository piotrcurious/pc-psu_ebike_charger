#!/bin/bash
set -e

INC="-Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -Itests"

echo "Compiling tests..."
g++ $INC tests/test_bulk.cpp tests/mock_arduino/Arduino.cpp -o tests/run_bulk
g++ $INC tests/test_cv.cpp tests/mock_arduino/Arduino.cpp -o tests/run_cv

echo "Running Bulk Simulation..."
./tests/run_bulk | tail -n 20

echo -e "\nRunning CV Simulation..."
./tests/run_cv | tail -n 20

echo -e "\nTests completed."
