#!/bin/bash
set -e
INC="-Itests/mock_arduino -Itests/mock_arduino/Adafruit_SSD1306 -Itests/mock_arduino/Adafruit_GFX -Itests/mock_arduino/Wire -Itests"
echo "Compiling tests..."
g++ $INC tests/test_bulk.cpp tests/mock_arduino/Arduino.cpp -o tests/run_bulk
g++ $INC tests/test_cv.cpp tests/mock_arduino/Arduino.cpp -o tests/run_cv
g++ $INC tests/test_noisy.cpp tests/mock_arduino/Arduino.cpp -o tests/run_noisy
g++ $INC tests/test_ui.cpp tests/mock_arduino/Arduino.cpp -o tests/run_ui
echo "Running Bulk Simulation..."
./tests/run_bulk | tail -n 20
echo "Running CV Simulation..."
./tests/run_cv | tail -n 20
echo "Running Noisy Simulation..."
./tests/run_noisy | tail -n 20
echo "Running UI Test..."
./tests/run_ui
rm tests/run_bulk tests/run_cv tests/run_noisy tests/run_ui
echo -e "\nTests completed successfully."
