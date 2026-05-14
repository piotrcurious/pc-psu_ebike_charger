# pc-psu_ebike_charger
Step-up e-bike battery charger using a PC ATX PSU and ESP32-C3.

## Features
- **Robust Control:** Step-based CC/CV charging strategy, less sensitive to hardware tolerances.
- **ATX PSU Integration:** Controls the PSU via the PS_ON line with a 10-second anti-cycle cooldown.
- **Safety:**
  - Battery presence detection (minimum voltage threshold).
  - Maximum charge timeout (12 hours).
  - Overcurrent emergency shutdown.
  - Periodic unloaded voltage checks.
- **Accuracy:** Startup zero-offset calibration for ACS712 current sensor and digital filtering.
- **UI:** OLED display (SSD1306) showing real-time status.

## Testing
The project includes a C++ mock Arduino environment and physics-based simulation.
To run the tests:
\`\`\`bash
./run_tests.sh
\`\`\`
