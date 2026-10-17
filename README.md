# IB3 Autonomous Bicycle — Hardware API & Protocol Documentation

[![Course](https://img.shields.io/badge/KU_Leuven-IB3-lightgrey)](https://www.kuleuven.be/)
[![Deploy Documentation](https://github.com/ZennDC/ib3-api/actions/workflows/deploy.yml/badge.svg?branch=master)](https://github.com/ZennDC/ib3-api/actions/workflows/deploy.yml)
[![Pages Status](https://img.shields.io/badge/GitHub%20Pages-Active-brightgreen?logo=github)](https://ZennDC.github.io/ib3-api/)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17%2F20-blue?logo=c%2B%2B)](include/)
[![Docs](https://img.shields.io/badge/docs-MkDocs%20%7C%20Doxygen-blue?logo=materialformkdocs)](https://ZennDC.github.io/ib3-api/)

- **API Reference & Documentation**: [https://ZennDC.github.io/ib3-api/](https://ZennDC.github.io/ib3-api/)
- **Project Report & Requirements**: [download the PDF from the site](https://ZennDC.github.io/ib3-api/assets/Report%20Vergadering%20IB3%20project.pdf) or [view it in the repository](docs/assets/Report%20Vergadering%20IB3%20project.pdf)

## File and Project Structure

```text
.
├── include/
│   ├── interface.hpp       # Abstract C++ HAL interface (BikeHardware::Interface)
│   ├── types.hpp           # High-level data models (BikeTelemetry, MotionCommand, enums, SafetyFlag)
│   └── protocol.hpp        # DRAFT - Binary SPI protocol definitions, headers, opcodes, and wire structs
├── docs/
│   ├── index.md            # Main MkDocs documentation page
│   ├── requirements.txt    # Python dependencies for MkDocs & MkDoxy
│   └── assets/
│       └── Report Vergadering IB3 project.pdf  # Report containing the requirements
├── .github/
│   └── workflows/
│       └── deploy.yml      # CI/CD pipeline for automatic GitHub Pages publication
├── mkdocs.yml              # MkDocs configuration
└── README.md               # This document
```

---

## Example Code

### ICT Usage Example

```cpp
#include "interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

void control_loop(BikeHardware::Interface& bike) {
    // 1. Initialize the hardware connection
    if (!bike.init()) {
        std::cerr << "Error initializing bicycle hardware!\n";
        return;
    }

    // 2. Register a callback for telemetry updates
    bike.register_telemetry_callback([](const BikeHardware::BikeTelemetry& telem) {
        if (telem.safety_flags & BikeHardware::SafetyFlag::FALL_DETECTED) {
            std::cerr << "ALARM: Bicycle has tipped over / fall detected!\n";
        }
    });

    // 3. Set lights to automatic mode
    bike.set_lights(BikeHardware::LightMode::AUTO);

    // 4. Send a motion command: 2.5 m/s, steering angle 3 degrees
    BikeHardware::MotionCommand cmd;
    cmd.target_speed_mps = 2.5f;
    cmd.target_acceleration_mps2 = 0.5f;
    cmd.target_steering_deg = 3.0f;
    cmd.duration_rotations = 8.0f;
    bike.set_motion(cmd);

    // 5. Regular control loop with heartbeat
    while (bike.get_status() == BikeHardware::SystemStatus::ACTIVE_AUTONOMOUS) {
        bike.send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    bike.shutdown();
}
```

### Electronics HAL Skeleton

```cpp
#include "interface.hpp"
#include "protocol.hpp"

namespace BikeHardware {

class BikeHardwareSPI : public Interface {
public:
    bool init() override {
        // Initialize the SPI bus to the ESP32 and perform the handshake
        return true;
    }

    void shutdown() override {
        // Safely disable actuators and close the communication bus
    }

    bool send_heartbeat() override {
        // Send a Protocol::Opcode::HEARTBEAT frame over SPI
        return true;
    }

    void emergency_stop(const std::string& reason) override {
        // Send WIRE_OP_EMERGENCY_STOP and engage the brakes
    }

    bool reset_emergency_stop() override {
        // Send WIRE_OP_RESET_EMERGENCY if it is safe to do so
        return true;
    }

    bool set_motion(const MotionCommand& cmd) override {
        // Convert float values to WireMotionCmd integer representations and send them over SPI
        return true;
    }

    bool set_brake(const BrakeCommand& cmd) override {
        // Convert cmd.front_force_pct, cmd.rear_force_pct, and enable_regenerative to WireBrakeCmd
        return true;
    }

    // ... implement remaining virtual methods ...
};

} // namespace BikeHardware
```

## Preview Documentation Locally

Make sure `doxygen` and Python are installed on your system:

```bash
# Dependencies (Ubuntu)
sudo apt update && sudo apt install -y doxygen graphviz

# Python virtual environment
python3 -m venv .venv
source .venv/bin/activate
pip install -r docs/requirements.txt

# Start the local server
mkdocs serve
```
