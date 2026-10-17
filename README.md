# IB3 Autonomous Bicycle — Hardware API & Protocol Documentation

[![Course](https://img.shields.io/badge/KU_Leuven-IB3-lightgrey)](https://www.kuleuven.be/)
[![Deploy Documentation](https://github.com/ZennDC/ib3-api/actions/workflows/deploy.yml/badge.svg?branch=master)](https://github.com/ZennDC/ib3-api/actions/workflows/deploy.yml)
[![Pages Status](https://img.shields.io/badge/GitHub%20Pages-Active-brightgreen?logo=github)](https://ZennDC.github.io/ib3-api/)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17%2F20-blue?logo=c%2B%2B)](include/)
[![Docs](https://img.shields.io/badge/docs-MkDocs%20%7C%20Doxygen-blue?logo=materialformkdocs)](https://ZennDC.github.io/ib3-api/)

- **API Referentie & Documentatie**: [https://ZennDC.github.io/ib3-api/](https://ZennDC.github.io/ib3-api/)
- **Projectrapport & Requirements**: [PDF downloaden via site](https://ZennDC.github.io/ib3-api/assets/Report%20Vergadering%20IB3%20project.pdf) of [bekijken in repo](docs/assets/Report%20Vergadering%20IB3%20project.pdf)

## Bestands- en Projectstructuur

```text
.
├── include/
│   ├── interface.hpp       # Abstracte C++ HAL interface (BikeHardware::Interface)
│   ├── types.hpp           # High-level datamodellen (BikeTelemetry, MotionCommand, enums, SafetyFlag)
│   └── protocol.h          # DRAFT - Binaire SPI-protocoldefinities, headers, opcodes en wire-structs
├── docs/
│   ├── index.md            # Hoofdpagina van de MkDocs-documentatie
│   ├── requirements.txt    # Python dependencies voor MkDocs & MkDoxy
│   └── assets/
│       └── Report Vergadering IB3 project.pdf  # Verslag met requirements
├── .github/
│   └── workflows/
│       └── deploy.yml      # CI/CD pipeline voor automatische publicatie naar GitHub Pages
├── mkdocs.yml              # MkDocs configuratie
└── README.md               # Dit document
```

---

## Voorbeeldcode

### ICT Gebruiksvoorbeeld

```cpp
#include "interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

void control_loop(BikeHardware::Interface& bike) {
    // 1. Initialiseer de hardwareverbinding
    if (!bike.init()) {
        std::cerr << "Fout bij initialisatie van fietshardware!\n";
        return;
    }

    // 2. Registreer callback voor telemetrie-updates
    bike.register_telemetry_callback([](const BikeHardware::BikeTelemetry& telem) {
        if (telem.safety_flags & BikeHardware::SafetyFlag::FALL_DETECTED) {
            std::cerr << "ALARM: Fiets is gekanteld / val gedetecteerd!\n";
        }
    });

    // 3. Verlichting instellen op automatische modus
    bike.set_lights(BikeHardware::LightMode::AUTO);

    // 4. Stuur een bewegingscommando: 2.5 m/s, stuurhoek 3 graden
    BikeHardware::MotionCommand cmd;
    cmd.target_speed_mps = 2.5f;
    cmd.target_acceleration_mps2 = 0.5f;
    cmd.target_steering_deg = 3.0f;
    cmd.duration_rotations = 8.0f;
    bike.set_motion(cmd);

    // 5. Regelmatige besturingsloop met heartbeat
    while (bike.get_status() == BikeHardware::SystemStatus::ACTIVE_AUTONOMOUS) {
        bike.send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    bike.shutdown();
}
```

### Elektronica HAL Skeleton

```cpp
#include "interface.hpp"
#include "protocol.h"

namespace BikeHardware {

class BikeHardwareSPI : public Interface {
public:
    bool init() override {
        // Initialiseer SPI-bus naar ESP32 en voer handshake uit
        return true;
    }

    void shutdown() override {
        // Schakel actuatoren veilig uit en sluit communicatiebus
    }

    bool send_heartbeat() override {
        // Verstuur WIRE_OP_HEARTBEAT frame over SPI
        return true;
    }

    void emergency_stop(const std::string& reason) override {
        // Verstuur WIRE_OP_EMERGENCY_STOP en bekrachtig remmen
    }

    bool reset_emergency_stop() override {
        // Verstuur WIRE_OP_RESET_EMERGENCY indien veilig
        return true;
    }

    bool set_motion(const MotionCommand& cmd) override {
        // Converteer float waarden naar WireMotionCmd integer representaties en verstuur over SPI
        return true;
    }

    bool set_brake(float force_pct) override {
        // Converteer 0.0f-1.0f naar WireBrakeCmd (0-100%) en verstuur over SPI
        return true;
    }

    // ... overige virtuele methoden implementeren ...
};

} // namespace BikeHardware
```

## Lokaal Documentatie Previewen

Zorg dat `doxygen` en Python geïnstalleerd zijn op je systeem:

```bash
# Dependencies (Ubuntu)
sudo apt update && sudo apt install -y doxygen graphviz

# Virtuele Python omgeving
python3 -m venv .venv
source .venv/bin/activate
pip install -r docs/requirements.txt

# Start lokale server
mkdocs serve
```
