/**
 * @file types.hpp
 * @brief Gemeenschappelijke datamodellen, datatypes, enums en foutvlaggen.
 *
 * Dit bestand bevat alle high-level C++ datastructuren en typen die worden uitgewisseld
 * via de @ref BikeHardware::Interface.
 * 
 * @see interface.hpp
 * @see protocol.h
 */

#pragma once

#include <cstdint>
#include <string>

namespace BikeHardware
{

    // verlichtingsmodes
    enum class LightMode : uint8_t
    {
        OFF = 0,
        ON = 1,
        AUTO = 2
    };

    // richtingsaanwijzers
    enum class IndicatorMode : uint8_t
    {
        OFF = 0,
        LEFT = 1,
        RIGHT = 2,
        HAZARD = 3
    };

    // veiligheids- en foutflags voor ICT foutdetectie
    namespace SafetyFlag
    {
        constexpr uint8_t EMERGENCY_STOP_ACTIVE = 1 << 0; // Hardware- of softwarenoodstop actief
        constexpr uint8_t MOTOR_STALL           = 1 << 1; // Motor geblokkeerd / overstroom
        constexpr uint8_t WATCHDOG_TIMEOUT      = 1 << 2; // Communicatie / heartbeat timeout
        constexpr uint8_t FALL_DETECTED         = 1 << 3; // Fiets gekanteld / val gedetecteerd
        constexpr uint8_t HARDWARE_FAULT        = 1 << 4; // Safety PCB storing
    }

    // gps-positie
    struct GPS
    {
        double latitude = 0.0;
        double longitude = 0.0;
        float altitude_m = 0.0f;
        float speed_over_ground_mps = 0.0f; // zou berekend worden door gps module ??
        float heading_deg = 0.0f;           // zou berekend worden door gps module ??
        uint8_t satellites_visible = 0;
    };

    // beweging- en stuurcommando
    struct MotionCommand
    {
        float target_speed_mps = 0.0f;         // gewenste snelheid (m/s)
        float target_acceleration_mps2 = 0.5f; // gewenste acceleratie (m/s^2)
        float target_steering_deg = 0.0f;      // gewenste stuurhoek (graden)
        float duration_rotations = 1.0f;       // duratie (# wielrotaties)
    };

    struct Vector3D
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct IMUData
    {
        Vector3D acceleration_g;       // versnelling (g: 9.81 m/s^2)
        Vector3D angular_velocity_dps; // (dps: graden per seconde)
        Vector3D orientation_deg;      // (roll, pitch, yaw) in graden
    };

    // toestand
    struct BikeTelemetry
    {
        uint64_t timestamp_ms = 0;

        GPS gps;
        float current_speed_mps = 0.0f;
        float current_steering_deg = 0.0f;

        IMUData frame_imu;     // imu op frame (9 DoF)
        IMUData handlebar_imu; // optioneel (9 DoF op stuur)

        float battery_soc_pct = 0.0f;
        float battery_voltage_v = 0.0f; // optioneel
        float battery_current_a = 0.0f; // optioneel

        // Terugkoppeling van fysieke actuatoren
        bool kickstand_deployed = false; // terugkoppeling fysieke status staander
        bool lock_engaged = false;       // terugkoppeling fysieke status slot
        IndicatorMode indicator_state = IndicatorMode::OFF;

        // Veiligheids- en foutflags voor ICT (zie SafetyFlag bitmask)
        uint8_t safety_flags = 0;

        // Balans status
        float balance_offset_deg = 0.0f; // ingestelde balans/kantelhoek offset

        // optionele sensordata
        float ambient_temperature_c = -1.0f;
        float ambient_humidity_pct = -1.0f;
        float ambient_light_lux = 0.0f;
        float tire_pressure_front_bar = -1.0f;
        float tire_pressure_rear_bar = -1.0f;
        float air_quality_index = -1.0f; // -1 indien niet aanwezig
    };

    enum class SystemStatus : uint8_t
    {
        STANDBY = 0,
        ACTIVE_AUTONOMOUS = 1,
        MANUAL_REMOTE_CONTROL = 2,
        EMERGENCY_STOP = 3,
        HARDWARE_FAULT = 4
    };

    enum class LogLevel
    {
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };

} // namespace BikeHardware