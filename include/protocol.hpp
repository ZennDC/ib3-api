/**
 * @file protocol.hpp
 * @brief Low-level SPI wire-protocol definitions between Raspberry Pi 5 and ESP32.
 *
 * @note DRAFT: This protocol is a working draft serving as the baseline for
 *       communication between the RPi 5 and ESP32. Further optimizations by
 *       the Electronics team are possible.
 *
 * @see interface.hpp
 * @see types.hpp
 */

#pragma once

#include <cstdint>

namespace BikeHardware::Protocol
{

    // Packet framing delimiters
    inline constexpr uint8_t MAGIC_1 = 0xAA; ///< First packet synchronization magic byte (0xAA).
    inline constexpr uint8_t MAGIC_2 = 0x55; ///< Second packet synchronization magic byte (0x55).
    inline constexpr uint8_t TAIL = 0x0A;    ///< Frame tail delimiter byte ('\n').

    /**
     * @brief Safety and fault bitmask flags reported in WireFastTelemetry.
     */
    namespace SafetyFlag
    {
        inline constexpr uint8_t EMERGENCY_STOP = (1 << 0);   ///< Hardware or software emergency stop active.
        inline constexpr uint8_t MOTOR_STALL = (1 << 1);      ///< Drive motor stalled or overcurrent condition.
        inline constexpr uint8_t WATCHDOG_TIMEOUT = (1 << 2); ///< Watchdog heartbeat timeout occurred.
        inline constexpr uint8_t FALL_DETECTED = (1 << 3);    ///< Bicycle fall or excessive lean angle detected by IMU.
        inline constexpr uint8_t HARDWARE_FAULT = (1 << 4);   ///< General Safety PCB or power subsystem fault.
    }

    /**
     * @brief Actuator status feedback bitmask flags reported in WireFastTelemetry.
     */
    namespace ActuatorFlag
    {
        inline constexpr uint8_t KICKSTAND = (1 << 0); ///< 1: Kickstand deployed (parked), 0: Retracted.
        inline constexpr uint8_t LOCK = (1 << 1);      ///< 1: Frame lock engaged, 0: Released.
    }

    /**
     * @brief Wire protocol message opcodes for bus communications.
     */
    enum class Opcode : uint8_t
    {
        HEARTBEAT = 0x01,       ///< Periodic watchdog heartbeat ping.
        EMERGENCY_STOP = 0x02,  ///< Trigger emergency stop immediately.
        RESET_EMERGENCY = 0x03, ///< Reset software emergency stop state.
        SET_MODE = 0x04,        ///< Configure operational mode (Standby, Autonomous, Manual).
        SET_MOTION = 0x10,      ///< Set motion setpoints (speed, accel, steering, rotations).
        SET_BRAKE = 0x11,       ///< Set analog braking force percentage.
        SET_ACTUATORS = 0x12,   ///< Set actuator states (lights, blinkers, kickstand, lock, sound).
        SET_BAL_OFFSET = 0x13,  ///< Set active balance lean offset angle.

        TELEMETRY_FAST = 0x80, ///< High-frequency kinematic telemetry stream (100–200 Hz).
        TELEMETRY_SLOW = 0x81  ///< Low-frequency diagnostic telemetry stream (1–5 Hz).
    };

#pragma pack(push, 1)

    /**
     * @brief Universal packet framing header prepended to every wire transmission.
     */
    struct WireHeader
    {
        uint8_t magic1{MAGIC_1}; ///< Synchronization magic byte 1 (0xAA).
        uint8_t magic2{MAGIC_2}; ///< Synchronization magic byte 2 (0x55).
        uint8_t opcode{0};       ///< Packet opcode (cast from @ref Opcode).
        uint16_t seq_id{0};      ///< Monotonically increasing packet sequence counter.
        uint8_t payload_len{0};  ///< Length of subsequent payload bytes.
    };

    /**
     * @brief Command payload to configure the active operational mode.
     */
    struct WireSetModeCmd
    {
        uint8_t system_status; ///< Target status: 0=Standby, 1=Autonomous, 2=Manual remote, etc.
    };

    /**
     * @brief Scaled integer motion command payload for minimal bus bandwidth.
     */
    struct WireMotionCmd
    {
        int16_t target_speed_mms;          ///< Target speed in millimeters per second (mm/s).
        int16_t target_accel_mms2;         ///< Target acceleration limit in mm/s^2.
        int16_t target_steering_centi_deg; ///< Target steering angle in centi-degrees (1/100 deg, +/-).
        uint16_t duration_rotations_centi; ///< Target duration in centi-rotations (1/100 wheel rotations).
    };

    /**
     * @brief Proportional analog braking command payload.
     */
    struct WireBrakeCmd
    {
        uint8_t brake_force_pct; ///< Braking force from 0 (released) to 100 (maximum).
    };

    /**
     * @brief Balance lean/tilt offset command payload.
     */
    struct WireBalanceOffsetCmd
    {
        int16_t balance_offset_centi_deg; ///< Desired lean offset in centi-degrees (+/-, e.g. static lock).
    };

    /**
     * @brief Peripheral actuators command payload.
     */
    struct WireActuatorsCmd
    {
        uint8_t lights_mode;        ///< Lighting: 0=Off, 1=On, 2=Auto (ambient light sensor).
        uint8_t indicator_mode;     ///< Blinkers: 0=Off, 1=Left, 2=Right, 3=Hazard.
        uint8_t kickstand_deploy;   ///< Kickstand: 0=Retracted, 1=Deployed (parked).
        uint8_t lock_engaged;       ///< Electronic lock: 0=Unlocked, 1=Locked.
        uint16_t sound_duration_ms; ///< Acoustic warning duration in ms (0 = stop/silent).
    };

    /**
     * @brief Scaled integer 9-DoF IMU measurements.
     */
    struct WireIMUData
    {
        int16_t accel_x_mg;       ///< Linear acceleration X in milli-g (+/- 16G range for bump detection).
        int16_t accel_y_mg;       ///< Linear acceleration Y in milli-g (+/- 16G range).
        int16_t accel_z_mg;       ///< Linear acceleration Z in milli-g (+/- 16G range).
        int16_t gyro_x_dps_centi; ///< Angular velocity X in centi-degrees/sec (1/100 dps).
        int16_t gyro_y_dps_centi; ///< Angular velocity Y in centi-degrees/sec (1/100 dps).
        int16_t gyro_z_dps_centi; ///< Angular velocity Z in centi-degrees/sec (1/100 dps).
        int16_t roll_centi_deg;   ///< Roll orientation angle in centi-degrees (1/100 deg).
        int16_t pitch_centi_deg;  ///< Pitch orientation angle in centi-degrees (1/100 deg).
        int16_t yaw_centi_deg;    ///< Yaw orientation angle in centi-degrees (1/100 deg).
    };

    /**
     * @brief High-rate fast telemetry payload (polled at 100–200 Hz).
     *
     * Contains critical real-time kinematics required for balance control,
     * trajectory tracking, and safety watchdog monitoring.
     */
    struct WireFastTelemetry
    {
        uint32_t timestamp_ms;              ///< Board timestamp in milliseconds.
        int16_t current_speed_mms;          ///< Speed over ground derived from wheel rotations (mm/s).
        uint32_t total_wheel_rotations;     ///< Cumulative wheel rotation count (tachometer odometry).
        int16_t current_steering_centi_deg; ///< Fork steering angle in centi-degrees (1/100 deg, +/-).

        WireIMUData imu; ///< 9-DoF frame IMU measurements (+/- 16G, polled at 200 Hz).

        uint8_t safety_flags;      ///< Active safety flags bitmask (see SafetyFlag::*).
        uint8_t actuator_feedback; ///< Physical actuator feedback bitmask (see ActuatorFlag::*).
    };

    /**
     * @brief Low-rate diagnostic telemetry payload (polled at 1–5 Hz).
     *
     * Transmits auxiliary metrics such as battery state, regenerative current,
     * ambient conditions, tire pressure, and environmental air quality.
     */
    struct WireSlowTelemetry
    {
        uint16_t battery_voltage_mv; ///< Battery pack voltage in millivolts (mV).
        int16_t battery_current_ma;  ///< Battery current in mA (positive = discharge, negative = charge/regen).
        uint8_t battery_soc_pct;     ///< Battery State of Charge in percent (0–100%).
        uint16_t light_lux;          ///< Ambient illuminance in lux.

        int16_t ambient_temp_centi_c;        ///< Ambient temperature in centi-Celsius (-1000 = absent/fault).
        uint16_t ambient_humidity_centi_pct; ///< Relative humidity in centi-percent (0–10000 = 0.00% to 100.00%).
        uint16_t tire_pressure_front_mbar;   ///< Front tire pressure in millibar (mbar).
        uint16_t tire_pressure_rear_mbar;    ///< Rear tire pressure in millibar (mbar).
        uint16_t air_quality_index;          ///< Air Quality Index score (0xFFFF if unavailable).
    };

#pragma pack(pop)

} // namespace BikeHardware::Protocol
