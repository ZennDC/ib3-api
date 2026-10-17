/**
 * @file protocol.hpp
 * @brief Low-level SPI wire-protocol definitions between Raspberry Pi 5 and ESP32.
 *
 * @details
 * ### SPI Master/Slave Architecture & Handshaking
 * - **SPI Master**: Raspberry Pi 5 (initiates all SPI clock pulses and CS assertions).
 * - **SPI Slave**: ESP32 microcontroller (Safety & Real-Time Actuator Controller).
 * - **Interrupt Line (DATA_READY GPIO)**:
 *   Because an SPI Slave cannot autonomously transmit data, the ESP32 asserts a dedicated
 *   hardware interrupt line (active-low or rising edge on a designated RPi GPIO pin) when
 *   new fast kinematic telemetry (100–200 Hz) is ready in its DMA buffer. Upon receiving
 *   this interrupt, the RPi triggers a full-duplex SPI transaction sending a polling/request
 *   opcode (@ref Opcode::REQUEST_FAST_TELEMETRY) to clock out the pending frame.
 *
 * ### Frame Integrity & EMI Protection
 * Brushless drive motors and high-current power stages induce heavy electromagnetic interference (EMI).
 * To guard against frame corruption and bitflips:
 * 1. Synchronous preamble bytes (`MAGIC_1` + `MAGIC_2`) allow the stream parser to discard line noise.
 * 2. A 16-bit CRC (CRC16-CCITT / Kermit) verifies payload integrity prior to execution.
 * 3. A frame delimiter (`TAIL`) guarantees packet boundary alignment.
 *
 * @see interface.hpp
 * @see types.hpp
 */

#pragma once

#include <cstdint>

namespace BikeHardware::Protocol
{

    /**
     * @name Framing Delimiters & Synchronization Constants
     * @{
     */
    /**
     * @brief First synchronization preamble byte.
     * Marks candidate start-of-frame across the SPI byte stream.
     */
    inline constexpr uint8_t MAGIC_1 = 0xAA;

    /**
     * @brief Second synchronization preamble byte.
     * Paired with MAGIC_1 (0xAA 0x55) to eliminate false positives in raw telemetry data streams.
     */
    inline constexpr uint8_t MAGIC_2 = 0x55;

    /**
     * @brief Frame termination delimiter ('\n' / 0x0A).
     * Used for boundary integrity checks and frame synchronization recovery.
     */
    inline constexpr uint8_t TAIL = 0x0A;
    /** @} */

    /**
     * @brief Safety and fault bitmask flags reported in WireFastTelemetry.
     */
    namespace SafetyFlag
    {
        inline constexpr uint8_t EMERGENCY_STOP   = (1 << 0); ///< Hardware or software emergency stop active.
        inline constexpr uint8_t MOTOR_STALL      = (1 << 1); ///< Drive motor stalled or overcurrent condition.
        inline constexpr uint8_t WATCHDOG_TIMEOUT = (1 << 2); ///< Watchdog heartbeat timeout occurred.
        inline constexpr uint8_t FALL_DETECTED    = (1 << 3); ///< Bicycle fall or excessive lean angle detected by IMU.
        inline constexpr uint8_t HARDWARE_FAULT   = (1 << 4); ///< General Safety PCB or power subsystem fault.
        inline constexpr uint8_t SENSOR_FAULT     = (1 << 5); ///< Disconnected or corrupted sensor data stream.
    }

    /**
     * @brief Actuator status feedback bitmask flags reported in WireFastTelemetry.
     */
    namespace ActuatorFlag
    {
        inline constexpr uint8_t KICKSTAND = (1 << 0); ///< 1: Kickstand deployed (parked), 0: Retracted.
        inline constexpr uint8_t LOCK      = (1 << 1); ///< 1: Frame lock engaged, 0: Released.
    }

    /**
     * @brief Wire protocol message opcodes for bus communications.
     */
    enum class Opcode : uint8_t
    {
        // Master -> Slave Control Commands (0x01 - 0x1F)
        HEARTBEAT              = 0x01, ///< Periodic watchdog heartbeat ping.
        EMERGENCY_STOP         = 0x02, ///< Trigger emergency stop immediately.
        RESET_EMERGENCY        = 0x03, ///< Reset software emergency stop state.
        SET_MODE               = 0x04, ///< Configure operational mode (Standby, Autonomous, Manual, Deep Sleep).
        SET_MOTION             = 0x10, ///< Set motion setpoints (speed, accel, steering, rotations).
        SET_BRAKE              = 0x11, ///< Set differentiated front/rear/regenerative braking force.
        SET_ACTUATORS          = 0x12, ///< Set actuator states (lights, blinkers, kickstand, lock, sound).
        SET_BAL_OFFSET         = 0x13, ///< Set active balance lean offset angle.

        // Master -> Slave Telemetry Requests / Polling (0x20 - 0x2F)
        REQUEST_FAST_TELEMETRY  = 0x20, ///< Clock out latest fast kinematic telemetry (response to DATA_READY).
        REQUEST_SLOW_TELEMETRY  = 0x21, ///< Request thermal and dynamic diagnostics telemetry.
        REQUEST_SUPER_SLOW_TELEM= 0x22, ///< Request static battery, pressure, and air quality telemetry.

        // Slave -> Master Telemetry Streams / Responses (0x80 - 0x8F)
        TELEMETRY_FAST         = 0x80, ///< High-frequency kinematic telemetry stream (100–200 Hz).
        TELEMETRY_SLOW         = 0x81, ///< Medium-frequency dynamic diagnostic stream (1–5 Hz).
        TELEMETRY_SUPER_SLOW   = 0x82  ///< Low-frequency environmental and BMS telemetry (0.05–0.2 Hz).
    };

#pragma pack(push, 1)

    /**
     * @brief Universal packet framing header prepended to every transmission.
     */
    struct WireHeader
    {
        uint8_t magic1{MAGIC_1}; ///< Synchronization magic byte 1 (0xAA).
        uint8_t magic2{MAGIC_2}; ///< Synchronization magic byte 2 (0x55).
        uint8_t opcode{0};       ///< Packet opcode (cast from @ref Opcode).
        uint16_t seq_id{0};      ///< Monotonically increasing packet sequence counter.
        uint8_t payload_len{0};  ///< Length of subsequent payload bytes (excluding CRC and TAIL).
    };

    /**
     * @brief Command payload to configure the active operational mode.
     */
    struct WireSetModeCmd
    {
        uint8_t system_status;   ///< Target status: 0=Standby, 1=Autonomous, 2=Manual, 3=E-Stop, 4=Fault, 5=Deep Sleep.
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
     * @brief Differentiated analog braking command payload.
     * Prevents forward pitchover by decoupling front and rear braking forces.
     */
    struct WireBrakeCmd
    {
        uint8_t front_brake_force_pct;     ///< Front brake force from 0 (released) to 100 (maximum).
        uint8_t rear_brake_force_pct;      ///< Rear brake force from 0 (released) to 100 (maximum).
        uint8_t enable_regenerative;       ///< 1: Enable regenerative electric braking, 0: Disabled.
    };

    /**
     * @brief Balance lean/tilt offset command payload.
     */
    struct WireBalanceOffsetCmd
    {
        int16_t balance_offset_centi_deg;  ///< Desired lean offset in centi-degrees (+/-, e.g. static lock).
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
        uint8_t sound_type;         ///< Sound profile: 0=Bell, 1=Horn, 2=Warning, 3=Alarm.
        uint16_t sound_duration_ms; ///< Acoustic warning duration in ms (0 = stop/silent).
    };

    /**
     * @brief Scaled integer 9-DoF IMU measurements.
     */
    struct WireIMUData
    {
        int16_t accel_x_mg;       ///< Linear acceleration X in milli-g (+/- 16G range).
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

        WireIMUData imu;                    ///< 9-DoF frame IMU measurements (+/- 16G, polled at 200 Hz).

        uint8_t safety_flags;               ///< Active safety flags bitmask (see SafetyFlag::*).
        uint8_t actuator_feedback;          ///< Physical actuator feedback bitmask (see ActuatorFlag::*).
    };

    /**
     * @brief Medium-rate diagnostic telemetry payload (polled at 1–5 Hz).
     *
     * Transmits rapidly fluctuating thermal and electrical diagnostics
     * (motor currents, temperatures, and dynamic voltages).
     */
    struct WireSlowTelemetry
    {
        uint16_t motor_voltage_mv;          ///< Drive motor operational voltage in millivolts (mV).
        int16_t motor_current_ma;           ///< Drive motor current draw in mA.
        int16_t motor_temp_centi_c;         ///< Motor winding temperature in centi-Celsius.
        int16_t controller_temp_centi_c;    ///< Motor controller MOSFET temperature in centi-Celsius.
        uint16_t light_lux;                 ///< Ambient illuminance in lux.
    };

    /**
     * @brief Low-frequency telemetry payload (polled at 0.05–0.2 Hz / every 5–20 seconds).
     *
     * Transmits slowly changing auxiliary metrics to conserve processing power,
     * bus bandwidth, and remote cellular/LTE transmission costs.
     */
    struct WireSuperSlowTelemetry
    {
        uint16_t battery_voltage_mv;        ///< Total battery pack voltage in millivolts (mV).
        int16_t battery_current_ma;         ///< Total battery current in mA (+ = discharging, - = charging/solar).
        uint8_t battery_soc_pct;            ///< Battery State of Charge in percent (0–100%).
        int16_t battery_temp_centi_c;       ///< Battery pack internal temperature in centi-Celsius.

        uint16_t tire_pressure_front_mbar;  ///< Front tire pressure in millibar (mbar).
        uint16_t tire_pressure_rear_mbar;   ///< Rear tire pressure in millibar (mbar).

        int16_t ambient_temp_centi_c;       ///< Ambient air temperature in centi-Celsius.
        uint16_t ambient_humidity_centi_pct;///< Relative humidity in centi-percent (0–10000 = 0.00% to 100.00%).
        uint16_t air_quality_index;         ///< Air Quality Index score (0xFFFF if unavailable).
    };

    /**
     * @brief Complete transmission frame envelope including preamble, header, payload, CRC16, and tail.
     *
     * @tparam T Specific command or telemetry payload type.
     */
    template <typename T>
    struct WireFrame
    {
        WireHeader header;                  ///< Standard synchronization and metadata header.
        T payload;                          ///< Specific packet payload.
        uint16_t crc16;                     ///< CRC-16 calculated over header and payload bytes.
        uint8_t tail{TAIL};                 ///< End of frame delimiter byte ('\n' / 0x0A).
    };

#pragma pack(pop)

} // namespace BikeHardware::Protocol
