/**
 * @file types.hpp
 * @brief Common data models, datatypes, enums, and error flags.
 *
 * This file contains all high-level C++ data structures and types exchanged
 * via the @ref BikeHardware::Interface.
 *
 * @see interface.hpp
 * @see protocol.hpp
 */

#pragma once

#include <cstdint>
#include <string>

namespace BikeHardware
{

    /**
     * @brief Headlight and taillight operating modes.
     */
    enum class LightMode : uint8_t
    {
        OFF = 0, ///< Lights permanently off.
        ON = 1,  ///< Lights permanently on.
        AUTO = 2 ///< Automatic lighting controlled via ambient light sensor.
    };

    /**
     * @brief Turn indicator signaling modes.
     */
    enum class IndicatorMode : uint8_t
    {
        OFF = 0,   ///< Indicators disabled.
        LEFT = 1,  ///< Left indicator blinking.
        RIGHT = 2, ///< Right indicator blinking.
        HAZARD = 3 ///< Hazard lights blinking (both directions).
    };

    /**
     * @brief Safety and diagnostic bitmask flags for fault detection.
     */
    namespace SafetyFlag
    {
        inline constexpr uint8_t EMERGENCY_STOP_ACTIVE = 1 << 0; ///< Hardware or software emergency stop active.
        inline constexpr uint8_t MOTOR_STALL = 1 << 1;           ///< Drive motor stalled or overcurrent detected.
        inline constexpr uint8_t WATCHDOG_TIMEOUT = 1 << 2;      ///< Communication heartbeat watchdog timeout.
        inline constexpr uint8_t FALL_DETECTED = 1 << 3;         ///< Bicycle fall or excessive tilt angle detected by IMU.
        inline constexpr uint8_t HARDWARE_FAULT = 1 << 4;        ///< General hardware fault on Safety PCB or power rail.
        inline constexpr uint8_t SENSOR_FAULT = 1 << 5;          ///< Missing, disconnected, or corrupted sensor telemetry stream.
    }

    /**
     * @brief Real-Time Kinematic (RTK) GNSS positioning status.
     *
     * Indicates fix status from high-precision multi-band GNSS receivers
     * (e.g. u-blox ZED-F9P, Quectel LC29H, Unicore UM980).
     */
    enum class RTKStatus : uint8_t
    {
        NONE = 0,       ///< No GNSS fix acquired.
        AUTONOMOUS = 1, ///< Standard 3D GNSS fix (~3 m horizontal accuracy).
        FLOAT = 2,      ///< RTK Float fix (sub-meter / decimeter precision).
        FIX = 3         ///< RTK Fixed solution (centimeter-level precision).
    };

    /**
     * @brief Geodetic GPS position, ground velocity, and RTK accuracy metrics.
     *
     * Provides geodetic coordinates, velocity vectors, and positioning quality metrics
     * read either directly on the Raspberry Pi 5 (via Pi-HAT) or through the HAL.
     */
    struct GPS
    {
        double latitude = 0.0;   ///< Geodetic latitude in degrees (-90.0 to +90.0).
        double longitude = 0.0;  ///< Geodetic longitude in degrees (-180.0 to +180.0).
        float altitude_m = 0.0f; ///< Altitude above mean sea level in meters.

        /**
         * @brief Ground speed in meters per second (m/s).
         * @note Auxiliary reference metric only. Only valid while vehicle is actively in motion. For real-time feedback control loops,
         *       always prioritize wheel-tachometer feedback via @ref BikeTelemetry::current_speed_mps.
         */
        float speed_over_ground_mps = 0.0f;

        /**
         * @brief Ground course / heading in degrees (0.0 to 360.0).
         * @note Only valid while vehicle is actively in motion. When stationary or at low speeds,
         *       rely on IMU yaw orientation via @ref IMUData::orientation_deg.
         */
        float heading_deg = 0.0f;

        float horizontal_accuracy_m = 0.0f;     ///< Estimated horizontal accuracy in meters (cm-level under RTK Fix).
        float vertical_accuracy_m = 0.0f;       ///< Estimated vertical accuracy in meters.
        uint8_t satellites_visible = 0;         ///< Number of tracked/used satellites.
        RTKStatus rtk_status = RTKStatus::NONE; ///< RTK fix status.
    };

    /**
     * @brief Motion and steering setpoints dispatched from ICT to hardware.
     *
     * Specifies target longitudinal speed, acceleration profile, steering angle,
     * and execution duration expressed in wheel rotations.
     */
    struct MotionCommand
    {
        float target_speed_mps = 0.0f;         ///< Desired longitudinal speed in meters per second (m/s).
        float target_acceleration_mps2 = 0.5f; ///< Forward acceleration limit in m/s^2 (configurable per driving style).
        float target_steering_deg = 0.0f;      ///< Desired front steering angle in degrees (+/-).
        float duration_rotations = 1.0f;       ///< Target execution duration expressed in wheel rotations.
    };

    /**
     * @brief Generic 3-axis floating point Cartesian vector.
     */
    struct Vector3D
    {
        float x = 0.0f; ///< X-axis value.
        float y = 0.0f; ///< Y-axis value.
        float z = 0.0f; ///< Z-axis value.
    };

    /**
     * @brief 9-Degrees-of-Freedom (9-DoF) inertial measurement unit data.
     *
     * Captures 3-axis linear acceleration, 3-axis angular velocity, and estimated
     * orientation angles from the frame-mounted IMU (up to 16G range for bump detection, polled at 200 Hz).
     */
    struct IMUData
    {
        Vector3D acceleration_g;       ///< Linear acceleration in g (1 g = 9.81 m/s^2, up to +/- 16G for bump detection).
        Vector3D angular_velocity_dps; ///< Angular velocity in degrees per second (dps).
        Vector3D orientation_deg;      ///< Estimated orientation angles (roll, pitch, yaw) in degrees.
    };

    /**
     * @brief Comprehensive bicycle state and sensor telemetry snapshot.
     *
     * Aggregates positioning, kinematics, odometry, inertial measurements,
     * battery BMS state, actuator feedback, safety flags, and environmental readings.
     */
    struct BikeTelemetry
    {
        uint64_t timestamp_ms = 0; ///< Monotonic measurement timestamp in milliseconds.

        GPS gps;                            ///< RTK GNSS position and precision metrics (cm precision).
        float current_speed_mps = 0.0f;     ///< Current forward speed derived from wheel rotations (m/s).
        uint32_t total_wheel_rotations = 0; ///< Cumulative wheel rotations count (odometry via tachometer).
        float current_steering_deg = 0.0f;  ///< Current front fork steering angle in degrees (+/-).

        IMUData imu; ///< 9-DoF frame-mounted IMU (up to 16G bump detection, 200 Hz polling).

        float battery_soc_pct = 0.0f;   ///< Battery State of Charge in percent (0.0% to 100.0%).
        float battery_voltage_v = 0.0f; ///< Total battery pack voltage in volts (V).
        float battery_current_a = 0.0f; ///< Battery current in amperes (positive = discharging, negative = regenerative charging / solar).

        // Actuator physical feedback
        bool kickstand_deployed = false;                    ///< Physical status of kickstand (true = deployed/parked).
        bool lock_engaged = false;                          ///< Physical status of frame lock (true = locked).
        IndicatorMode indicator_state = IndicatorMode::OFF; ///< Current physical turn indicator state.

        // Safety and diagnostic flags
        uint8_t safety_flags = 0; ///< Active safety flags bitmask (see @ref BikeHardware::SafetyFlag).

        // Active balance status
        float balance_offset_deg = 0.0f; ///< Active balance/lean offset angle in degrees.

        // Optional environmental sensor telemetry (-1.0f if sensor is absent or unread)
        float ambient_temperature_c = -1.0f;   ///< Ambient air temperature in degrees Celsius (°C).
        float ambient_humidity_pct = -1.0f;    ///< Relative ambient humidity in percent (0.0% to 100.0%).
        float ambient_light_lux = 0.0f;        ///< Ambient illuminance in lux.
        float tire_pressure_front_bar = -1.0f; ///< Front tire pressure in bar.
        float tire_pressure_rear_bar = -1.0f;  ///< Rear tire pressure in bar.
        float air_quality_index = -1.0f;       ///< Air Quality Index score (-1 if unavailable).
    };

    /**
     * @brief High-level operational control mode of the bicycle.
     */
    enum class SystemStatus : uint8_t
    {
        STANDBY = 0,               ///< System powered on but actuators disengaged.
        ACTIVE_AUTONOMOUS = 1,     ///< Autonomous driving and path following active.
        MANUAL_REMOTE_CONTROL = 2, ///< Manual teleoperation / controller override.
        EMERGENCY_STOP = 3,        ///< Emergency stop active, drive cut, brakes engaged.
        HARDWARE_FAULT = 4,        ///< Hardware or safety PCB critical failure.
        DEEP_SLEEP = 5             ///< Low-power sleep mode to conserve battery when parked.
    };

    /**
     * @brief Severity levels for diagnostics and system logging.
     */
    enum class LogLevel : uint8_t
    {
        DEBUG = 0,   ///< Low-level diagnostics, register states, and bus tracing.
        INFO = 1,    ///< Informational operational messages.
        WARNING = 2, ///< Non-critical warnings or degraded conditions.
        ERROR = 3,   ///< Recoverable faults or rejected commands.
        CRITICAL = 4 ///< Safety-critical failures requiring immediate shutdown.
    };

    /**
     * @brief Acoustic warning sound types.
     */
    enum class SoundType : uint8_t
    {
        BELL = 0,    ///< Standard bicycle bell ring.
        HORN = 1,    ///< Loud acoustic horn for vehicle warning.
        WARNING = 2, ///< Caution / proximity alert tone.
        ALARM = 3    ///< High-priority anti-theft or emergency alarm siren.
    };

    /**
     * @brief Differentiated analog braking setpoints and energy recovery.
     *
     * Provides separate front/rear brake force controls to prevent over-the-bars pitchover
     * during hard deceleration, alongside optional regenerative braking.
     */
    struct BrakeCommand
    {
        float front_force_pct = 0.0f;     ///< Front mechanical brake force (0.0f = released, 1.0f = full force).
        float rear_force_pct = 0.0f;      ///< Rear mechanical brake force (0.0f = released, 1.0f = full force).
        bool enable_regenerative = false; ///< Enable motor regenerative braking / energy recovery.
    };

} // namespace BikeHardware
