/**
 * @file interface.hpp
 * @brief Abstract C++ Hardware Abstraction Layer (HAL) interface.
 *
 * This file defines the abstract class @ref BikeHardware::Interface.
 *
 * @note ICT implements the application logic against this abstract interface.
 *       Electronics provides the concrete derived class that translates commands
 *       and telemetry (e.g., over SPI via @ref protocol.h).
 *
 * @see types.hpp
 * @see protocol.h
 */

#pragma once

#include "types.hpp"
#include <functional>
#include <string>

namespace BikeHardware
{

    /**
     * @brief Pure abstract C++ Hardware Abstraction Layer (HAL) interface.
     */
    class Interface
    {
    public:
        virtual ~Interface() = default;

        /**
         * @brief Initialize communication buses and perform hardware handshake.
         */
        virtual bool init() = 0;

        /**
         * @brief Safely bring actuators to a standstill and close communication links.
         */
        virtual void shutdown() = 0;

        /**
         * @brief Transmit periodic watchdog heartbeat (e.g. every 50-100 ms) to prevent emergency stop.
         */
        virtual bool send_heartbeat() = 0;

        /**
         * @brief Trigger a software emergency stop and immediately disengage drive power.
         * @param reason Human-readable diagnostic reason for emergency stop activation.
         */
        virtual void emergency_stop(const std::string &reason) = 0;

        /**
         * @brief Reset software emergency stop state once fault conditions are cleared.
         */
        virtual bool reset_emergency_stop() = 0;

        /**
         * @brief Dispatch a motion command with steering angle, target speed, acceleration limit, and duration.
         * @param cmd Motion parameters (target speed, acceleration profile, steering angle, wheel rotations).
         */
        virtual bool set_motion(const MotionCommand &cmd) = 0;

        /**
         * @brief Apply proportional analog braking (mechanical friction and/or regenerative energy recovery).
         * @param force_pct Braking force normalized from 0.0f (released) to 1.0f (full braking).
         */
        virtual bool set_brake(float force_pct) = 0;

        /**
         * @brief Set or lock a stabilization lean/tilt angle offset (balance manipulation).
         * @param offset_deg Desired lean offset angle in degrees (+/-), e.g. 5.0f for cornering or static lock.
         */
        virtual bool set_balance_offset(float offset_deg) = 0;

        /**
         * @brief Configure headlight and taillight state (off, on, or automatic via ambient sensor).
         * @param mode Target lighting mode.
         */
        virtual bool set_lights(LightMode mode) = 0;

        /**
         * @brief Control turn indicator blinkers (off, left, right, or hazard lights).
         * @param mode Desired indicator mode.
         */
        virtual bool set_indicators(IndicatorMode mode) = 0;

        /**
         * @brief Deploy or retract the automated parking kickstand.
         * @param deploy True to deploy kickstand (parking), false to retract.
         */
        virtual bool set_kickstand(bool deploy) = 0;

        /**
         * @brief Engage or disengage the electronic bicycle frame lock.
         * @param locked True to lock bicycle frame/wheel, false to release.
         */
        virtual bool set_lock(bool locked) = 0;

        /**
         * @brief Emit an acoustic warning signal (electronic bell / buzzer) for a set duration.
         * @param duration_ms Duration of sound in milliseconds (default: 300 ms).
         */
        virtual bool trigger_sound(uint16_t duration_ms = 300) = 0;

        /**
         * @brief Configure operational control mode (STANDBY, ACTIVE_AUTONOMOUS, MANUAL_REMOTE_CONTROL, etc.).
         * @param mode Desired system status mode.
         */
        virtual bool set_control_mode(SystemStatus mode) = 0;

        /**
         * @brief Synchronously retrieve the latest complete telemetry snapshot.
         * @param[out] out_telemetry Destination struct populated with the freshest telemetry data.
         */
        virtual bool get_telemetry(BikeTelemetry &out_telemetry) = 0;

        /**
         * @brief Register an event callback for asynchronous telemetry streaming.
         * @param callback Function invoked upon receiving new telemetry updates.
         */
        virtual void register_telemetry_callback(std::function<void(const BikeTelemetry &)> callback) = 0;

        /**
         * @brief Query the current operational state of the bicycle system.
         */
        virtual SystemStatus get_status() const = 0;
    };

} // namespace BikeHardware
