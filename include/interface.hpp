/**
 * @file interface.hpp
 * @brief Abstracte C++ Hardware Abstraction Layer (HAL) interface.
 *
 * Dit bestand definieert de abstracte klasse @ref BikeHardware::Interface.

 * @note ICT implementeert de applicatielogica tegen deze abstracte interface.
 *       Elektronica levert de concrete afgeleide klasse die de commando's en
 *       telemetrie vertaalt (bv. over SPI via @ref protocol.h).
 *
 * @see types.hpp
 * @see protocol.h
 */

#pragma once

#include "types.hpp"
#include <functional>
#include <string>

namespace BikeHardware {

/**
 * @brief Abstracte C++ interface tussen ICT en de onderliggende fietshardware.
 * 
 * Sensoren op rpi: GPS, Camera
 */
class Interface {
public:
    virtual ~Interface() = default;

    // ==========================================
    // 1. Initialisatie & Veiligheid
    // ==========================================
    
    virtual bool init() = 0;
    virtual void shutdown() = 0;

    /**
     * @brief stuur periodieke heartbeat (bv. max elke 50-100ms).
     */
    virtual bool send_heartbeat() = 0;

    /**
     * @brief activeer noodstop
     */
    virtual void emergency_stop(const std::string& reason) = 0;

    /**
     * @brief reset softwarematig de noodstop
     */
    virtual bool reset_emergency_stop() = 0;

    // ==========================================
    // 2. Beweging, Balans & Remmen
    // ==========================================

    /**
     * @brief zend stuurhoek en aandrijfsnelheid
     */
    virtual bool set_motion(const MotionCommand& cmd) = 0;

    /**
     * @brief analoge remsturing
     * @param force_pct waarde van 0.0f (geen rem) tot 1.0f (volledig remmen)
     */
    virtual bool set_brake(float force_pct) = 0;

    /**
     * @brief stel een stabilisatie-/kantelhoek offset in
     * @param offset_deg gewenste kantelhoek in graden (+/-)
     */
    virtual bool set_balance_offset(float offset_deg) = 0;

    // ==========================================
    // 3. Actuatoren & Randapparatuur
    // ==========================================

    /**
     * @brief zet de verlichting aan/uit of automatisch (op basis van lichtsensor)
     */
    virtual bool set_lights(LightMode mode) = 0;

    /**
     * @brief stuur de richtingaanwijzers aan (links, rechts, alarmlichten of uit)
     */
    virtual bool set_indicators(IndicatorMode mode) = 0;

    /**
     * @brief activeer of deactiveer de staander
     */
    virtual bool set_kickstand(bool deploy) = 0;

    /**
     * @brief activeer of deactiveer het slot
     */
    virtual bool set_lock(bool locked) = 0;

    /**
     * @brief speel een geluid gedurende een bepaalde tijd
     * @param duration_ms duur van het geluid in milliseconden
     */
    virtual bool trigger_sound(uint16_t duration_ms = 300) = 0;

    // ==========================================
    // 4. Modusbeheer (Manuele overname)
    // ==========================================

    /**
     * @brief zet de besturingsmodus (bv. STANDBY, ACTIVE_AUTONOMOUS, MANUAL_REMOTE_CONTROL)
     */
    virtual bool set_control_mode(SystemStatus mode) = 0;

    // ==========================================
    // 5. Telemetrie & Diagnose
    // ==========================================

    /**
     * @brief vraag de meest recente telemetrie op
     */
    virtual bool get_telemetry(BikeTelemetry& out_telemetry) = 0;

    /**
     * @brief registreer een callback voor asynchrone updates van snelle sensordata
     */
    virtual void register_telemetry_callback(std::function<void(const BikeTelemetry&)> callback) = 0;

    /**
     * @brief vraag de huidige status van het systeem op
     */
    virtual SystemStatus get_status() const = 0;
};

} // namespace BikeHardware