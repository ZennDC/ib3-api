/**
 * @file protocol.h
 * @brief Laagniveau SPI wire-protocol definities tussen Raspberry Pi 5 en ESP32.
 *
 * @note DRAFT: Dit protocol is een werkversie (draft) als basis voor de communicatie
 *       tussen RPi 5 en ESP32. Verdere optimalisaties door Elektronica zijn mogelijk.
 *
 * @see interface.hpp
 * @see types.hpp
 */

#pragma once

#include <stdint.h>

#define BIKE_PROTO_MAGIC_1      0xAA
#define BIKE_PROTO_MAGIC_2      0x55
#define BIKE_PROTO_TAIL         0x0A

// safety flags
#define BIKE_SAFETY_FLAG_EMERGENCY_STOP   (1 << 0) // Noodstop actief (hardware of software)
#define BIKE_SAFETY_FLAG_MOTOR_STALL      (1 << 1) // Motor geblokkeerd / overstroom
#define BIKE_SAFETY_FLAG_WATCHDOG_TIMEOUT (1 << 2) // Communicatie / heartbeat timeout
#define BIKE_SAFETY_FLAG_FALL_DETECTED    (1 << 3) // Fiets gekanteld / val gedetecteerd
#define BIKE_SAFETY_FLAG_HARDWARE_FAULT   (1 << 4) // Safety PCB algemene hardware storing

// actuator flags
#define BIKE_ACTUATOR_FLAG_KICKSTAND      (1 << 0) // 1: Uitgeklapt, 0: Ingeklapt
#define BIKE_ACTUATOR_FLAG_LOCK           (1 << 1) // 1: Vergrendeld, 0: Vrij

// opcodes voor busverkeer
enum BikeWireOpcode {
    WIRE_OP_HEARTBEAT         = 0x01,
    WIRE_OP_EMERGENCY_STOP    = 0x02, // Noodstop activeren
    WIRE_OP_RESET_EMERGENCY   = 0x03, // Noodstop softwarematig resetten
    WIRE_OP_SET_MODE          = 0x04, // Systeemmodus (bv. Standby, Autonoom, Manuele overname)
    WIRE_OP_SET_MOTION        = 0x10,
    WIRE_OP_SET_BRAKE         = 0x11,
    WIRE_OP_SET_ACTUATORS     = 0x12,
    WIRE_OP_SET_BAL_OFFSET    = 0x13,
    
    WIRE_OP_TELEMETRY_FAST    = 0x80,
    WIRE_OP_TELEMETRY_SLOW    = 0x81
};

#pragma pack(push, 1)

// algemene header
struct WireHeader {
    uint8_t magic1;         // 0xAA
    uint8_t magic2;         // 0x55
    uint8_t opcode;
    uint16_t seq_id;        // volgnummer
    uint8_t payload_len;    // lengte van data
};

struct WireSetModeCmd {
    uint8_t system_status;  // 0: Standby, 1: Autonoom, 2: Manueel (controller), etc.
};

// floats omgezet naar 16-bit ints: bespaart bandbreedte
struct WireMotionCmd {
    int16_t target_speed_mms;           // mm/s
    int16_t target_accel_mms2;          // mm/s^2 (acceleratiebegrenzing)
    int16_t target_steering_centi_deg;  // 1/100 graden (+/-)
    uint16_t duration_rotations_centi;  // 1/100 rotaties
};

struct WireBrakeCmd {
    uint8_t brake_force_pct;            // 0 (los) tot 100 (volledig remmen)
};

struct WireBalanceOffsetCmd {
    int16_t balance_offset_centi_deg;   // 1/100 graden gewenste kantelhoek offset (+/-)
};

struct WireActuatorsCmd {
    uint8_t lights_mode;                // 0: Uit, 1: Aan, 2: Auto (lichtsensor)
    uint8_t indicator_mode;             // 0: Uit, 1: Links, 2: Rechts, 3: Alarm (hazard)
    uint8_t kickstand_deploy;           // 0: Ingeklapt, 1: Uitgeklapt (parkeren)
    uint8_t lock_engaged;               // 0: Vrij, 1: Op slot
    uint16_t sound_duration_ms;         // Duur geluid in ms (0 = geen geluid / stop)
};

struct WireIMUData {
    int16_t accel_x_mg;                 // milli-g (+/- 16G bereik)
    int16_t accel_y_mg;
    int16_t accel_z_mg;
    int16_t gyro_x_dps_centi;           // 1/100 graden/sec
    int16_t gyro_y_dps_centi;
    int16_t gyro_z_dps_centi;
    int16_t roll_centi_deg;             // 1/100 graden
    int16_t pitch_centi_deg;
    int16_t yaw_centi_deg;
};

// Snelle telemetrie (100-200 Hz): rijsnelheid, stuurhoek, 2x 9-DoF IMU, status & flags
struct WireFastTelemetry {
    uint32_t timestamp_ms;
    int16_t current_speed_mms;
    int16_t current_steering_centi_deg;
    
    WireIMUData frame_imu;              // Hoofd-IMU gemonteerd op het frame
    WireIMUData handlebar_imu;          // Secundaire IMU gemonteerd op het stuur
    
    uint8_t safety_flags;               // Zie BIKE_SAFETY_FLAG_* (voor ICT)
    uint8_t actuator_feedback;          // Zie BIKE_ACTUATOR_FLAG_* (staander, slot)
};

// Trage telemetrie (1-5 Hz): batterij, omgevingswaarden, bandenspanning, luchtkwaliteit
struct WireSlowTelemetry {
    uint16_t battery_voltage_mv;
    int16_t battery_current_ma;
    uint8_t battery_soc_pct;
    uint16_t light_lux;
    
    int16_t ambient_temp_centi_c;       // 1/100 graden Celsius (-1000 = onbekend/fout)
    uint16_t ambient_humidity_centi_pct;// 1/100 % (0-10000)
    uint16_t tire_pressure_front_mbar;  // mbar
    uint16_t tire_pressure_rear_mbar;   // mbar
    uint16_t air_quality_index;         // AQI score (0xFFFF indien niet aanwezig)
};

#pragma pack(pop)