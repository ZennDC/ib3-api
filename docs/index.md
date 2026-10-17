# Bike Hardware API

> **Hardware Interface & Communication Protocol Documentation**  
> _KU Leuven — Bachelor in Industrial Engineering: Electronics & ICT (IB3)_

Welcome to the API reference for the **IB3 Autonomous Bicycle**. This documentation serves as the formal contract between the high-level **ICT software stack** and the low-level **Electronics hardware layer**.

---

## Header Catalog

Public C/C++ interface declarations reside in the [`include/`](https://github.com/ZennDC/ib3-api/tree/master/include) directory:

| Header                                                   | Scope          | Description                                                                                                                                                                                                                                                                                                                                              |
| :------------------------------------------------------- | :------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **[`include/interface.hpp`](bikeApi/interface_8hpp.md)** | High-Level HAL | Pure abstract C++ interface ([`BikeHardware::Interface`](bikeApi/classBikeHardware_1_1Interface.md)) covering motion, analog braking, stabilization, actuators, modes, and telemetry.                                                                                                                                                                    |
| **[`include/types.hpp`](bikeApi/types_8hpp.md)**         | Data Models    | Domain structs ([`BikeTelemetry`](bikeApi/structBikeHardware_1_1BikeTelemetry.md), [`MotionCommand`](bikeApi/structBikeHardware_1_1MotionCommand.md), RTK [`GPS`](bikeApi/structBikeHardware_1_1GPSData.md), [`IMUData`](bikeApi/structBikeHardware_1_1IMUData.md)), enums, and [`SafetyFlag`](bikeApi/namespaceBikeHardware_1_1SafetyFlag.md) bitmasks. |
| **[`include/protocol.h`](bikeApi/protocol_8h.md)**       | Low-Level Wire | Binary SPI wire-protocol (packed C structs, opcodes, sequence framing, fast 100–200 Hz telemetry, and slow 1–5 Hz diagnostics).                                                                                                                                                                                                                          |

---

## API Quick Navigation

| Section             | Destination                               | Details                                                                      |
| :------------------ | :---------------------------------------- | :--------------------------------------------------------------------------- |
| **Class Index**     | [Classes & Structs](bikeApi/annotated.md) | Definitions, data structs, and driver interfaces across all C/C++ components |
| **Header Catalog**  | [Header Files](bikeApi/files.md)          | Doxygen file summaries and declarations inside `include/`                    |
| **Class Hierarchy** | [Class Hierarchy](bikeApi/hierarchy.md)   | Peripheral base classes and inheritance trees                                |
| **Namespaces**      | [Namespaces](bikeApi/namespaces.md)       | Module partitions for `BikeHardware` and `SafetyFlag`                        |

---

## Resources

- **Project Report**: [Download Meeting Report (PDF)](assets/Report%20Vergadering%20IB3%20project.pdf)
- **Source Repository**: [GitHub — ZennDC/ib3-api](https://github.com/ZennDC/ib3-api) — Source code
