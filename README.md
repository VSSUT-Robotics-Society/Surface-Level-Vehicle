# Surface Level Vehicle (SLV)

An autonomous surface water robot designed for environmental monitoring, research, and rescue operations.

---

## Table of Contents
1. [Overview](#overview)
2. [Our Goals](#our-goals)
3. [What We Currently Use](#what-we-currently-use)
    - [Hardware](#hardware)
    - [Software](#software)
    - [Packages](#packages)
4. [How to Run the Project](#how-to-run-the-project)
    - [Installation](#installation)
5. [Problems Currently Being Faced](#problems-currently-being-faced)
6. [Bill of Materials Required](#bill-of-materials-required)
7. [Future Applications/Features](#future-applicationsfeatures)
8. [References](#references)

---

## Overview

The Surface Level Vehicle (SLV) is an unmanned watercraft designed for surface-level operations. It incorporates a lightweight, durable aluminum extrusion frame for structural support, enhancing stability, modularity, and ease of assembly. Powered by four BLDC motors and equipped with GPS for autonomous navigation, the SLV is built for efficient and precise aquatic missions.

---

## Our Goals

- Develop a cost-effective and modular surface vehicle.
- Enable autonomous water navigation using GPS.
- Support various environmental, research, and safety applications.
- Ensure stability, durability, and ease of customization.

---

## What We Currently Use

### Hardware

- Aluminum extrusion frame  
- 4 × High-RPM BLDC motors with waterproof propellers  
- 2 × Air-filled PVC pipes (floatation)  
- GPS module  
- Rechargeable battery pack  
- Waterproof enclosures

### Software

- Arduino IDE (for motor and GPS control)  
- Mission planning software (e.g., QGroundControl or custom path planner)

### Packages

- Easy navigation and motor control libraries (to be listed if available)

---

## How to Run the Project

### Installation

1. Connect the GPS and motor controllers to the microcontroller.
2. Upload the control code using Arduino IDE.
3. Ensure all connections are waterproof and secured.
4. Calibrate motors and GPS before deployment.

---

## Problems Currently Being Faced

- Fine-tuning GPS accuracy for precise waypoint navigation.
- Ensuring reliable waterproofing in high humidity or splash zones.
- Battery life optimization during long missions.

---

## Bill of Materials Required

| Item                     | Quantity | Description                       |
|--------------------------|----------|-----------------------------------|
| Aluminum extrusions      |   xN     | Main structural frame             |
| BLDC Motors (Waterproof) |    4     | Propulsion                        |
| Propellers               |    4     | Attached to motors                |
| PVC Pipes (Floatation)   |    2     | Buoyancy and stability            |
| GPS Module               |    1     | Navigation                        |
| Rechargeable Battery     |    1     | Power source                      |
| Waterproof Enclosure     |    N     | Electronics protection            |

---

## Future Applications/Features

- Solar power integration using the flat frame surface  
- Mounting of advanced sensors (LIDAR, sonar, turbidity, etc.)  
- Hydrodynamic optimization of float structures  
- Modular add-ons for different missions (sample collectors, probes)

---

## References

- Internal project documentation  
- Arduino & sensor datasheets  
- Research on autonomous surface vehicles