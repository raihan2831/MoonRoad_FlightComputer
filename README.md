# MoonRoad – F-Class Thrust Vector Control (TVC) Rocket

## 🚀 Overview
**MoonRoad** is an ambitious student-led rocketry project focused on developing an F-class Thrust Vector Control (TVC) rocket. Grounded in the core mission of **VITAL**, the project aims to pioneer reusable vertical take-off and landing (VTVL) rocket systems. 

Through iterative flight testing, precise mathematical modeling, and advanced navigation control, MoonRoad pushes the boundaries of student-engineered aerospace technology.

---

## 🌍 Mission & Vision

### 1. Technical Focus
* **Vertical Take-off & Landing (VTVL):** Designing systems capable of multiple take-offs and landings using rocket motors to refine flight hardware and control loops.
* **Thrust Vector Control & Navigation:** Developing robust dynamic models, state-space equations, and cascade PID control algorithms to manage real-time stabilization and trajectory tracking.
* **Modular Avionics Architecture:** Integrating custom-built Power Distribution Boards (PDBs), high-precision flight computers, high-voltage servos (BMS-A10H), and onboard sensor suites (IMU, barometer, LoRa telemetry).

### 2. Social & Environmental Focus (Sustainability)
* **Reusability over Disposable Hardware:** Challenging the traditional single-use paradigm of model and amateur rocketry to promote sustainable engineering practices.
* **Space Debris Mitigation:** Addressing the critical global challenge of orbital debris and dead satellites that endanger future human spaceflight and near-Earth operations.
* **Interplanetary Readiness:** Aligning with the sustainable technologies required for future long-duration exploration and landing missions on Mars.

---

## 📂 Repository Structure

- **`Power Delivery Board/`** – KiCad schematics, PCB layouts, and component BOMs for the custom power distribution and management system.
- **`Flight Computer PCB/`** – KiCad project files and gerbers for the main flight computer and sensor integration board.
- **`Firmware/`** – Embedded C/C++ firmware (PlatformIO) running on microcontrollers for sensor fusion, telemetry, and actuator control.
- **`Simulation Control/`** – MATLAB and Simulink models implementing state-space representations and cascade PID tuning for the TVC testbench.
- **`telemetry Analysis/`** – Python scripts (NumPy, Matplotlib) for post-flight data processing, sensor logging analysis, and trajectory evaluation.

---

## 🤝 Contributing & Teamwork
MoonRoad brings together diverse engineering disciplines—spanning embedded systems, PCB design, control theory, and mechanical packaging. Whether you are working on hardware integration or flight code, every subsystem is built with flight-readiness and reusability in mind.
