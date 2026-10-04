# Real-Time Industrial Car Wash Controller (80C552 / 8051 Assembly)

An embedded real-time industrial automation system designed to control a multi-stage automated car wash tunnel. Developed in Assembly for the Philips/NXP 80C552 microcontroller (8051 core architecture) and simulated using Keil µVision.

---

## 🛠 Hardware Architecture & Peripherals

* **Microcontroller Target:** Philips/NXP 80C552 (Intel 8051 derivative) @ 12 MHz oscillator.
* **Core Language:** 8051 Assembly Language (ASM).
* **Toolchain & Simulation:** Keil µVision2 IDE.
* **Core Peripherals Used:**
  * **Timer 0 (16-bit Mode):** Time-base generation via periodic hardware interrupts.
  * **ADC (Analog-to-Digital Converter):** 8-bit resolution input tracking distance sensors (vertical and horizontal rollers).
  * **PWM Generators (PWM0 & PWM1):** Frequency-matched actuator power management for roller agitation and air dryer blowers.
  * **Parallel I/O Ports (P0–P2):** Interfacing optical vehicle presence sensors, mechanical limit switches, chemical dispensers, and signalization LEDs.

---

## ⚙ System Architecture: 21-State Finite State Machine (FSM)

The control logic is governed by a non-blocking Finite State Machine (FSM) structured around 21 discrete states (E0 to E20), ensuring deterministic real-time response:

    [E0: Idle / Standby] 
       └──> [E1–E2: Coin Validation & User Program Selection]
       └──> [E3–E5: Vehicle Ingestion, Pre-wash Water & Chemical Soap Deployment]
       └──> [E6–E13: Reactive Roller Positioning & Scrubbing Cycle]
       └──> [E14–E16: High-Pressure Rinsing & Distance Re-alignment]
       └──> [E17–E18: Automated Air Blower & Drying Subsystem]
       └──> [E19–E20: Bridge Homing, Green Beacon Cycle & Platform Reset]

* State dispatch is implemented through memory pointer table lookups (DPTR offset indexing via `JMP @A+DPTR`), eliminating deep branching overhead and guaranteeing fast interrupt-to-action transition times.

---

## 📐 Hardware Calculations & Engineering Specifications

### 1. Timer 0 Periodic Interrupt (50 ms Time-Base)
Configured to produce predictable software ticks for system timers (100 ms, 1 s, 4 s, 30 s, and 60 s):
* F_osc = 12 MHz => T_machine = 12 / 12 MHz = 1 µs
* Counts required for 50 ms: 50,000 counts.
* 16-bit reload value: 65,536 - 50,000 = 15,536 = 0x3CB0
* **Reload Registers:** `TH0 = 0x3C`, `TL0 = 0xB0`.

### 2. ADC Calibration & Sensor Digital Mapping
Linear ultrasonic/optical distance sensors (50 mV/cm sensitivity, 0–5 V range, 8-bit quantization):
Digital Value = 2^8 * (V_in - V_ref-) / (V_ref+ - V_ref-)

* **30 cm (1.5 V):** Value = 77 => **0x4D**
* **40 cm (2.0 V, Target Operating Distance):** Value = 102 => **0x66**
* **50 cm (2.5 V):** Value = 128 => **0x80**

### 3. PWM Actuator Modulation
* Carrier frequency: F_PWM = 1 kHz.
* Reload register: PWMP = (12 MHz / (2 * 255 * 1 kHz)) - 1 ≈ 24 = **0x18**.
* **Motor Scrubbing (PWM0):** 50% duty cycle (0x80) / 100% duty cycle (0x00).
* **Dryer Blowers (PWM1):** Configurable up to 85% duty cycle (0x26).

---

## 📂 Repository Layout

    ├── src/
    │   └── carwash_controller.asm   # Full 80C552 Assembly source code
    ├── docs/
    │   ├── State_Machine_Diagram.png # State transition model visualization
    │   └── 80C552_System_Report.pdf  # Comprehensive academic engineering documentation
    └── README.md

---

## 👤 Academic Context

Developed as the Final Laboratory Capstone Project for **Computer Architecture (Konputagailuen Arkitektura)** within the Management Information Systems Computer Engineering curriculum at the **School of Engineering of Bilbao (UPV/EHU)**. 
