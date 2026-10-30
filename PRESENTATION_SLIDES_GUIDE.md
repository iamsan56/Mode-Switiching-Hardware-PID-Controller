# PS-02 Hardware PID Controller — 10-Minute Presentation Guide
## Slide-by-Slide Deck Structure & Speaker Script (Deliverable 5)
### VLSI Hackathon 2026 — VIT Chennai · Team 8: Palanivel Rajan S & Sanjeev D

---

## Slide 1: Title & Overview (Time: 0:00 – 1:00)
- **Title:** Mode-Switching Hardware PID Controller (PS-02)
- **Subtitle:** Autonomous 45nm ASIC Flight Controller with Predictive Bumpless Transfer
- **Presenters:** Palanivel Rajan S (24BEC1005) & Sanjeev D (24BEC1053) · Team 8
- **Problem Setter:** Sudarshana Semiconductors
- **Speaker (Palanivel):**
  > *"Good morning respected judges and mentors. Today, we present our complete RTL-to-GDSII tapeout of PS-02: a hardware-only, fixed-point PID flight controller for multi-rotor drones. Rather than presenting a high-level simulation, Team 8 has carried this design through every stage of the ASIC design flow: architectural specification, Verilog RTL, closed-loop plant co-simulation, logic synthesis in Cadence Genus, and complete physical implementation and sign-off in Cadence Innovus down to a verified 104 MB GDSII."*

---

## Slide 2: Problem Statement & Key Challenges (Time: 1:00 – 2:00)
- **Four Core Requirements:**
  1. Multi-Axis Cascaded Loops: Roll, Pitch, Yaw parallel attitude/rate control + Altitude channel.
  2. Four Flight Regimes: Acro (rate only), Stabilize (attitude + rate), AltHold, and Failsafe.
  3. Bumpless Mode Switching: Zero transient actuator jump across mode boundaries.
  4. Robustness & Safety: Derivative kick elimination, anti-windup clamping, sensor dropout watchdog, and controlled failsafe descent.
- **Speaker (Sanjeev):**
  > *"The foundational challenge of PS-02 is that flight controllers operate under non-zero rotational dynamics. When switching modes from Acro to Stabilize, differing proportional and integral gains cause severe torque discontinuities, which in flight can flip a drone. Our mission was to guarantee zero output jump by construction, with cycle-deterministic latency and complete silicon sign-off."*

---

## Slide 3: Top-Level Architecture & Multi-Rate Schedulers (Time: 2:00 – 3:00)
- **Key Visual:** System Block Diagram (`system_block_diagram.png`).
- **Core Elements:**
  - 50 MHz Master Clock with hardware divider counters.
  - 1 kHz Inner Rate Loop (50,000 cycles) & 100 Hz Outer Attitude Loop (500,000 cycles).
  - 3 Parallel Cascaded Axis Channels + Dedicated Altitude Channel.
  - Memory-Mapped Host Register Interface (32 registers/axis).
- **Speaker (Palanivel):**
  > *"Here is our top-level architecture. We evaluated a single time-shared datapath versus three parallel instances. We chose three parallel instances because it completely eliminates inter-axis timing skew, phase lag, and context-switching overhead during maneuvers. Our hardware dividers provide exact 1 kHz and 100 Hz strobes from our 50 MHz clock, ensuring jitter-free execution with zero external timer dependencies."*

---

## Slide 4: Arithmetic Datapath & 4-Cycle Pipeline (Time: 3:00 – 4:00)
- **Fixed-Point Formats:**
  - Q15 (1.15 signed) for Setpoints, Sensors, and Actuators (Range: [-1.0, +1.0], Resolution: 30.5 µV).
  - Q8.8 signed for PID Gains (Range: [-128.0, +127.996]).
  - 32-bit Wide Signed Integrator with Dynamic Clamping.
- **4-Cycle Pipeline Latency:**
  - Cycle 0 (`S_IDLE`): Sample error & raw measurement delta.
  - Cycle 1 (`S_FILTER`): 1st-order IIR derivative low-pass filter with symmetric rounding bias (`+0x4000`).
  - Cycle 2 (`S_MULT`): Parallel Q15 × Q8.8 saturating multiplication.
  - Cycle 3 (`S_SUM`): Anti-windup clamped accumulation and output saturation.
- **Speaker (Sanjeev):**
  > *"Every calculation in our datapath is strictly deterministic. Our PID core runs on a 4-state pipeline FSM with an invariant latency of exactly 4 clock cycles—80 nanoseconds at 50 MHz. Saturating arithmetic primitives (`sat_add16`, `sat_mul_gain`, `sat_clamp`) ensure that two's-complement overflow wrap-around is physically impossible."*

---

## Slide 5: The Bumpless Transfer Breakthrough (Time: 4:00 – 5:00)
- **Key Visual:** Mode-Switching Transient Waveform (`bumpless_transfer_plot.png`).
- **The Mathematical Solution:**
  $$I_{\text{target}} = u_{\text{prev}} - P_{\text{target}} - I_{\text{step}} + D_{\text{target}}$$
  $$u_{\text{new}} = P_{\text{target}} + (I_{\text{target}} + I_{\text{step}}) - D_{\text{target}} = u_{\text{prev}} \implies \mathbf{\Delta u = 0\text{ LSB}}$$
- **3-State Transition FSM:** `S_STEADY` $\to$ `S_CALC` $\to$ `S_APPLY`.
- **Speaker (Palanivel):**
  > *"This is our standout technical achievement. Standard bumpless algorithms only adjust the proportional term. Under dynamic rotation, the derivative and upcoming integration step are non-zero, causing a 50–150 LSB jump. We derived a closed-form predictive back-calculation that pre-seeds the integrator register with $I_{\text{target}} = u_{\text{prev}} - P - I_{\text{step}} + D$. On the exact switch cycle, the new mode evaluates to the exact previous output. Across all 8 directed mode pairs, we proved a 0-LSB transient jump in simulation."*

---

## Slide 6: Safety Architecture: Watchdog & Failsafe Ramp (Time: 5:00 – 6:00)
- **Hardware Watchdog:** 32-bit countdown timer (default: 250,000 cycles = 5 ms at 50 MHz).
- **Stale Data Detector:** Resets on `imu_valid`. If no sample arrives in 5 ms, asserts `watchdog_tripped`.
- **Autonomous Recovery:** Supervisor forces mode `2'b11` (Failsafe) without MCU intervention.
- **Linear Descent Ramp (`pid_failsafe_ramp.v`):** Roll/Pitch leveled to 0°; thrust linearly decrements by `0x0200` per sample down to `0x0800` (safe hover-minus).
- **Speaker (Sanjeev):**
  > *"For aerial systems, robustness and safety carry 10% of our score. If an IMU cable disconnects or freezes, our 32-bit watchdog trips in 5 milliseconds. But rather than cutting motor power, which would cause an uncontrollable crash, our autonomous ramp module levels the wings to zero and smoothly ramps thrust down at a controlled slope to hover-minus speed, executing a safe, controlled emergency descent."*

---

## Slide 7: Verification Suite & Golden Model Co-Simulation (Time: 6:00 – 7:00)
- **Key Visuals:** NCLaunch 20/20 PASS terminal log & Closed-loop co-sim plot.
- **Top-Level Regression:** 20/20 self-checking test cases passing in Cadence NCLaunch / ncsim.
- **Closed-Loop Co-Simulation Results:**
  - Continuous-time quadrotor rotational plant ($J\ddot{\theta} = \tau - b\dot{\theta}$).
  - Hardware Fidelity Match: **97.38%** against double-precision Golden PID.
  - Mean Tracking Discrepancy: **0.5243°**.
  - Derivative Spike on Setpoint Step: **0.0000°/s (Zero kick)**.
- **Speaker (Palanivel):**
  > *"We verified our design at both the algorithmic and gate levels. In Cadence NCLaunch, our automated regression suite ran 20 directed test cases covering reset, multi-axis tracking, all mode switches, watchdog dropout, and saturation—all 20 passed. In Python, closed-loop co-simulation against a dynamic quadrotor plant showed a 97.38% match with the floating-point golden model, with zero derivative spike on setpoint steps."*

---

## Slide 8: Cadence Genus 21.14 Logic Synthesis (Time: 7:00 – 8:00)
- **Key Visual:** Genus QoR terminal summary & power report screenshots.
- **Synthesis Results (45nm `typical.lib`, 1.2V, 25°C):**
  - Standard Cells: **79,754 leaf instances** (3,107 Flip-Flops, 76,647 Combinational).
  - Timing Slack: **+11,552.6 ps (+11.55 ns setup slack at 50 MHz)**.
  - Zero Violating Paths (TNS = 0.0 ps).
  - Total Power: **38.0 mW** (Leakage: 0.03 mW, Internal: 26.5 mW, Switching: 11.5 mW).
- **Speaker (Sanjeev):**
  > *"In Cadence Genus, we synthesized the complete 3-axis architecture to 79,754 standard cells. At our 50 MHz clock constraint, the critical path achieved +11.55 nanoseconds of positive slack, demonstrating that our logic can easily scale to 118 MHz. Total power consumption is a frugal 38 milliwatts."*

---

## Slide 9: Cadence Innovus 21.15 Physical Design & Sign-Off (Time: 8:00 – 9:00)
- **Key Visual:** Innovus full chip layout (Fig 10) & Post-Route timing/DRC screenshots.
- **Physical Sign-Off Metrics:**
  - Die Size: **812.40 µm × 809.78 µm** (Core Area: 0.658 mm², Density: **42.103%**).
  - Placed Instances: **79,933 physical cells** (0 unplaced).
  - Perimeter Pins: **373 boundary I/O pins** legally placed across 4 edges on M3/M4.
  - Routing: **85,321 nets** (100% routed, 1.58 m wire), **632,959 vias** (**96.0% multi-cut**).
  - DRC (`verifyGeometry`): **0 violations across all 49 sub-areas**.
  - LVS (`verifyConnectivity`): **0 violations / 0 warnings**.
  - Post-Route Timing (OCV + SI): Setup WNS **+5.913 ns** (F_max ≈ 71 MHz), Hold WNS **+0.107 ns**.
  - Tapeout Output: **104 MB GDSII stream** (`pid_controller.gds`).
- **Speaker (Palanivel):**
  > *"In Cadence Innovus, we solved the 373-pin congestion challenge by distributing I/O pins across all four chip edges on alternating metal layers. NanoRoute completed 100% routing with 96% multi-cut vias for high fabrication yield. Post-route timing closed under full On-Chip Variation and crosstalk analysis with +5.91 ns setup margin and +0.107 ns hold margin. Physical sign-off confirmed 0 DRCs and 0 LVS errors, streaming out the 104 MB tapeout GDSII."*

---

## Slide 10: Conclusion & Deliverables Summary (Time: 9:00 – 10:00)
- **Summary Matrix:**
  - Functional Correctness (30%): All PS-02 requirements met, 0-LSB bumpless switch, 97.38% golden match.
  - Verification Quality (20%): 20/20 test cases, closed-loop co-sim, fault injection, saturation stress.
  - Design Quality PPA (20%): 79,933 cells, 42.1% utilization, 38 mW, +5.91 ns slack, 71 MHz.
  - Robustness & Safety (10%): 32-bit watchdog, linear descent ramp, saturating math, synchronous clocking.
  - Documentation & Demo (20%): Complete design document, test plan, README, and live demo ready.
  - Integration Bonus (+5%): Complete 3-Axis + Alt single-chip integration.
- **Speaker (Palanivel & Sanjeev):**
  > *"To conclude, Team 8 has delivered a fully verified, timing-closed, DRC/LVS-clean 45nm ASIC flight controller that answers every requirement of PS-02 with proven margins. We are now ready to demonstrate our live simulation and show our design in SimVision and Innovus. Thank you, and we welcome your questions!"*
