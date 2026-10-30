# PS-02 Hardware PID Controller — Live Demonstration Runbook
## Step-by-Step Judge Demonstration Cheat Sheet (Deliverable 5)
### VLSI Hackathon 2026 · Team 8: Palanivel Rajan S & Sanjeev D

This document gives you the exact terminal commands and talking points to execute a flawless live demonstration in front of the judging panel in under 5 minutes.

---

## Part 1: Live Functional Regression & Self-Checking Proof (1 Minute)

### Terminal Command:
Open a terminal on the lab machine (`ic312.vit.ac.in`) and run:
```csh
cd /home/vlsi/Desktop/bs-2
source /home/install/cshrc
ncverilog +access+rwc rtl/*.v pid_controller.v tb/tb_pid_top.v
```

### What to show the Judges on Screen:
The simulation runs and displays the self-checking table in under 5 seconds:
```
=============================================================
  Results: 20 PASSED,  0 FAILED  (of 20 total checks)
  >>> ALL TOP-LEVEL REGRESSION TESTS PASSED <<<
=============================================================
Simulation complete via $finish(1) at time 8641 NS + 0
```
### Key Speaking Point:
> *"Judges, we have built a fully automated, self-checking regression testbench covering 20 distinct verification scenarios. As you can see, 20 out of 20 tests pass with zero assertions failing, verifying reset, multi-axis parallel maneuvering, bumpless mode transitions, sensor dropout, and live register tuning."*

---

## Part 2: Live SimVision Waveform Inspection (2 Minutes)

### Terminal Command:
To launch SimVision with waveforms pre-loaded:
```csh
simvision waves.shm &
```

### 3 Key Waveforms to Point Out to the Judges:
1. **Waveform 1 — 0-LSB Bumpless Mode Switch:**
   - Zoom in on `pilot_mode` changing from `00` (Acro) to `01` (Stabilize).
   - Point to `actuator_roll` and `actuator_pitch`.
   - **Show:** The actuator command does not experience a single LSB jump ($\Delta u = 0$). The output continues completely uninterrupted.
   - **Explain:** *"Notice our predictive back-calculation in `pid_mode_fsm.v`. On the transition edge, the integrator register is pre-seeded so that the upcoming proportional and integral steps sum to exactly the previous output."*

2. **Waveform 2 — Master Sensor Dropout & Synchronized Failsafe Ramp:**
   - Look at the time window where `imu_valid` strobes stop.
   - Point to `watchdog_timeout` counting down.
   - Show `watchdog_tripped` asserting and `active_mode` switching autonomously to `2'b11` (Failsafe).
   - Point to `actuator_thrust`: show it ramping down smoothly at a constant slope of `0x0200` per sample down to `0x0800` (safe hover-minus target).
   - **Explain:** *"This demonstrates autonomous vehicle recovery. We do not kill the motors instantly, which would cause the drone to tumble. Our dedicated hardware ramp module ensures a controlled, level descent."*

3. **Waveform 3 — Derivative Kick Suppression:**
   - Show a sudden step change in pilot stick input.
   - Show that `d_filt` does NOT spike, because the derivative is computed exclusively on the measurement signal, not the error.

---

## Part 3: Python Golden Model Closed-Loop Plant Co-Simulation (1 Minute)

### Terminal Command:
```bash
python ref_model/pid_reference.py
```

### What to show the Judges:
The script executes in 1 second and prints:
```
======================================================================
  PS-02 CLOSED-LOOP PLANT CO-SIMULATION REPORT                        
======================================================================
  Step Amplitude            : 20.0 degrees
  Rise Time (10% to 90%)     : 99.0 ms
  Settling Time (2% band)   : 325.0 ms
  Mean Tracking Discrepancy : 0.5243 degrees (Golden vs RTL)
  Hardware Fidelity Match   : 97.38 %
======================================================================
```
### Key Speaking Point:
> *"To prove algorithmic correctness against a real physical plant, we co-simulated our bit-true Q1.15 fixed-point RTL model against a quadrotor rotational-dynamics plant ($J\ddot{\theta} = \tau - b\dot{\theta}$) alongside a double-precision floating-point Golden PID model. Our fixed-point hardware matches the golden model with a 97.38% fidelity match and a mean tracking error of only 0.52 degrees."*

---

## Part 4: Physical Design & 45nm Silicon Tapeout (1 Minute)

### Terminal Command:
```csh
cd /home/vlsi/Desktop/bs-2/pnr
innovus -init outputs/pid_controller_final.enc
```

### What to show the Judges in the Innovus GUI:
1. **Full Chip Layout:**
   - Press `F` to fit the design.
   - Show the square die: **812.40 µm × 809.78 µm** (Core density: **42.103%**).
   - Show all **373 boundary I/O pins** placed evenly across all four edges on Metal 3 and Metal 4.
2. **Power Grid:**
   - Turn on Power visibility (`VDD`, `VSS`).
   - Show the dual M9/M10 core rings and dense M1 standard cell rails.
3. **DRC / LVS & Timing Sign-Off Reports:**
   - In terminal, display the final sign-off reports:
     ```csh
     cat reports/geometry_final.rpt
     # Displays: Cells: 0, Wiring: 0, Short: 0 -> No DRC violations were found
     
     cat reports/connectivity_final.rpt
     # Displays: Found no problems or warnings -> 0 LVS violations
     
     zcat reports/postRoute_setup.summary.gz | head -n 20
     # Displays: Setup WNS: +5.913 ns, Violating Paths: 0
     
     zcat reports/postRoute_hold_hold.summary.gz | head -n 20
     # Displays: Hold WNS: +0.107 ns, Violating Paths: 0
     ```

### Final Punchline for Judges:
> *"Judges, we have not stopped at RTL or synthesis. We have carried PS-02 to a full 45nm ASIC sign-off with 0 DRCs, 0 LVS errors, +5.91 ns of setup margin under full on-chip variation and crosstalk, and streamed out the final 104 MB tapeout GDSII (`pid_controller.gds`). This design is ready for silicon fabrication."*
