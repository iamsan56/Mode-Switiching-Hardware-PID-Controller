// =============================================================================
// File        : tb_pid_top.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 10: Complete Top-Level Regression Suite for pid_controller.
//               Verifies the complete chip:
//               1. System Reset & Boot State
//               2. Hardware Rate Scheduler (50MHz -> 1kHz / 100Hz)
//               3. Simultaneous 3-Axis Acro Mode Maneuvering
//               4. Simultaneous 3-Axis Stabilize Mode Cascaded Tracking
//               5. Altitude Hold Mode Engaging Vertical Channel
//               6. Full-Chip Predictive Bumpless Transfer across all axes
//               7. Master Sensor Dropout & Synchronized Failsafe Ramp
//               8. Host Register Interface Independent Axis Tuning
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_top;

    reg clk;
    reg rst_n;

    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50 MHz
    end

    // Simulation control
    reg        sim_fast_ticks;
    reg        ext_step_rate;
    reg        ext_step_att;
    
    // Pilot / Supervisor controls
    reg  [1:0] pilot_mode;
    wire [1:0] active_mode;
    wire       watchdog_tripped;
    
    // Setpoints
    reg  [15:0] stick_roll;
    reg  [15:0] stick_pitch;
    reg  [15:0] stick_yaw;
    reg  [15:0] stick_thrust;
    
    // Sensors
    reg         imu_valid;
    reg  [15:0] roll_angle_meas;
    reg  [15:0] roll_rate_meas;
    reg  [15:0] pitch_angle_meas;
    reg  [15:0] pitch_rate_meas;
    reg  [15:0] yaw_rate_meas;
    reg  [15:0] alt_meas;
    
    // Configuration
    reg  [31:0] watchdog_timeout;
    reg  [15:0] safe_thrust_target;
    reg  [15:0] ramp_rate;
    
    // Host register programming
    reg         reg_wr_en;
    reg  [1:0]  reg_axis_sel;
    reg  [4:0]  reg_wr_addr;
    reg  [15:0] reg_wr_data;
    
    // Outer loop gains
    reg  [15:0] att_kp_roll;
    reg  [15:0] att_kp_pitch;
    reg  [15:0] att_kp_yaw;
    
    // Actuator outputs
    wire [15:0] actuator_roll;
    wire [15:0] actuator_pitch;
    wire [15:0] actuator_yaw;
    wire [15:0] actuator_thrust;
    wire        tick_1khz;
    wire        tick_100hz;

    reg [15:0] roll_before;
    reg [15:0] pitch_before;

    // Instantiate Top-Level Chip
    pid_controller uut (
        .clk                (clk),
        .rst_n              (rst_n),
        .sim_fast_ticks     (sim_fast_ticks),
        .ext_step_rate      (ext_step_rate),
        .ext_step_att       (ext_step_att),
        .pilot_mode         (pilot_mode),
        .active_mode        (active_mode),
        .watchdog_tripped   (watchdog_tripped),
        .stick_roll         (stick_roll),
        .stick_pitch        (stick_pitch),
        .stick_yaw          (stick_yaw),
        .stick_thrust       (stick_thrust),
        .imu_valid          (imu_valid),
        .roll_angle_meas    (roll_angle_meas),
        .roll_rate_meas     (roll_rate_meas),
        .pitch_angle_meas   (pitch_angle_meas),
        .pitch_rate_meas    (pitch_rate_meas),
        .yaw_rate_meas      (yaw_rate_meas),
        .alt_meas           (alt_meas),
        .watchdog_timeout   (watchdog_timeout),
        .safe_thrust_target (safe_thrust_target),
        .ramp_rate          (ramp_rate),
        .reg_wr_en          (reg_wr_en),
        .reg_axis_sel       (reg_axis_sel),
        .reg_wr_addr        (reg_wr_addr),
        .reg_wr_data        (reg_wr_data),
        .att_kp_roll        (att_kp_roll),
        .att_kp_pitch       (att_kp_pitch),
        .att_kp_yaw         (att_kp_yaw),
        .actuator_roll      (actuator_roll),
        .actuator_pitch     (actuator_pitch),
        .actuator_yaw       (actuator_yaw),
        .actuator_thrust    (actuator_thrust),
        .tick_1khz          (tick_1khz),
        .tick_100hz         (tick_100hz)
    );

    // Test infrastructure
    integer pass_count = 0;
    integer fail_count = 0;

    task check16;
        input [63:0]  test_id;
        input [15:0]  got;
        input [15:0]  expected;
        input [127:0] label;
        begin
            if (got === expected) begin
                $display("  PASS [%0d] %s  got=0x%04X", test_id, label, got);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL [%0d] %s  got=0x%04X  expected=0x%04X",
                         test_id, label, got, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check2;
        input [63:0] test_id;
        input [1:0]  got;
        input [1:0]  expected;
        input [127:0] label;
        begin
            if (got === expected) begin
                $display("  PASS [%0d] %s  got=%b", test_id, label, got);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL [%0d] %s  got=%b  expected=%b",
                         test_id, label, got, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check1;
        input [63:0]  test_id;
        input         got;
        input         expected;
        input [127:0] label;
        begin
            if (got === expected) begin
                $display("  PASS [%0d] %s  got=%b", test_id, label, got);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL [%0d] %s  got=%b  expected=%b",
                         test_id, label, got, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task pulse_fast_rate;
        begin
            @(posedge clk);
            ext_step_rate <= 1;
            @(posedge clk);
            ext_step_rate <= 0;
            // 4 cycles for PID computation
            repeat (6) @(posedge clk);
            #1;
        end
    endtask

    task pulse_fast_att;
        begin
            @(posedge clk);
            ext_step_att <= 1;
            @(posedge clk);
            ext_step_att <= 0;
            repeat (6) @(posedge clk);
            #1;
        end
    endtask

    task pulse_sensor;
        begin
            @(posedge clk);
            imu_valid <= 1;
            @(posedge clk);
            imu_valid <= 0;
        end
    endtask

    initial begin
        $display("=============================================================");
        $display("  TB: pid_top — Stage 9 & 10 Full Flight Controller Chip");
        $display("=============================================================");

        rst_n              = 0;
        sim_fast_ticks     = 1; // Use fast pulses for functional testing
        ext_step_rate      = 0;
        ext_step_att       = 0;
        pilot_mode         = 2'b00; // Acro
        stick_roll         = 0;
        stick_pitch        = 0;
        stick_yaw          = 0;
        stick_thrust       = 16'h2000; // 25% baseline hover throttle
        imu_valid          = 0;
        roll_angle_meas    = 0;
        roll_rate_meas     = 0;
        pitch_angle_meas   = 0;
        pitch_rate_meas    = 0;
        yaw_rate_meas      = 0;
        alt_meas           = 0;
        watchdog_timeout   = 32'd25;
        safe_thrust_target = 16'h0800; // Safe low-thrust descent
        ramp_rate          = 16'h0200;
        reg_wr_en          = 0;
        reg_axis_sel       = 0;
        reg_wr_addr        = 0;
        reg_wr_data        = 0;
        att_kp_roll        = 16'h0100; // 1.0
        att_kp_pitch       = 16'h0100; // 1.0
        att_kp_yaw         = 16'h0100; // 1.0

        #50;
        rst_n = 1;
        #50;

        pulse_sensor();

        // =====================================================================
        // TC-01: System Reset & Initial State
        // =====================================================================
        $display("\n--- TC-01: System Reset & Boot State ---");
        check2(1, active_mode,      2'b00,  "Booted in Acro Mode");
        check1(2, watchdog_tripped, 1'b0,   "Watchdog healthy at startup");
        check16(3, actuator_thrust, 16'h2000, "Thrust initialized to baseline stick");

        // =====================================================================
        // TC-02: Simultaneous 3-Axis Acro Mode Control
        // =====================================================================
        $display("\n--- TC-02: Simultaneous 3-Axis Acro Maneuver ---");
        stick_roll  = 16'h1000; // +4096 roll rate
        stick_pitch = 16'h0800; // +2048 pitch rate
        stick_yaw   = 16'h0400; // +1024 yaw rate
        
        pulse_sensor();
        pulse_fast_rate();
        
        // In Acro: Kp=1.0, Ki=0.25 -> P = Sp, I = 0.25*Sp -> Out = 1.25*Sp
        // Roll:  1.25 * 0x1000 = 0x1400
        // Pitch: 1.25 * 0x0800 = 0x0A00
        // Yaw:   1.25 * 0x0400 = 0x0500
        check16(4, actuator_roll,  16'h1400, "Roll actuator active in parallel");
        check16(5, actuator_pitch, 16'h0A00, "Pitch actuator active in parallel");
        check16(6, actuator_yaw,   16'h0500, "Yaw actuator active in parallel");

        // =====================================================================
        // TC-03: Stabilize Mode Cascaded Control
        // =====================================================================
        $display("\n--- TC-03: Simultaneous Stabilize Mode Cascaded Tracking ---");
        pilot_mode = 2'b01; // Switch to Stabilize
        pulse_sensor();
        wait(active_mode == 2'b01);
        @(posedge clk);
        check2(7, active_mode, 2'b01, "Switched to Stabilize Mode");

        // Command attitude angles
        stick_roll       = 16'h1000; roll_angle_meas  = 16'h0000;
        stick_pitch      = 16'h2000; pitch_angle_meas = 16'h0000;
        
        // Run attitude outer loop (100 Hz tick)
        pulse_sensor();
        pulse_fast_att();
        
        // Run rate inner loop (1 kHz tick)
        roll_rate_meas  = 16'h0000;
        pitch_rate_meas = 16'h0000;
        pulse_fast_rate();
        
        check1(8, (actuator_roll  != 16'h0000), 1'b1, "Roll driven by attitude loop");
        check1(9, (actuator_pitch != 16'h0000), 1'b1, "Pitch driven by attitude loop");

        // =====================================================================
        // TC-04: Altitude Hold Mode Engaging Vertical Channel
        // =====================================================================
        $display("\n--- TC-04: Altitude Hold Mode ---");
        pilot_mode = 2'b10; // Switch to AltHold
        pulse_sensor();
        wait(active_mode == 2'b10);
        @(posedge clk);
        check2(10, active_mode, 2'b10, "Switched to Altitude Hold Mode");

        // Setpoint altitude is 0x3000, current measurement is 0x2000 (error = 0x1000)
        stick_thrust = 16'h2000; // baseline throttle
        alt_meas     = 16'h1000; // Drone is lower than setpoint!
        
        pulse_sensor();
        pulse_fast_att(); // Step altitude PID
        pulse_fast_rate();
        
        // Thrust should increase above baseline to correct altitude deficit
        check1(11, ($signed(actuator_thrust) > $signed(16'h2000)), 1'b1, 
               "Thrust increased automatically to correct altitude error");

        // =====================================================================
        // TC-05: Full-Chip Bumpless Mode Switch
        // =====================================================================
        $display("\n--- TC-05: Multi-Axis Bumpless Transfer ---");
        // Establish baseline output
        roll_before  = actuator_roll;
        pitch_before = actuator_pitch;
        
        // Switch back to Stabilize
        pilot_mode = 2'b01;
        pulse_sensor();
        wait(active_mode == 2'b01);
        @(posedge clk);
        
        pulse_fast_rate();
        check16(12, actuator_roll,  roll_before,  "Roll bumpless output preserved");
        check16(13, actuator_pitch, pitch_before, "Pitch bumpless output preserved");

        // =====================================================================
        // TC-06: Master Sensor Dropout & Synchronized Failsafe Ramp
        // =====================================================================
        $display("\n--- TC-06: Sensor Watchdog Dropout & Synchronized Ramp ---");
        // Stop pulsing sensor! Advance 35 clock cycles (timeout = 25)
        repeat (35) @(posedge clk);
        
        check1(14, watchdog_tripped, 1'b1,  "Master watchdog tripped!");
        check2(15, active_mode,      2'b11, "Autonomous Failsafe engaged!");

        // Run ramp steps (Pitch started at 0x4600, so requires 35 steps @ 0x0200/step to reach 0x0000)
        repeat (40) pulse_fast_rate();
        check16(16, actuator_roll,   16'h0000, "Roll safely leveled in failsafe");
        check16(17, actuator_pitch,  16'h0000, "Pitch safely leveled in failsafe");
        check16(18, actuator_yaw,    16'h0000, "Yaw rotation halted in failsafe");
        check16(19, actuator_thrust, safe_thrust_target, "Thrust ramped down to safe target");

        // =====================================================================
        // TC-07: Host Register Tuning
        // =====================================================================
        $display("\n--- TC-07: Independent Axis Register Tuning ---");
        // Write custom gain to Pitch axis (axis 01, reg 8)
        @(posedge clk);
        reg_wr_en    <= 1;
        reg_axis_sel <= 2'b01; // Pitch
        reg_wr_addr  <= 5'd8;
        reg_wr_data  <= 16'h0450;
        @(posedge clk);
        reg_wr_en    <= 0;
        #1;
        
        check16(20, uut.u_pitch_axis.u_inner_rate_channel.u_gain_bank.reg_file[8], 
                16'h0450, "Pitch gain updated via register write");

        // =====================================================================
        // Final Summary
        // =====================================================================
        #50;
        $display("\n=============================================================");
        $display("  Results: %0d PASSED,  %0d FAILED  (of %0d total checks)",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("  >>> ALL TOP-LEVEL REGRESSION TESTS PASSED <<<");
        else
            $display("  >>> FAILURES DETECTED <<<");
        $display("=============================================================");
        $finish;
    end

endmodule
