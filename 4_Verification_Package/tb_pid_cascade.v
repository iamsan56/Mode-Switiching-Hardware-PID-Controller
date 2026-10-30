// =============================================================================
// File        : tb_pid_cascade.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Self-checking testbench for Stage 7 & 8:
//               - Cascaded Attitude (100 Hz) -> Rate (1 kHz) loop operation
//               - Acro mode manual bypass verification
//               - Sensor Watchdog expiration on stale IMU data
//               - Autonomous Failsafe entry on watchdog timeout
//               - Failsafe actuator ramp behavior (controlled descent)
//               - Sensor recovery after dropout
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_cascade;

    reg clk;
    reg rst_n;

    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50 MHz
    end

    // Multi-rate triggers
    reg        step_rate;
    reg        step_att;
    
    // Pilot controls
    reg [1:0]  pilot_mode;
    wire [1:0] active_mode;
    wire       watchdog_tripped;
    
    // Watchdog & Failsafe configuration
    reg        sensor_fresh;
    reg [31:0] watchdog_timeout;
    reg [15:0] safe_target;
    reg [15:0] ramp_rate;
    
    // Setpoints & Measurements
    reg [15:0] stick_angle_sp;
    reg [15:0] stick_rate_sp;
    reg [15:0] angle_meas;
    reg [15:0] rate_meas;
    
    // Outer loop gains
    reg [15:0] att_kp;
    reg [15:0] att_ki;
    
    // Host register write (unused for baseline)
    reg        reg_wr_en;
    reg [4:0]  reg_wr_addr;
    reg [15:0] reg_wr_data;
    
    // Outputs
    wire [15:0] actuator_out;
    wire        done_rate;
    wire        done_att;

    // Instantiate Cascaded Axis
    pid_cascaded_axis uut (
        .clk              (clk),
        .rst_n            (rst_n),
        .step_rate        (step_rate),
        .step_att         (step_att),
        .pilot_mode       (pilot_mode),
        .active_mode      (active_mode),
        .watchdog_tripped (watchdog_tripped),
        .sensor_fresh     (sensor_fresh),
        .watchdog_timeout (watchdog_timeout),
        .safe_target      (safe_target),
        .ramp_rate        (ramp_rate),
        .stick_angle_sp   (stick_angle_sp),
        .stick_rate_sp    (stick_rate_sp),
        .angle_meas       (angle_meas),
        .rate_meas        (rate_meas),
        .att_kp           (att_kp),
        .att_ki           (att_ki),
        .reg_wr_en        (reg_wr_en),
        .reg_wr_addr      (reg_wr_addr),
        .reg_wr_data      (reg_wr_data),
        .actuator_out     (actuator_out),
        .done_rate        (done_rate),
        .done_att         (done_att)
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

    // Pulse rate loop tick
    task do_rate_step;
        begin
            @(posedge clk);
            step_rate <= 1;
            @(posedge clk);
            step_rate <= 0;
            wait(done_rate == 1);
            @(posedge clk);
            #1;
        end
    endtask

    // Pulse attitude loop tick
    task do_att_step;
        begin
            @(posedge clk);
            step_att <= 1;
            @(posedge clk);
            step_att <= 0;
            wait(done_att == 1);
            @(posedge clk);
            #1;
        end
    endtask

    // Pulse sensor fresh heartbeat
    task pulse_sensor;
        begin
            @(posedge clk);
            sensor_fresh <= 1;
            @(posedge clk);
            sensor_fresh <= 0;
        end
    endtask

    initial begin
        $display("=============================================================");
        $display("  TB: pid_cascade — Stage 7 & 8 Cascaded + Watchdog + Ramp");
        $display("=============================================================");

        rst_n            = 0;
        step_rate        = 0;
        step_att         = 0;
        pilot_mode       = 2'b00; // Acro
        sensor_fresh     = 0;
        watchdog_timeout = 32'd20; // Fast 20-cycle timeout for simulation
        safe_target      = 16'h0000;
        ramp_rate        = 16'h0400; // Ramp down 0x0400 per rate step
        stick_angle_sp   = 0;
        stick_rate_sp    = 0;
        angle_meas       = 0;
        rate_meas        = 0;
        att_kp           = 16'h0100; // Outer loop Kp = 1.0
        att_ki           = 16'h0000;
        reg_wr_en        = 0;
        reg_wr_addr      = 0;
        reg_wr_data      = 0;

        #50;
        rst_n = 1;
        #50;

        // Keep watchdog happy initially
        pulse_sensor();

        // =====================================================================
        // SECTION 1: Acro Mode (Outer Loop Bypass)
        // =====================================================================
        $display("\n--- Test: Acro Mode (Outer Loop Bypassed) ---");
        // Stick commands rate directly
        stick_rate_sp = 16'h1000; // +4096 rate request
        rate_meas     = 16'h0000;
        stick_angle_sp= 16'h3000; // Angle stick should be completely ignored!
        angle_meas    = 16'h0000;
        
        pulse_sensor();
        do_rate_step();
        
        // Inner rate loop in Acro uses stick_rate_sp = 0x1000 directly
        // In Acro: Kp=1.0, Ki=0.25 -> P = 0x1000, I = 0x0400 -> Out = 0x1400
        check16(1, uut.inner_rate_sp, 16'h1000, "Acro uses manual rate setpoint directly");
        check16(2, actuator_out,      16'h1400, "Acro rate loop output matches stick command");

        // =====================================================================
        // SECTION 2: Stabilize Mode (Cascaded Attitude -> Rate)
        // =====================================================================
        $display("\n--- Test: Stabilize Mode (Cascaded Loop) ---");
        // Switch to Stabilize
        pilot_mode = 2'b01;
        pulse_sensor();
        @(posedge clk);
        wait(uut.u_inner_rate_channel.active_mode == 2'b01);
        @(posedge clk);
        check2(3, active_mode, 2'b01, "Stabilize mode active");

        // Pilot commands an angle: +30 degrees (0x2000 in Q15)
        stick_angle_sp = 16'h2000;
        angle_meas     = 16'h0000;
        stick_rate_sp  = 16'h7FFF; // Should be ignored in Stabilize!
        
        // Step the outer attitude loop (100 Hz tick)
        // Outer loop: Err = 0x2000. att_kp = 1.0. Outer rate cmd = 0x2000.
        pulse_sensor();
        do_att_step();
        check16(4, uut.outer_rate_cmd, 16'h2000, "Outer attitude PID generated desired rate");
        check16(5, uut.inner_rate_sp,  16'h2000, "Inner rate loop receives cascaded setpoint");

        // Now step the inner rate loop (1 kHz tick)
        rate_meas = 16'h0000;
        do_rate_step();
        check1(6, (actuator_out != 16'h0000), 1'b1, "Actuator driven by cascaded command");

        // =====================================================================
        // SECTION 3: Sensor Watchdog Expiration & Autonomous Failsafe
        // =====================================================================
        $display("\n--- Test: Sensor Watchdog Expiration ---");
        check1(7, watchdog_tripped, 1'b0, "Watchdog healthy before dropout");
        
        // Simulate sensor failure: DO NOT PULSE sensor_fresh!
        // Advance clock by 30 cycles (timeout is set to 20 cycles)
        repeat (30) @(posedge clk);
        
        check1(8, watchdog_tripped, 1'b1,  "Watchdog tripped on stale data!");
        check2(9, active_mode,      2'b11, "Autonomous Failsafe mode forced by watchdog!");

        // =====================================================================
        // SECTION 4: Controlled Failsafe Ramp
        // =====================================================================
        $display("\n--- Test: Controlled Failsafe Actuator Ramp ---");
        // Output before failsafe was non-zero.
        // On each rate step, it must ramp down toward safe_target (0x0000) by ramp_rate (0x0400).
        $display("  [INFO] Actuator command entering ramp = 0x%04X", actuator_out);
        
        // Step 1: Ramp down
        do_rate_step();
        $display("  [INFO] Actuator command after Ramp Step 1 = 0x%04X", actuator_out);
        
        // Step 2: Continue ramping
        do_rate_step();
        $display("  [INFO] Actuator command after Ramp Step 2 = 0x%04X", actuator_out);
        
        // Run enough steps to reach safe_target = 0x0000
        repeat (15) do_rate_step();
        check16(10, actuator_out, 16'h0000, "Failsafe ramp smoothly brought output to safe target 0x0000");

        // =====================================================================
        // SECTION 5: Sensor Recovery
        // =====================================================================
        $display("\n--- Test: Sensor Data Recovery ---");
        // Sensor comes back alive!
        pulse_sensor();
        @(posedge clk);
        check1(11, watchdog_tripped, 1'b0, "Watchdog cleared upon sensor recovery");
        
        // Wait for Mode FSM to finish bumpless transition back to Stabilize
        wait(active_mode == 2'b01);
        @(posedge clk);
        check2(12, active_mode,      2'b01, "Pilot Stabilize mode restored");

        // =====================================================================
        // Final Summary
        // =====================================================================
        #50;
        $display("\n=============================================================");
        $display("  Results: %0d PASSED,  %0d FAILED  (of %0d total checks)",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("  >>> ALL STAGE 7 & 8 TESTS PASSED <<<");
        else
            $display("  >>> FAILURES DETECTED <<<");
        $display("=============================================================");
        $finish;
    end

endmodule
