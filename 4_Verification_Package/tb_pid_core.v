// =============================================================================
// File        : tb_pid_core.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Self-checking testbench for Stage 4 pid_core.v
//               Covers:
//               - P, I, D functionality independently
//               - Anti-windup clamping and output saturation
//               - Low-pass derivative filtering (smoothing & bypass)
//               - Integrator preload for bumpless mode transfer
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_core;

    // -------------------------------------------------------------------------
    // DUT Signals
    // -------------------------------------------------------------------------
    reg         clk;
    reg         rst_n;
    reg         step;
    reg  [15:0] setpoint;
    reg  [15:0] measurement;
    reg  [15:0] kp, ki, kd;
    reg  [15:0] d_filter_coeff;
    reg  [15:0] integ_max, integ_min;
    reg  [15:0] out_max, out_min;
    reg         integ_load;
    reg  [15:0] integ_load_val;
    
    wire [15:0] out;
    wire [15:0] integ_state;
    wire        done;

    pid_core uut (
        .clk            (clk),
        .rst_n          (rst_n),
        .step           (step),
        .setpoint       (setpoint),
        .measurement    (measurement),
        .kp             (kp),
        .ki             (ki),
        .kd             (kd),
        .d_filter_coeff (d_filter_coeff),
        .integ_max      (integ_max),
        .integ_min      (integ_min),
        .out_max        (out_max),
        .out_min        (out_min),
        .integ_load     (integ_load),
        .integ_load_val (integ_load_val),
        .out            (out),
        .integ_state    (integ_state),
        .done           (done)
    );

    // -------------------------------------------------------------------------
    // Clock Generation (50 MHz)
    // -------------------------------------------------------------------------
    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 20ns period
    end

    // -------------------------------------------------------------------------
    // Test Infrastructure
    // -------------------------------------------------------------------------
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

    // Helper task to trigger one step and wait for done
    task do_step;
        begin
            @(posedge clk);
            step <= 1;
            @(posedge clk);
            step <= 0;
            wait(done == 1);
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Main Test Sequence
    // -------------------------------------------------------------------------
    initial begin
        $display("=============================================================");
        $display("  TB: pid_core — Stage 4 (Filtered D + Bumpless Preload)");
        $display("=============================================================");
        
        // Initialize inputs
        rst_n          = 0;
        step           = 0;
        setpoint       = 0;
        measurement    = 0;
        kp             = 0;
        ki             = 0;
        kd             = 0;
        d_filter_coeff = 16'h7FFF; // 1.0 (filter bypass by default)
        integ_max      = 16'h7FFF;
        integ_min      = 16'h8000;
        out_max        = 16'h7FFF;
        out_min        = 16'h8000;
        integ_load     = 0;
        integ_load_val = 0;

        // Reset system
        #50;
        rst_n = 1;
        #50;

        // =====================================================================
        // TC-01: Proportional Only (Kp = 1.0)
        // =====================================================================
        $display("\n--- Test: Proportional Only ---");
        kp = 16'h0100; // 1.0 in Q8.8
        ki = 0; kd = 0;
        setpoint    = 16'h1000; // +4096
        measurement = 16'h0000;
        
        do_step();
        check16(1, out, 16'h1000, "P-only step response");

        // =====================================================================
        // TC-02: Integral Only (Ki = 0.5)
        // =====================================================================
        $display("\n--- Test: Integral Only ---");
        kp = 0;
        ki = 16'h0080; // 0.5 in Q8.8
        kd = 0;
        setpoint    = 16'h1000;
        measurement = 16'h0000;
        
        rst_n = 0; #20; rst_n = 1; #20;
        
        do_step();
        check16(2, out, 16'h0800, "I-only Step 1");
        
        do_step();
        check16(3, out, 16'h1000, "I-only Step 2");
        
        do_step();
        check16(4, out, 16'h1800, "I-only Step 3");

        // =====================================================================
        // TC-03: Derivative Only with Filter Bypass (alpha = 1.0 / 0x7FFF)
        // =====================================================================
        $display("\n--- Test: Derivative Only (Unfiltered) ---");
        rst_n = 0; #20; rst_n = 1; #20;
        kp = 0; ki = 0; 
        kd = 16'h0100; // 1.0 in Q8.8
        d_filter_coeff = 16'h7FFF; // 1.0 (bypass)
        
        setpoint = 0; measurement = 0;
        do_step();
        
        // Measurement jumps by 0x1000
        measurement = 16'h1000;
        do_step();
        check16(5, out, 16'hF000, "D kick suppression (unfiltered)");

        do_step();
        check16(6, out, 16'h0000, "D settles to zero");

        // =====================================================================
        // TC-04: Anti-Windup Integrator Clamping
        // =====================================================================
        $display("\n--- Test: Anti-Windup Clamping ---");
        rst_n = 0; #20; rst_n = 1; #20;
        kp = 0; kd = 0;
        ki = 16'h0100; // 1.0 in Q8.8
        setpoint = 16'h2000;
        measurement = 0;
        
        integ_max = 16'h3000;
        integ_min = 16'h8000;
        
        do_step();
        check16(7, out, 16'h2000, "I-clamp step 1");
        
        do_step();
        check16(8, out, 16'h3000, "I-clamp applied at 0x3000");

        do_step();
        check16(9, out, 16'h3000, "I-clamp holds at 0x3000");
        
        setpoint = 16'hE000; // -0x2000
        do_step();
        check16(10, out, 16'h1000, "I-clamp immediate unwind");

        // =====================================================================
        // TC-05: Final Output Saturation
        // =====================================================================
        $display("\n--- Test: Output Saturation ---");
        rst_n = 0; #20; rst_n = 1; #20;
        
        kp = 16'h0200; // 2.0 in Q8.8
        ki = 0; kd = 0;
        
        out_max = 16'h2500;
        out_min = 16'hD000; // -0x3000
        
        setpoint = 16'h1000; measurement = 0;
        do_step();
        check16(11, out, 16'h2000, "Out sat below max");
        
        setpoint = 16'h2000; measurement = 0;
        do_step();
        check16(12, out, 16'h2500, "Out sat clamped to max");
        
        setpoint = 16'hE000; measurement = 0;
        do_step();
        check16(13, out, 16'hD000, "Out sat clamped to min");

        // =====================================================================
        // TC-06: Low-Pass Filtered Derivative (alpha = 0.5 / 0x4000)
        // =====================================================================
        $display("\n--- Test: Filtered Derivative (alpha = 0.5) ---");
        rst_n = 0; #20; rst_n = 1; #20;
        // Reset limits to full range
        out_max   = 16'h7FFF;
        out_min   = 16'h8000;
        integ_max = 16'h7FFF;
        integ_min = 16'h8000;

        kp = 0; ki = 0;
        kd = 16'h0100;             // 1.0 in Q8.8
        d_filter_coeff = 16'h4000; // alpha = 0.5
        
        setpoint = 0; measurement = 0;
        do_step();
        
        // Measurement jumps by 0x2000
        measurement = 16'h2000;
        do_step();
        check16(14, out, 16'hF000, "Filtered D step 1 (smoothed to 50%)");
        
        // Measurement stays at 0x2000
        do_step();
        check16(15, out, 16'hF800, "Filtered D step 2 (decaying to 25%)");

        // =====================================================================
        // TC-07: Bumpless Transfer Integrator Preload
        // =====================================================================
        $display("\n--- Test: Bumpless Integrator Preload ---");
        rst_n = 0; #20; rst_n = 1; #20;
        kp = 0; ki = 16'h0100; kd = 0;
        
        // Full limits
        out_max   = 16'h7FFF;
        out_min   = 16'h8000;
        integ_max = 16'h7FFF;
        integ_min = 16'h8000;
        
        // Preload integrator cleanly on posedge clk
        @(posedge clk);
        integ_load     <= 1;
        integ_load_val <= 16'h4500;
        @(posedge clk);
        integ_load     <= 0;
        integ_load_val <= 16'h0000;
        #1; // Allow settling
        
        check16(16, integ_state, 16'h4500, "Integrator state preloaded");
        
        // Run a step with zero error; output must equal preloaded value with 0 bump
        setpoint = 0; measurement = 0;
        do_step();
        check16(17, out, 16'h4500, "Bumpless output matches preloaded state");

        // =====================================================================
        // Final summary
        // =====================================================================
        #50;
        $display("\n=============================================================");
        $display("  Results: %0d PASSED,  %0d FAILED  (of %0d total checks)",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("  >>> ALL STAGE 4 TESTS PASSED <<<");
        else
            $display("  >>> FAILURES DETECTED <<<");
        $display("=============================================================");
        $finish;
    end

endmodule
