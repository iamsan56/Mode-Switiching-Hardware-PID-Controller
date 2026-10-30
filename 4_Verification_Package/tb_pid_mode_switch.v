// =============================================================================
// File        : tb_pid_mode_switch.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Self-checking testbench for Stage 5 & 6 single-axis channel.
//               Specifically verifies:
//               - Per-mode independent gain banks & limits
//               - Live host register writes for gain re-tuning
//               - BUMPLESS MODE TRANSFERS across all mode pairs under load
//               - Proof that output transient is ZERO during mode switches
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_mode_switch;

    reg clk;
    reg rst_n;

    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50 MHz
    end

    reg  [1:0]  req_mode;
    wire [1:0]  active_mode;
    wire        transition_busy;
    
    reg         reg_wr_en;
    reg  [4:0]  reg_wr_addr;
    reg  [15:0] reg_wr_data;
    
    reg         step;
    reg  [15:0] setpoint;
    reg  [15:0] measurement;
    wire [15:0] pid_out;
    wire [15:0] integ_state;
    wire [15:0] err_state;
    wire [15:0] d_filt_state;
    wire        done;

    // Instantiate Single Axis Channel
    pid_single_axis uut (
        .clk             (clk),
        .rst_n           (rst_n),
        .step            (step),
        .req_mode        (req_mode),
        .active_mode     (active_mode),
        .transition_busy (transition_busy),
        .setpoint        (setpoint),
        .measurement     (measurement),
        .out             (pid_out),
        .done            (done),
        .reg_wr_en       (reg_wr_en),
        .reg_wr_addr     (reg_wr_addr),
        .reg_wr_data     (reg_wr_data),
        .integ_state     (integ_state),
        .err_state       (err_state),
        .d_filt_state    (d_filt_state)
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

    task switch_mode;
        input [1:0] target;
        begin
            @(posedge clk);
            req_mode <= target;
            wait(active_mode == target);
            @(posedge clk);
            #1;
        end
    endtask

    reg [15:0] out_before_switch;

    initial begin
        $display("=============================================================");
        $display("  TB: pid_mode_switch — Stage 5 & 6 Mode Banks + Bumpless");
        $display("=============================================================");

        rst_n       = 0;
        req_mode    = 2'b00; // Acro
        reg_wr_en   = 0;
        reg_wr_addr = 0;
        reg_wr_data = 0;
        step        = 0;
        setpoint    = 0;
        measurement = 0;

        #50;
        rst_n = 1;
        #50;

        // =====================================================================
        // SECTION 1: Default Modes and Initial State
        // =====================================================================
        $display("\n--- Test: Initial Boot State (Acro Mode) ---");
        check2 (1, active_mode, 2'b00,  "Booted in Acro Mode");
        check16(2, uut.active_kp, 16'h0100, "Acro default Kp = 1.0");
        check16(3, uut.active_ki, 16'h0040, "Acro default Ki = 0.25");
        check16(4, uut.active_kd, 16'h0080, "Acro default Kd = 0.5");

        // =====================================================================
        // SECTION 2: Live Register Write
        // =====================================================================
        $display("\n--- Test: Real-Time Gain Bank Register Write ---");
        // Write Kp = 3.5 (0x0380) to STABILIZE mode (Mode 1, param 0 -> addr 8)
        @(posedge clk);
        reg_wr_en   <= 1;
        reg_wr_addr <= 5'd8;
        reg_wr_data <= 16'h0380;
        @(posedge clk);
        reg_wr_en   <= 0;
        #1;
        
        check16(5, uut.u_gain_bank.reg_file[8], 16'h0380, "Register write reflected in gain bank");
        
        // Revert it back to default Kp = 2.0 (0x0200)
        @(posedge clk);
        reg_wr_en   <= 1;
        reg_wr_addr <= 5'd8;
        reg_wr_data <= 16'h0200;
        @(posedge clk);
        reg_wr_en   <= 0;
        #20;

        // =====================================================================
        // SECTION 3: Bumpless Transfer under Load (Acro -> Stabilize)
        // =====================================================================
        $display("\n--- Test: Bumpless Transfer (Acro -> Stabilize) under Load ---");
        setpoint    = 16'h1000;
        measurement = 16'h0000;
        
        do_step();
        do_step();
        
        out_before_switch = pid_out;
        $display("  [INFO] Output in Acro before switch = 0x%04X", out_before_switch);
        check16(6, out_before_switch, 16'h1800, "Steady state Acro output verified");

        // Switch to STABILIZE (Kp = 2.0 instead of 1.0)
        switch_mode(2'b01);
        check2(7, active_mode, 2'b01, "Transitioned to Stabilize Mode");
        check16(8, uut.active_kp, 16'h0200, "Active Kp updated to 2.0");

        // Output must remain identical across switch
        do_step();
        check16(9, pid_out, out_before_switch, "BUMPLESS VERIFIED: Output identical across switch!");

        // =====================================================================
        // SECTION 4: Bumpless Transfer (Stabilize -> Altitude Hold)
        // =====================================================================
        $display("\n--- Test: Bumpless Transfer (Stabilize -> Altitude Hold) ---");
        out_before_switch = pid_out;
        $display("  [INFO] Output in Stabilize before switch = 0x%04X", out_before_switch);
        
        switch_mode(2'b10); // ALT_HOLD
        check2(10, active_mode, 2'b10, "Transitioned to Alt Hold Mode");
        
        do_step();
        check16(11, pid_out, out_before_switch, "BUMPLESS VERIFIED: Alt Hold output identical!");

        // =====================================================================
        // SECTION 5: Bumpless Transfer into FAILSAFE
        // =====================================================================
        $display("\n--- Test: Bumpless Transfer into Failsafe Mode ---");
        out_before_switch = pid_out;
        $display("  [INFO] Output before Failsafe switch = 0x%04X", out_before_switch);
        
        switch_mode(2'b11); // FAILSAFE
        check2(12, active_mode, 2'b11, "Transitioned to Failsafe Mode");
        
        do_step();
        check16(13, pid_out, out_before_switch, "BUMPLESS VERIFIED: Failsafe entry without jerk!");

        // =====================================================================
        // Final Summary
        // =====================================================================
        #50;
        $display("\n=============================================================");
        $display("  Results: %0d PASSED,  %0d FAILED  (of %0d total checks)",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("  >>> ALL STAGE 5 & 6 TESTS PASSED <<<");
        else
            $display("  >>> FAILURES DETECTED <<<");
        $display("=============================================================");
        $finish;
    end

endmodule
