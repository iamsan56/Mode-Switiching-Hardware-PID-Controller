// =============================================================================
// File        : tb_pid_helpers.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Self-checking testbench for pid_helpers.v
//               Tests all four arithmetic primitives against manually
//               computed expected values. Uses $error() to flag failures.
//
// How to run in ModelSim (after adding to project):
//   vsim work.tb_pid_helpers
//   run -all
//   Look for "ALL TESTS PASSED" or individual FAIL messages.
//
// Verilog-2001 compliant.
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_helpers;

    // -------------------------------------------------------------------------
    // Test infrastructure: pass/fail counters
    // -------------------------------------------------------------------------
    integer test_num;
    integer pass_count;
    integer fail_count;

    // Task: check a 16-bit result and report pass/fail
    // Inputs passed as task arguments (Verilog-2001 compatible)
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

    // -------------------------------------------------------------------------
    // DUT signals — sat_clamp
    // -------------------------------------------------------------------------
    reg  [31:0] clamp_in;
    reg  [15:0] clamp_lo;
    reg  [15:0] clamp_hi;
    wire [15:0] clamp_out;

    sat_clamp uut_clamp (
        .in_wide (clamp_in),
        .lo      (clamp_lo),
        .hi      (clamp_hi),
        .out     (clamp_out)
    );

    // -------------------------------------------------------------------------
    // DUT signals — sat_add16
    // -------------------------------------------------------------------------
    reg  [15:0] add_a, add_b;
    wire [15:0] add_out;
    wire        add_ov;

    sat_add16 uut_add (
        .a        (add_a),
        .b        (add_b),
        .out      (add_out),
        .overflow (add_ov)
    );

    // -------------------------------------------------------------------------
    // DUT signals — sat_mul_q15
    // -------------------------------------------------------------------------
    reg  [15:0] mq15_a, mq15_b;
    wire [15:0] mq15_out;
    wire        mq15_ov;

    sat_mul_q15 uut_mulq15 (
        .a        (mq15_a),
        .b        (mq15_b),
        .out      (mq15_out),
        .overflow (mq15_ov)
    );

    // -------------------------------------------------------------------------
    // DUT signals — sat_mul_gain
    // -------------------------------------------------------------------------
    reg  [15:0] mg_data, mg_gain;
    wire [15:0] mg_out;
    wire        mg_ov;

    sat_mul_gain uut_mulgain (
        .data     (mg_data),
        .gain     (mg_gain),
        .out      (mg_out),
        .overflow (mg_ov)
    );

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        pass_count = 0;
        fail_count = 0;
        test_num   = 0;

        $display("=============================================================");
        $display("  TB: pid_helpers — Stage 1 Arithmetic Primitives");
        $display("=============================================================");

        // =====================================================================
        // SECTION 1: sat_clamp tests
        // =====================================================================
        $display("\n--- sat_clamp ---");

        // TC-01: Value within range (no clamping)
        clamp_in = 32'sh00001000;   // +4096 → within [-32768, +32767]
        clamp_lo = 16'h8000;        // -32768
        clamp_hi = 16'h7FFF;        // +32767
        #1;
        check16(1, clamp_out, 16'h1000, "within range");

        // TC-02: Value above upper limit → clamp to hi
        clamp_in = 32'sh00010000;   // +65536 (overflows Q15 range)
        clamp_lo = 16'h8000;
        clamp_hi = 16'h7FFF;
        #1;
        check16(2, clamp_out, 16'h7FFF, "above hi → saturate to 7FFF");

        // TC-03: Value below lower limit → clamp to lo
        clamp_in = 32'shFFFF0000;   // very negative
        clamp_lo = 16'h8000;
        clamp_hi = 16'h7FFF;
        #1;
        check16(3, clamp_out, 16'h8000, "below lo → saturate to 8000");

        // TC-04: Exact at upper boundary (should pass through, no saturation)
        clamp_in = 32'sh00007FFF;
        clamp_lo = 16'h8000;
        clamp_hi = 16'h7FFF;
        #1;
        check16(4, clamp_out, 16'h7FFF, "exactly at hi boundary");

        // TC-05: Exact at lower boundary
        clamp_in = 32'shFFFF8000;
        clamp_lo = 16'h8000;
        clamp_hi = 16'h7FFF;
        #1;
        check16(5, clamp_out, 16'h8000, "exactly at lo boundary");

        // TC-06: Custom asymmetric limits
        clamp_in = 32'sh00000200;   // +512
        clamp_lo = 16'hFF00;        // -256
        clamp_hi = 16'h0100;        // +256
        #1;
        check16(6, clamp_out, 16'h0100, "above asymmetric hi");

        // TC-07: Zero input, zero limits (edge case)
        clamp_in = 32'sh00000000;
        clamp_lo = 16'h0000;
        clamp_hi = 16'h0000;
        #1;
        check16(7, clamp_out, 16'h0000, "zero in, zero limits");

        // =====================================================================
        // SECTION 2: sat_add16 tests
        // =====================================================================
        $display("\n--- sat_add16 ---");

        // TC-08: Normal addition, no overflow
        add_a = 16'sh0100;   // +256
        add_b = 16'sh0200;   // +512
        #1;
        // Expected: +768 = 0x0300, no overflow
        check16(8,  add_out, 16'h0300, "normal positive add");
        check1 (9,  add_ov,  1'b0,     "normal add no overflow flag");

        // TC-10: Normal negative addition
        add_a = 16'shFF00;   // -256
        add_b = 16'shFE00;   // -512
        #1;
        // Expected: -768 = 0xFD00
        check16(10, add_out, 16'hFD00, "normal negative add");
        check1 (11, add_ov,  1'b0,     "negative add no overflow flag");

        // TC-12: Positive overflow → saturate to 0x7FFF
        add_a = 16'h7FFF;   // +32767 (max positive)
        add_b = 16'h0001;   // +1
        #1;
        check16(12, add_out, 16'h7FFF, "positive overflow → 7FFF");
        check1 (13, add_ov,  1'b1,     "positive overflow flag");

        // TC-14: Negative overflow → saturate to 0x8000
        add_a = 16'h8000;   // -32768 (min negative)
        add_b = 16'hFFFF;   // -1
        #1;
        check16(14, add_out, 16'h8000, "negative overflow → 8000");
        check1 (15, add_ov,  1'b1,     "negative overflow flag");

        // TC-16: Mixed signs — can never overflow
        add_a = 16'h7FFF;   // +32767
        add_b = 16'h8000;   // -32768
        #1;
        // Result: -1 = 0xFFFF
        check16(16, add_out, 16'hFFFF, "mixed signs no overflow");
        check1 (17, add_ov,  1'b0,     "mixed signs no overflow flag");

        // TC-18: Both zero
        add_a = 16'h0000;
        add_b = 16'h0000;
        #1;
        check16(18, add_out, 16'h0000, "0 + 0 = 0");
        check1 (19, add_ov,  1'b0,     "0 + 0 no overflow flag");

        // =====================================================================
        // SECTION 3: sat_mul_q15 tests
        // =====================================================================
        $display("\n--- sat_mul_q15 ---");

        // Q15 encoding:
        //   +1.0  = 0x7FFF (closest representation)
        //   -1.0  = 0x8000
        //   +0.5  = 0x4000
        //   -0.5  = 0xC000

        // TC-20: 0.5 × 0.5 = 0.25
        // 0x4000 * 0x4000 = 0x1000_0000; >>15 = 0x2000 = 0.25 ✓
        mq15_a = 16'h4000;
        mq15_b = 16'h4000;
        #1;
        check16(20, mq15_out, 16'h2000, "0.5 × 0.5 = 0.25");
        check1 (21, mq15_ov,  1'b0,     "0.5x0.5 no overflow");

        // TC-22: 1.0 × 1.0 — nearest Q15 representation
        // 0x7FFF * 0x7FFF = 0x3FFF_0001; >>15 = 0x7FFE (≈0.9999)
        mq15_a = 16'h7FFF;
        mq15_b = 16'h7FFF;
        #1;
        check16(22, mq15_out, 16'h7FFE, "0x7FFF × 0x7FFF (near 1.0)");
        check1 (23, mq15_ov,  1'b0,     "no overflow for 0x7FFF × 0x7FFF");

        // TC-24: -1.0 × -1.0 → saturation edge case
        // 0x8000 * 0x8000 = signed: (-32768) * (-32768) = +1073741824
        // = 0x4000_0000; >>15 = 0x8000 which is -1.0 in Q15 (wrong!)
        // → Must saturate to 0x7FFF
        mq15_a = 16'h8000;
        mq15_b = 16'h8000;
        #1;
        check16(24, mq15_out, 16'h7FFF, "-1.0 × -1.0 saturates to 0x7FFF");
        check1 (25, mq15_ov,  1'b1,     "-1.0 × -1.0 overflow flag");

        // TC-26: +0.5 × -1.0 = -0.5
        // 0x4000 * 0x8000 = 0xE000_0000 (signed); >>15 = 0xFFFFC000 → 0xC000 = -0.5 ✓
        mq15_a = 16'h4000;
        mq15_b = 16'h8000;
        #1;
        check16(26, mq15_out, 16'hC000, "+0.5 × -1.0 = -0.5");
        check1 (27, mq15_ov,  1'b0,     "+0.5 × -1.0 no overflow");

        // TC-28: Anything × 0 = 0
        mq15_a = 16'h7FFF;
        mq15_b = 16'h0000;
        #1;
        check16(28, mq15_out, 16'h0000, "any × 0 = 0");
        check1 (29, mq15_ov,  1'b0,     "any × 0 no overflow");

        // =====================================================================
        // SECTION 4: sat_mul_gain tests (Q15 × Q8.8)
        // =====================================================================
        $display("\n--- sat_mul_gain ---");

        // Q8.8 encoding for gain:
        //   +1.0  = 0x0100
        //   +2.0  = 0x0200
        //   -1.0  = 0xFF00
        //   +0.5  = 0x0080

        // TC-30: error=0.5, gain=1.0 → result=0.5
        // data=0x4000 (Q15), gain=0x0100 (Q8.8)
        // product = 0x4000 * 0x0100 = 0x0040_0000; >>8 = 0x004000 → 0x4000 ✓
        mg_data = 16'h4000;   // +0.5 in Q15
        mg_gain = 16'h0100;   // +1.0 in Q8.8
        #1;
        check16(30, mg_out, 16'h4000, "0.5 × 1.0 = 0.5");
        check1 (31, mg_ov,  1'b0,     "0.5 × 1.0 no overflow");

        // TC-32: error=0.5, gain=2.0 → result=1.0 (near max)
        mg_data = 16'h4000;   // +0.5
        mg_gain = 16'h0200;   // +2.0
        #1;
        // 0x4000 * 0x0200 = 0x0080_0000; >>8 = 0x008000 → 0x7FFF (saturate!)
        // Wait: 0x008000 = 32768 which is > 0x7FFF=32767 → saturate
        check16(32, mg_out, 16'h7FFF, "0.5 × 2.0 saturates to 0x7FFF");
        check1 (33, mg_ov,  1'b1,     "0.5 × 2.0 overflow flag");

        // TC-34: negative error × positive gain → negative result
        // data=-0.5=0xC000, gain=+1.0=0x0100
        // product = (-16384) * (256) = -4194304 = 0xFFC0_0000; >>8 = 0xFFFFC000 = 0xC000 ✓
        mg_data = 16'hC000;   // -0.5 in Q15
        mg_gain = 16'h0100;   // +1.0 in Q8.8
        #1;
        check16(34, mg_out, 16'hC000, "-0.5 × 1.0 = -0.5");
        check1 (35, mg_ov,  1'b0,     "-0.5 × 1.0 no overflow");

        // TC-36: zero data → zero output regardless of gain
        mg_data = 16'h0000;
        mg_gain = 16'h7FFF;   // large gain
        #1;
        check16(36, mg_out, 16'h0000, "0 × large_gain = 0");
        check1 (37, mg_ov,  1'b0,     "0 × large_gain no overflow");

        // TC-38: small gain (0.5) applied to max positive error
        // data=0x7FFF, gain=0x0080 (+0.5 in Q8.8)
        // product = 32767 * 128 = 4194176 = 0x003F_FF80; >>8 = 0x003FFF → 0x3FFF ✓
        mg_data = 16'h7FFF;
        mg_gain = 16'h0080;   // +0.5 in Q8.8
        #1;
        check16(38, mg_out, 16'h3FFF, "max_err × 0.5 ≈ 0.5");
        check1 (39, mg_ov,  1'b0,     "max_err × 0.5 no overflow");

        // =====================================================================
        // Final summary
        // =====================================================================
        #10;
        $display("\n=============================================================");
        $display("  Results: %0d PASSED,  %0d FAILED  (of %0d total checks)",
                 pass_count, fail_count, pass_count + fail_count);
        if (fail_count == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> FAILURES DETECTED — review output above <<<");
        $display("=============================================================");
        $finish;
    end

endmodule
