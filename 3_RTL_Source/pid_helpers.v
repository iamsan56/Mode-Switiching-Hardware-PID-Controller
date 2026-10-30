`timescale 1ns / 1ps
// =============================================================================
// File        : pid_helpers.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Author      : VIT Chennai Hackathon 2026
// Description : Saturating fixed-point arithmetic primitives.
//               All PID modules depend on these; verify this file first.
//
// Fixed-point convention used throughout this project:
//   DATA  signals : Q1.15  signed 16-bit  (range -1.0 to +0.999969...)
//   GAIN  signals : Q8.8   signed 16-bit  (range -128.0 to +127.996...)
//   ACCUM signals : signed 32-bit wide intermediate products
//
// Verilog-2001 compliant. No FPGA-specific primitives.
// =============================================================================

// -----------------------------------------------------------------------------
// Module: sat_clamp
// Purpose: Clamp a wide signed value to a signed 16-bit saturated result.
//          Used after every multiply and accumulate to prevent overflow
//          propagating into downstream logic.
//
// Inputs:
//   in_wide [31:0]  - signed 32-bit value to clamp
//   lo      [15:0]  - signed 16-bit lower bound (treated as signed)
//   hi      [15:0]  - signed 16-bit upper bound (treated as signed)
//
// Output:
//   out     [15:0]  - saturated result
//
// Behavior:
//   if in_wide > hi  → out = hi
//   if in_wide < lo  → out = lo
//   else             → out = in_wide[15:0]  (lower 16 bits, after sign-check)
//
// Note on sign extension: in_wide is declared as a 32-bit wire.
//   When we compare it against the sign-extended 32-bit equivalents of lo/hi,
//   Verilog performs the comparison correctly as long as we declare the
//   intermediate wires as signed. See the $signed() casts below.
// -----------------------------------------------------------------------------
module sat_clamp (
    input  wire [31:0] in_wide,   // signed 32-bit input
    input  wire [15:0] lo,        // signed 16-bit lower clamp limit
    input  wire [15:0] hi,        // signed 16-bit upper clamp limit
    output reg  [15:0] out        // signed 16-bit clamped output
);

    // Sign-extend lo and hi to 32 bits for comparison against in_wide.
    // Without $signed(), Verilog treats these as unsigned — wrong for negative limits.
    wire signed [31:0] in_s  = $signed(in_wide);
    wire signed [31:0] lo_s  = {{16{lo[15]}}, lo};  // manual sign-extension
    wire signed [31:0] hi_s  = {{16{hi[15]}}, hi};

    always @(*) begin
        if (in_s > hi_s)
            out = hi;
        else if (in_s < lo_s)
            out = lo;
        else
            out = in_wide[15:0];
    end

endmodule


// -----------------------------------------------------------------------------
// Module: sat_add16
// Purpose: Signed 16-bit saturating addition.
//          Detects signed overflow and clamps to ±32767 / -32768.
//          Used for integrator accumulation and output summation.
//
// Inputs:
//   a [15:0], b [15:0] - signed 16-bit operands
//
// Output:
//   out [15:0]          - signed 16-bit saturated sum
//   overflow            - 1 if saturation was applied, 0 otherwise
//
// Signed overflow detection:
//   Adding two positives and getting a negative → positive overflow
//   Adding two negatives and getting a positive → negative overflow
//   Mixed-sign additions can never overflow.
// -----------------------------------------------------------------------------
module sat_add16 (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] out,
    output reg         overflow
);

    // Extend to 17 bits to capture carry/borrow and detect overflow.
    wire signed [16:0] sum_wide = $signed({a[15], a}) + $signed({b[15], b});

    always @(*) begin
        // Positive overflow: bit 16 = 0 but bit 15 of result = 1 (unexpected sign flip)
        // Negative overflow: bit 16 = 1 but bit 15 of result = 0
        if (sum_wide[16] != sum_wide[15]) begin
            // Saturate: if the carry bit is 0 we overflowed positive
            if (sum_wide[16] == 1'b0)
                out = 16'h7FFF;  // +32767 (maximum positive Q15)
            else
                out = 16'h8000;  // -32768 (minimum negative Q15)
            overflow = 1'b1;
        end else begin
            out      = sum_wide[15:0];
            overflow = 1'b0;
        end
    end

endmodule


// -----------------------------------------------------------------------------
// Module: sat_mul_q15
// Purpose: Signed Q15 × Q15 fixed-point multiply, result in Q15.
//          Both operands are 1.15 signed (range ±1.0).
//          Result is the upper 16 bits of the 32-bit product, with saturation.
//
// Math:
//   Q15 × Q15 → Q1.30 (32-bit, 2 integer bits, 30 fractional bits)
//   To recover Q15, right-shift by 15 bits and take lower 16 bits.
//   The shift discards fractional precision beyond 15 bits (truncation).
//
//   Special case: (-1.0) × (-1.0) in Q15 = 0x8000 × 0x8000
//   Product = 0x4000_0000 (positive, fits in Q15 after shift = 0x8000?)
//   Actually 0x4000_0000 >> 15 = 0x8000 which is -1.0, not +1.0.
//   This is the one overflow edge case in Q15 multiplication.
//   We saturate to 0x7FFF in that case.
//
// Inputs:
//   a [15:0], b [15:0] - signed Q15 operands
//
// Output:
//   out      [15:0]    - signed Q15 result
//   overflow           - 1 if the edge case saturation was applied
// -----------------------------------------------------------------------------
module sat_mul_q15 (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] out,
    output reg         overflow
);

    // Full 32-bit signed product (Q1.30 format)
    wire signed [31:0] product = $signed(a) * $signed(b);

    // Right-shift by 15 to return to Q15.
    // Arithmetic right shift (>>>15) preserves sign.
    // We keep 32 bits before slicing to catch the edge case.
    wire signed [31:0] shifted = product >>> 15;

    always @(*) begin
        // Edge case: 0x8000 * 0x8000 = 0x4000_0000; >>15 = 0x8000 = -1.0
        // That is actually correct mathematically: (-1.0)*(-1.0) should be +1.0,
        // but +1.0 cannot be represented in Q15. Saturate to 0x7FFF.
        if (a == 16'h8000 && b == 16'h8000) begin
            out      = 16'h7FFF;
            overflow = 1'b1;
        end
        // Normal range: shifted value should fit in 16 bits signed
        else if (shifted > 32'sh00007FFF) begin
            out      = 16'h7FFF;
            overflow = 1'b1;
        end else if (shifted < 32'shFFFF8000) begin
            out      = 16'h8000;
            overflow = 1'b1;
        end else begin
            out      = shifted[15:0];
            overflow = 1'b0;
        end
    end

endmodule


// -----------------------------------------------------------------------------
// Module: sat_mul_gain
// Purpose: Signed Q15 × Q8.8 multiply. Used for:
//            error × Kp, error × Ki, error × Kd
//          where error is Q15 (16-bit) and gain is Q8.8 (16-bit).
//
// Math:
//   Q15 × Q8.8 → Q9.23 (32-bit intermediate)
//   To recover Q15, right-shift by 8 bits (the fractional bits of the gain).
//   Result is then clamped to 16-bit Q15 range.
//
// This gives gains a physical range of -128.0 to +127.996 which is sufficient
// for typical flight PID gains when error is normalized to ±1.0.
//
// Inputs:
//   data [15:0]  - signed Q15 measurement/error value
//   gain [15:0]  - signed Q8.8 gain coefficient
//
// Output:
//   out  [15:0]  - signed Q15 result (gain × data, saturated)
//   overflow     - 1 if saturation was applied
// -----------------------------------------------------------------------------
module sat_mul_gain (
    input  wire [15:0] data,   // signed Q15
    input  wire [15:0] gain,   // signed Q8.8
    output reg  [15:0] out,    // signed Q15 result
    output reg         overflow
);

    // 32-bit intermediate: Q15 × Q8.8 = Q9.23
    wire signed [31:0] product = $signed(data) * $signed(gain);

    // Shift right by 8 to align fractional point back to Q15.
    // After shift: Q9.15 in a 32-bit container.
    wire signed [31:0] shifted = product >>> 8;

    always @(*) begin
        if (shifted > 32'sh00007FFF) begin
            out      = 16'h7FFF;
            overflow = 1'b1;
        end else if (shifted < 32'shFFFF8000) begin
            out      = 16'h8000;
            overflow = 1'b1;
        end else begin
            out      = shifted[15:0];
            overflow = 1'b0;
        end
    end

endmodule
