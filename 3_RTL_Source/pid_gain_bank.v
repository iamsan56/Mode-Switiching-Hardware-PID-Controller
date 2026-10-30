`timescale 1ns / 1ps
// =============================================================================
// File        : pid_gain_bank.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 5: Per-mode gain and parameter bank.
//               Stores Kp, Ki, Kd, d_filter_coeff, integ_limits, out_limits
//               for all 4 flight modes (Acro, Stabilize, AltHold, Failsafe).
//               Supports real-time register writes and atomic mode readouts.
// =============================================================================

module pid_gain_bank (
    input  wire        clk,
    input  wire        rst_n,
    
    // Mode Selection
    input  wire [1:0]  active_mode,      // Current operating mode
    input  wire [1:0]  lookup_mode,      // Mode to preview for back-calculation
    
    // Register Write Interface (for host/SPI tuning)
    input  wire        wr_en,
    input  wire [4:0]  wr_addr,          // 32 configuration registers
    input  wire [15:0] wr_data,
    
    // Active Mode Outputs (routed directly to pid_core)
    output reg  [15:0] active_kp,
    output reg  [15:0] active_ki,
    output reg  [15:0] active_kd,
    output reg  [15:0] active_d_filter_coeff,
    output reg  [15:0] active_integ_max,
    output reg  [15:0] active_integ_min,
    output reg  [15:0] active_out_max,
    output reg  [15:0] active_out_min,
    
    // Lookup Outputs (target mode gains for bumpless back-calculation)
    output reg  [15:0] lookup_kp,
    output reg  [15:0] lookup_ki,
    output reg  [15:0] lookup_kd
);

    // Mode encodings
    localparam MODE_ACRO      = 2'b00;
    localparam MODE_STABILIZE = 2'b01;
    localparam MODE_ALT_HOLD  = 2'b10;
    localparam MODE_FAILSAFE  = 2'b11;

    // Register storage: 4 modes * 8 parameters = 32 registers
    reg [15:0] reg_file [0:31];

    // Address mapping helper: {mode[1:0], param[2:0]}
    // param 0: kp
    // param 1: ki
    // param 2: kd
    // param 3: d_filter_coeff
    // param 4: integ_max
    // param 5: integ_min
    // param 6: out_max
    // param 7: out_min

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // -----------------------------------------------------------------
            // Default Flight Presets (realistic normalized drone values)
            // -----------------------------------------------------------------
            
            // Mode 0: ACRO (Aggressive rate response, low I, high D filter cutoff)
            reg_file[0] <= 16'h0100; // Kp = 1.0 (Q8.8)
            reg_file[1] <= 16'h0040; // Ki = 0.25 (Q8.8)
            reg_file[2] <= 16'h0080; // Kd = 0.5 (Q8.8)
            reg_file[3] <= 16'h7FFF; // Filter coeff = 1.0 (raw bypass)
            reg_file[4] <= 16'h4000; // integ_max = +0.5
            reg_file[5] <= 16'hC000; // integ_min = -0.5
            reg_file[6] <= 16'h7FFF; // out_max = +1.0
            reg_file[7] <= 16'h8000; // out_min = -1.0
            
            // Mode 1: STABILIZE (Higher Kp for stiffness, higher Ki for level hold)
            reg_file[8]  <= 16'h0200; // Kp = 2.0 (Q8.8)
            reg_file[9]  <= 16'h0080; // Ki = 0.5 (Q8.8)
            reg_file[10] <= 16'h0040; // Kd = 0.25 (Q8.8)
            reg_file[11] <= 16'h4000; // Filter coeff = 0.5 (smoothed)
            reg_file[12] <= 16'h5000; // integ_max = +0.625
            reg_file[13] <= 16'hB000; // integ_min = -0.625
            reg_file[14] <= 16'h7FFF; // out_max = +1.0
            reg_file[15] <= 16'h8000; // out_min = -1.0
            
            // Mode 2: ALT_HOLD (Moderate attitude response)
            reg_file[16] <= 16'h0180; // Kp = 1.5
            reg_file[17] <= 16'h0060; // Ki = 0.375
            reg_file[18] <= 16'h0060; // Kd = 0.375
            reg_file[19] <= 16'h3000; // Filter coeff = ~0.375
            reg_file[20] <= 16'h4800; // integ_max
            reg_file[21] <= 16'hB800; // integ_min
            reg_file[22] <= 16'h7FFF; // out_max
            reg_file[23] <= 16'h8000; // out_min
            
            // Mode 3: FAILSAFE (Gentle gains, restricted limits)
            reg_file[24] <= 16'h0080; // Kp = 0.5
            reg_file[25] <= 16'h0020; // Ki = 0.125
            reg_file[26] <= 16'h0020; // Kd = 0.125
            reg_file[27] <= 16'h2000; // Filter coeff = 0.25
            reg_file[28] <= 16'h2000; // integ_max = +0.25
            reg_file[29] <= 16'hE000; // integ_min = -0.25
            reg_file[30] <= 16'h4000; // out_max = +0.5 (throttled)
            reg_file[31] <= 16'hC000; // out_min = -0.5
        end else begin
            if (wr_en) begin
                reg_file[wr_addr] <= wr_data;
            end
        end
    end

    // Combinational readout based on active_mode
    wire [4:0] base_addr = {active_mode, 3'b000};
    always @(*) begin
        active_kp             = reg_file[base_addr + 0];
        active_ki             = reg_file[base_addr + 1];
        active_kd             = reg_file[base_addr + 2];
        active_d_filter_coeff = reg_file[base_addr + 3];
        active_integ_max      = reg_file[base_addr + 4];
        active_integ_min      = reg_file[base_addr + 5];
        active_out_max        = reg_file[base_addr + 6];
        active_out_min        = reg_file[base_addr + 7];
    end

    // Lookup readout for transition back-calculation
    wire [4:0] lookup_base = {lookup_mode, 3'b000};
    always @(*) begin
        lookup_kp = reg_file[lookup_base + 0];
        lookup_ki = reg_file[lookup_base + 1];
        lookup_kd = reg_file[lookup_base + 2];
    end

endmodule
