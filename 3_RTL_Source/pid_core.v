`timescale 1ns / 1ps
// =============================================================================
// File        : pid_core.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 4: Single-axis PID core with:
//               - Low-pass filtered derivative on measurement (no setpoint kick)
//               - Configurable filter coefficient alpha (Q15) with rounding & bypass
//               - Anti-windup integrator clamping
//               - Output saturation
//               - Integrator preload interface for bumpless mode transfer
// Latency     : Fixed 4 clock cycles from 'step' assertion to 'done' pulse.
// =============================================================================

module pid_core (
    input  wire        clk,
    input  wire        rst_n,
    
    // Control interface
    input  wire        step,            // 1-cycle pulse to compute new output
    input  wire [15:0] setpoint,        // Q15
    input  wire [15:0] measurement,     // Q15
    
    // Gain & Limit Configuration
    input  wire [15:0] kp,              // Q8.8
    input  wire [15:0] ki,              // Q8.8
    input  wire [15:0] kd,              // Q8.8
    input  wire [15:0] d_filter_coeff,  // Q15 filter alpha (0x7FFF = pass-through bypass)
    input  wire [15:0] integ_max,       // Q15 - Integrator clamp high
    input  wire [15:0] integ_min,       // Q15 - Integrator clamp low
    input  wire [15:0] out_max,         // Q15 - Output saturation high
    input  wire [15:0] out_min,         // Q15 - Output saturation low
    
    // Bumpless Transfer Integrator Preload
    input  wire        integ_load,      // 1 to force-load integrator state
    input  wire [15:0] integ_load_val,  // Value to load (Q15)
    
    // Outputs
    output reg  [15:0] out,             // Q15
    output wire [15:0] integ_state,     // Q15 (exposed for bumpless back-calc)
    output wire [15:0] err_state,       // Q15 (exposed for telemetry & back-calc)
    output wire [15:0] d_filt_state,    // Q15 (exposed for telemetry & back-calc)
    output reg         done             // 1-cycle pulse when 'out' is valid
);

    // -------------------------------------------------------------------------
    // FSM State Definitions
    // -------------------------------------------------------------------------
    localparam S_IDLE   = 2'd0;
    localparam S_FILTER = 2'd1;
    localparam S_MULT   = 2'd2;
    localparam S_SUM    = 2'd3;
    
    reg [1:0] state;

    // -------------------------------------------------------------------------
    // Internal Registers
    // -------------------------------------------------------------------------
    reg signed [15:0] prev_meas;
    reg signed [15:0] err_reg;
    reg signed [15:0] d_raw_reg;
    reg signed [15:0] d_filt_reg;
    
    reg signed [15:0] p_term_reg;
    reg signed [15:0] i_term_reg;
    reg signed [15:0] d_term_reg;
    
    reg signed [15:0] integ_reg;

    assign integ_state  = integ_reg;
    assign err_state    = err_reg;
    assign d_filt_state = d_filt_reg;

    // -------------------------------------------------------------------------
    // Combinational Math Blocks
    // -------------------------------------------------------------------------
    
    // 1. Error and Raw Derivative Subtraction
    wire signed [16:0] raw_err   = $signed({setpoint[15], setpoint}) - $signed({measurement[15], measurement});
    wire signed [16:0] raw_d_raw = $signed({measurement[15], measurement}) - $signed({prev_meas[15], prev_meas});
    
    // 2. Low-Pass Derivative Filter Math:
    //    d_filt_next = d_filt_reg + alpha * (d_raw_reg - d_filt_reg)
    wire signed [16:0] filt_diff = $signed({d_raw_reg[15], d_raw_reg}) - $signed({d_filt_reg[15], d_filt_reg});
    
    // Add rounding bias (0x4000 = 0.5 LSB) before arithmetic shift to avoid truncation loss
    wire signed [31:0] filt_prod         = $signed(filt_diff) * $signed({1'b0, d_filter_coeff[14:0]});
    wire signed [31:0] filt_prod_rounded = filt_prod + 32'sh0000_4000;
    wire signed [15:0] filt_inc          = filt_prod_rounded >>> 15;
    
    wire [15:0] d_filt_calc;
    sat_add16 add_filter (
        .a        (d_filt_reg),
        .b        (filt_inc),
        .out      (d_filt_calc),
        .overflow ()
    );

    // Bypass check: 0x7FFF means 100% pass-through with no filtering delay/truncation
    wire [15:0] d_filt_next = (d_filter_coeff == 16'h7FFF) ? d_raw_reg : d_filt_calc;

    // 3. Multipliers (Q15 x Q8.8 -> Q15)
    wire signed [15:0] p_term_comb, i_term_comb, d_term_comb;
    
    sat_mul_gain mul_p ( .data(err_reg),    .gain(kp), .out(p_term_comb), .overflow() );
    sat_mul_gain mul_i ( .data(err_reg),    .gain(ki), .out(i_term_comb), .overflow() );
    sat_mul_gain mul_d ( .data(d_filt_reg), .gain(kd), .out(d_term_comb), .overflow() );

    // 4. Integrator Preload Clamping
    wire [15:0] integ_load_clamped;
    sat_clamp clamp_preload (
        .in_wide ({{16{integ_load_val[15]}}, integ_load_val}),
        .lo      (integ_min),
        .hi      (integ_max),
        .out     (integ_load_clamped)
    );

    // 5. Integrator Addition and Clamping (Anti-Windup)
    wire signed [31:0] integ_next_wide = $signed(integ_reg) + $signed(i_term_reg);
    wire signed [15:0] integ_next_clamped;
    
    sat_clamp clamp_integ (
        .in_wide (integ_next_wide),
        .lo      (integ_min),
        .hi      (integ_max),
        .out     (integ_next_clamped)
    );

    // 6. Output Summation and Saturation: Out = P + I - D
    wire signed [31:0] out_wide = $signed(p_term_reg) + $signed(integ_next_clamped) - $signed(d_term_reg);
    wire signed [15:0] out_clamped;
    
    sat_clamp clamp_out (
        .in_wide (out_wide),
        .lo      (out_min),
        .hi      (out_max),
        .out     (out_clamped)
    );

    // -------------------------------------------------------------------------
    // Sequential Logic (FSM & Registers)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            prev_meas  <= 16'h0000;
            err_reg    <= 16'h0000;
            d_raw_reg  <= 16'h0000;
            d_filt_reg <= 16'h0000;
            p_term_reg <= 16'h0000;
            i_term_reg <= 16'h0000;
            d_term_reg <= 16'h0000;
            integ_reg  <= 16'h0000;
            out        <= 16'h0000;
            done       <= 1'b0;
        end else begin
            done <= 1'b0;
            
            // Integrator preload has priority
            if (integ_load) begin
                integ_reg <= integ_load_clamped;
            end
            
            case (state)
                S_IDLE: begin
                    if (step) begin
                        // Clamp error to Q15 range
                        if (raw_err > 17'sh07FFF)       err_reg <= 16'h7FFF;
                        else if (raw_err < 17'sh18000)  err_reg <= 16'h8000;
                        else                            err_reg <= raw_err[15:0];
                        
                        // Clamp raw delta measurement to Q15 range
                        if (raw_d_raw > 17'sh07FFF)      d_raw_reg <= 16'h7FFF;
                        else if (raw_d_raw < 17'sh18000) d_raw_reg <= 16'h8000;
                        else                             d_raw_reg <= raw_d_raw[15:0];
                        
                        prev_meas <= measurement;
                        state     <= S_FILTER;
                    end
                end
                
                S_FILTER: begin
                    // Update low-pass filtered derivative state
                    d_filt_reg <= d_filt_next;
                    state      <= S_MULT;
                end
                
                S_MULT: begin
                    p_term_reg <= p_term_comb;
                    i_term_reg <= i_term_comb;
                    d_term_reg <= d_term_comb;
                    state      <= S_SUM;
                end
                
                S_SUM: begin
                    // If not currently preloading, accumulate normally
                    if (!integ_load) begin
                        integ_reg <= integ_next_clamped;
                    end
                    out   <= out_clamped;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
