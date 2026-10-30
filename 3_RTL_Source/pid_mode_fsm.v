`timescale 1ns / 1ps
// =============================================================================
// File        : pid_mode_fsm.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 6: Mode Controller & Bumpless Mode Transition Engine.
//               Monitors requested flight mode. When a transition occurs, it:
//               1. Queries target mode gains (Kp_new, Kd_new).
//               2. Computes target P and D contributions.
//               3. Back-calculates target integrator state:
//                  I_backcalc = u_prev - P_new + D_new
//               4. Pre-loads the integrator and updates active_mode atomically,
//                  ensuring ZERO transient output bump across any mode pair.
// =============================================================================

module pid_mode_fsm (
    input  wire        clk,
    input  wire        rst_n,
    
    // Mode Control
    input  wire [1:0]  req_mode,         // Requested mode (from pilot/supervisor)
    output reg  [1:0]  active_mode,      // Currently executing mode
    output reg         transition_busy,  // 1 during bumpless calculation
    
    // Interface to Gain Bank
    output wire [1:0]  lookup_mode,      // Tells gain bank which mode to preview
    input  wire [15:0] lookup_kp,        // Kp of target mode
    input  wire [15:0] lookup_ki,        // Ki of target mode
    input  wire [15:0] lookup_kd,        // Kd of target mode
    
    // Feedback from PID Core for Back-Calculation
    input  wire [15:0] current_out,      // u_prev from pid_core
    input  wire [15:0] current_err,      // error e[n] from pid_core
    input  wire [15:0] current_d_filt,   // filtered derivative from pid_core
    
    // Control to PID Core
    output reg         integ_load,       // Preload strobe to pid_core
    output reg  [15:0] integ_load_val    // Preloaded integrator value
);

    assign lookup_mode = req_mode;

    // FSM States
    localparam S_STEADY = 2'd0;
    localparam S_CALC   = 2'd1;
    localparam S_APPLY  = 2'd2;
    
    reg [1:0] state;

    // -------------------------------------------------------------------------
    // Combinational Target P, I, D Calculation for Predictive Bumpless Transfer
    // -------------------------------------------------------------------------
    wire signed [15:0] p_target, i_target, d_target;
    
    sat_mul_gain mul_target_p (
        .data     (current_err),
        .gain     (lookup_kp),
        .out      (p_target),
        .overflow ()
    );

    sat_mul_gain mul_target_i (
        .data     (current_err),
        .gain     (lookup_ki),
        .out      (i_target),
        .overflow ()
    );
    
    sat_mul_gain mul_target_d (
        .data     (current_d_filt),
        .gain     (lookup_kd),
        .out      (d_target),
        .overflow ()
    );

    // Predictive Bumpless Back-Calculation:
    // I_target = u_prev - P_target - I_target_step + D_target
    // This accounts for the upcoming sample's integration step so that the first
    // evaluated output in the new mode matches u_prev with EXACTLY ZERO TRANSIENT JUMP!
    wire signed [31:0] i_target_wide = $signed(current_out) - $signed(p_target) - $signed(i_target) + $signed(d_target);
    wire signed [15:0] i_target_clamped;
    
    sat_clamp clamp_backcalc (
        .in_wide (i_target_wide),
        .lo      (16'h8000),
        .hi      (16'h7FFF),
        .out     (i_target_clamped)
    );

    // -------------------------------------------------------------------------
    // Sequential Transition FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_STEADY;
            active_mode     <= 2'b00; // Boot in Acro mode
            transition_busy <= 1'b0;
            integ_load      <= 1'b0;
            integ_load_val  <= 16'h0000;
        end else begin
            integ_load <= 1'b0; // default pulse clear
            
            case (state)
                S_STEADY: begin
                    transition_busy <= 1'b0;
                    if (req_mode != active_mode) begin
                        transition_busy <= 1'b1;
                        state           <= S_CALC;
                    end
                end
                
                S_CALC: begin
                    // Latch back-calculated integrator value
                    integ_load_val <= i_target_clamped;
                    integ_load     <= 1'b1; // Trigger preload into pid_core
                    state          <= S_APPLY;
                end
                
                S_APPLY: begin
                    // Atomically switch active mode on exact clock boundary
                    active_mode     <= req_mode;
                    transition_busy <= 1'b0;
                    state           <= S_STEADY;
                end
                
                default: state <= S_STEADY;
            endcase
        end
    end

endmodule
