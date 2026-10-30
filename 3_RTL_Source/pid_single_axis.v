`timescale 1ns / 1ps
// =============================================================================
// File        : pid_single_axis.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Single-Axis Flight Controller Channel.
//               Integrates:
//               - pid_gain_bank (Per-mode gains & limits)
//               - pid_mode_fsm  (Bumpless transition engine)
//               - pid_core      (Discrete PID with anti-windup & filtered D)
// =============================================================================

module pid_single_axis (
    input  wire        clk,
    input  wire        rst_n,
    
    // Computation trigger
    input  wire        step,
    
    // Flight Mode Control
    input  wire [1:0]  req_mode,
    output wire [1:0]  active_mode,
    output wire        transition_busy,
    
    // Control I/O
    input  wire [15:0] setpoint,
    input  wire [15:0] measurement,
    output wire [15:0] out,
    output wire        done,
    
    // Host / SPI Register Tuning
    input  wire        reg_wr_en,
    input  wire [4:0]  reg_wr_addr,
    input  wire [15:0] reg_wr_data,
    
    // Telemetry Outputs
    output wire [15:0] integ_state,
    output wire [15:0] err_state,
    output wire [15:0] d_filt_state
);

    // Gain bank interconnects
    wire [1:0]  lookup_mode;
    wire [15:0] lookup_kp, lookup_ki, lookup_kd;
    
    wire [15:0] active_kp, active_ki, active_kd;
    wire [15:0] active_d_filter_coeff;
    wire [15:0] active_integ_max, active_integ_min;
    wire [15:0] active_out_max, active_out_min;
    
    // Mode FSM interconnects
    wire        integ_load;
    wire [15:0] integ_load_val;

    // 1. Gain Bank Instance
    pid_gain_bank u_gain_bank (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .active_mode           (active_mode),
        .lookup_mode           (lookup_mode),
        .wr_en                 (reg_wr_en),
        .wr_addr               (reg_wr_addr),
        .wr_data               (reg_wr_data),
        .active_kp             (active_kp),
        .active_ki             (active_ki),
        .active_kd             (active_kd),
        .active_d_filter_coeff (active_d_filter_coeff),
        .active_integ_max      (active_integ_max),
        .active_integ_min      (active_integ_min),
        .active_out_max        (active_out_max),
        .active_out_min        (active_out_min),
        .lookup_kp             (lookup_kp),
        .lookup_ki             (lookup_ki),
        .lookup_kd             (lookup_kd)
    );

    // 2. Mode FSM Instance (Bumpless Engine)
    pid_mode_fsm u_mode_fsm (
        .clk             (clk),
        .rst_n           (rst_n),
        .req_mode        (req_mode),
        .active_mode     (active_mode),
        .transition_busy (transition_busy),
        .lookup_mode     (lookup_mode),
        .lookup_kp       (lookup_kp),
        .lookup_ki       (lookup_ki),
        .lookup_kd       (lookup_kd),
        .current_out     (out),
        .current_err     (err_state),
        .current_d_filt  (d_filt_state),
        .integ_load      (integ_load),
        .integ_load_val  (integ_load_val)
    );

    // 3. PID Core Instance
    pid_core u_pid_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .step           (step),
        .setpoint       (setpoint),
        .measurement    (measurement),
        .kp             (active_kp),
        .ki             (active_ki),
        .kd             (active_kd),
        .d_filter_coeff (active_d_filter_coeff),
        .integ_max      (active_integ_max),
        .integ_min      (active_integ_min),
        .out_max        (active_out_max),
        .out_min        (active_out_min),
        .integ_load     (integ_load),
        .integ_load_val (integ_load_val),
        .out            (out),
        .integ_state    (integ_state),
        .err_state      (err_state),
        .d_filt_state   (d_filt_state),
        .done           (done)
    );

endmodule
