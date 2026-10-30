`timescale 1ns / 1ps
// =============================================================================
// File        : pid_cascaded_axis.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 7 & 8: Complete Single-Axis Cascaded Flight Control Channel.
//               Integrates:
//               - Outer Attitude PID Loop (100 Hz): Angle error -> Target Rate
//               - Acro Mode Bypass MUX: Stick commands rate directly in Acro
//               - Inner Rate PID Channel (1 kHz): Rate error -> Motor command
//               - Sensor Watchdog: Detects stale data & forces Failsafe
//               - Failsafe Output Ramp: Controlled actuator descent
// =============================================================================

module pid_cascaded_axis (
    input  wire        clk,
    input  wire        rst_n,
    
    // Multi-rate execution triggers
    input  wire        step_rate,          // 1 kHz inner loop trigger
    input  wire        step_att,           // 100 Hz outer loop trigger
    
    // Pilot / Supervisor Control
    input  wire [1:0]  pilot_mode,         // 00: Acro, 01: Stabilize, 10: AltHold, 11: Failsafe
    output wire [1:0]  active_mode,        // Effective mode (forced to 11 if watchdog trips)
    output wire        watchdog_tripped,
    
    // Sensor Health & Failsafe Configuration
    input  wire        sensor_fresh,       // IMU data-ready strobe
    input  wire [31:0] watchdog_timeout,   // Cycles until stale data fault
    input  wire [15:0] safe_target,        // Safe actuator target (Q15)
    input  wire [15:0] ramp_rate,          // Ramp step per rate sample (Q15)
    
    // Pilot Setpoints
    input  wire [15:0] stick_angle_sp,     // Attitude setpoint (used in Stabilize/AltHold)
    input  wire [15:0] stick_rate_sp,      // Angular rate setpoint (used directly in Acro)
    
    // IMU Measurements
    input  wire [15:0] angle_meas,         // Attitude angle feedback (Q15)
    input  wire [15:0] rate_meas,          // Gyro angular rate feedback (Q15)
    
    // Outer Attitude Loop Gains (Configurable)
    input  wire [15:0] att_kp,             // Outer loop proportional gain (Q8.8)
    input  wire [15:0] att_ki,             // Outer loop integral gain (Q8.8)
    
    // Register Tuning Interface for Inner Loop Gain Bank
    input  wire        reg_wr_en,
    input  wire [4:0]  reg_wr_addr,
    input  wire [15:0] reg_wr_data,
    
    // Actuator Command Output (toward PWM / motor mixer)
    output wire [15:0] actuator_out,
    output wire        done_rate,
    output wire        done_att
);

    // -------------------------------------------------------------------------
    // 1. Sensor Health Watchdog
    // -------------------------------------------------------------------------
    pid_watchdog u_watchdog (
        .clk              (clk),
        .rst_n            (rst_n),
        .sensor_fresh     (sensor_fresh),
        .timeout_cycles   (watchdog_timeout),
        .watchdog_tripped (watchdog_tripped)
    );

    // If watchdog trips, force mode to FAILSAFE (2'b11)
    wire [1:0] effective_req_mode = (watchdog_tripped) ? 2'b11 : pilot_mode;

    // -------------------------------------------------------------------------
    // 2. Outer Attitude PID Loop (100 Hz)
    //    Calculates angle error and produces the target angular rate setpoint.
    // -------------------------------------------------------------------------
    wire [15:0] outer_rate_cmd;
    
    pid_core u_outer_attitude_pid (
        .clk            (clk),
        .rst_n          (rst_n),
        .step           (step_att),
        .setpoint       (stick_angle_sp),
        .measurement    (angle_meas),
        .kp             (att_kp),
        .ki             (att_ki),
        .kd             (16'h0000),      // Outer loop typically uses P/PI
        .d_filter_coeff (16'h7FFF),
        .integ_max      (16'h3000),
        .integ_min      (16'hD000),
        .out_max        (16'h7FFF),
        .out_min        (16'h8000),
        .integ_load     (1'b0),
        .integ_load_val (16'h0000),
        .out            (outer_rate_cmd),
        .integ_state    (),
        .err_state      (),
        .d_filt_state   (),
        .done           (done_att)
    );

    // -------------------------------------------------------------------------
    // 3. Rate Setpoint MUX (Acro Mode Bypass)
    //    - In ACRO (2'b00): Outer loop bypassed; stick directly drives rate setpoint.
    //    - In STABILIZE/ALT_HOLD: Outer attitude PID drives rate setpoint.
    //    - In FAILSAFE (2'b11): Hold zero angular rate setpoint for level stabilization.
    // -------------------------------------------------------------------------
    reg [15:0] inner_rate_sp;
    always @(*) begin
        if (effective_req_mode == 2'b00) begin
            inner_rate_sp = stick_rate_sp;     // Acro: direct manual rate
        end else if (effective_req_mode == 2'b11) begin
            inner_rate_sp = 16'h0000;          // Failsafe: command 0 deg/s rotation
        end else begin
            inner_rate_sp = outer_rate_cmd;    // Stabilize/AltHold: cascaded attitude
        end
    end

    // -------------------------------------------------------------------------
    // 4. Inner Rate PID Channel (1 kHz)
    //    Complete single-axis channel with 4-mode gain banks and bumpless transfer.
    // -------------------------------------------------------------------------
    wire [15:0] raw_rate_pid_out;
    
    pid_single_axis u_inner_rate_channel (
        .clk             (clk),
        .rst_n           (rst_n),
        .step            (step_rate),
        .req_mode        (effective_req_mode),
        .active_mode     (active_mode),
        .transition_busy (),
        .setpoint        (inner_rate_sp),
        .measurement     (rate_meas),
        .out             (raw_rate_pid_out),
        .done            (done_rate),
        .reg_wr_en       (reg_wr_en),
        .reg_wr_addr     (reg_wr_addr),
        .reg_wr_data     (reg_wr_data),
        .integ_state     (),
        .err_state       (),
        .d_filt_state    ()
    );

    // -------------------------------------------------------------------------
    // 5. Failsafe Output Ramp Generator
    // -------------------------------------------------------------------------
    wire failsafe_active = (active_mode == 2'b11);
    
    pid_failsafe_ramp u_failsafe_ramp (
        .clk             (clk),
        .rst_n           (rst_n),
        .step            (step_rate),
        .failsafe_active (failsafe_active),
        .raw_pid_out     (raw_rate_pid_out),
        .safe_target     (safe_target),
        .ramp_rate       (ramp_rate),
        .actuator_out    (actuator_out)
    );

endmodule
