`timescale 1ns / 1ps
// =============================================================================
// File        : pid_controller.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 9: Complete 3-Axis + Altitude Flight Controller Top Level.
//               Architecture:
//               - Roll, Pitch, Yaw parallel cascaded control engines
//               - Altitude / Vertical-velocity channel (active in AltHold)
//               - 50 MHz -> 1 kHz (inner) & 100 Hz (outer) hardware rate scheduler
//               - 4 Flight Modes: Acro, Stabilize, AltHold, Failsafe
//               - Predictive bumpless mode switching (0 LSB output jump)
//               - Anti-windup integrator clamping & low-pass filtered D
//               - Sensor health watchdog & controlled failsafe ramp
//               - Host register programming interface for live gain tuning
// =============================================================================

module pid_controller (
    input  wire        clk,                 // 50 MHz master clock
    input  wire        rst_n,               // Asynchronous active-low reset
    
    // Simulation / Clocking Control
    input  wire        sim_fast_ticks,      // 1: Direct fast step pulses; 0: Real 50MHz clock dividers
    input  wire        ext_step_rate,       // Used if sim_fast_ticks == 1
    input  wire        ext_step_att,        // Used if sim_fast_ticks == 1
    
    // Pilot / Flight Supervisor Interface
    input  wire [1:0]  pilot_mode,          // 00: Acro, 01: Stabilize, 10: AltHold, 11: Failsafe
    output wire [1:0]  active_mode,         // Effective operating mode
    output wire        watchdog_tripped,    // Master sensor dropout alarm
    
    // Pilot Stick Inputs (Q15 normalized [-1.0, +1.0])
    input  wire [15:0] stick_roll,          // Roll angle/rate command
    input  wire [15:0] stick_pitch,         // Pitch angle/rate command
    input  wire [15:0] stick_yaw,           // Yaw rate command
    input  wire [15:0] stick_thrust,        // Baseline throttle / altitude setpoint
    
    // IMU & Altitude Sensor Inputs (Q15 normalized)
    input  wire        imu_valid,           // Fresh IMU sample strobe
    input  wire [15:0] roll_angle_meas,     // Roll attitude angle feedback
    input  wire [15:0] roll_rate_meas,      // Roll gyro angular rate feedback
    input  wire [15:0] pitch_angle_meas,    // Pitch attitude angle feedback
    input  wire [15:0] pitch_rate_meas,     // Pitch gyro angular rate feedback
    input  wire [15:0] yaw_rate_meas,       // Yaw gyro angular rate feedback
    input  wire [15:0] alt_meas,            // Baro/laser altitude feedback
    
    // Watchdog & Failsafe Configuration
    input  wire [31:0] watchdog_timeout,    // Clock cycles before stale-data trip
    input  wire [15:0] safe_thrust_target,  // Safe throttle trim in failsafe
    input  wire [15:0] ramp_rate,           // Descent ramp rate per sample
    
    // Host Register Interface (SPI / Memory-Mapped tuning)
    input  wire        reg_wr_en,
    input  wire [1:0]  reg_axis_sel,        // 00: Roll, 01: Pitch, 10: Yaw, 11: Alt
    input  wire [4:0]  reg_wr_addr,         // 0..31 registers per axis
    input  wire [15:0] reg_wr_data,
    
    // Outer Loop Attitude Gains (Q8.8)
    input  wire [15:0] att_kp_roll,
    input  wire [15:0] att_kp_pitch,
    input  wire [15:0] att_kp_yaw,
    
    // Actuator Command Outputs (Q15 [-1.0, +1.0] toward motor mixer / PWM)
    output wire [15:0] actuator_roll,
    output wire [15:0] actuator_pitch,
    output wire [15:0] actuator_yaw,
    output wire [15:0] actuator_thrust,
    
    // Execution Strobes
    output wire        tick_1khz,
    output wire        tick_100hz
);

    // -------------------------------------------------------------------------
    // 1. Hardware Multi-Rate Scheduler
    //    At 50 MHz:
    //    1 kHz  period = 50,000 cycles
    //    100 Hz period = 500,000 cycles (every 10 rate ticks)
    // -------------------------------------------------------------------------
    reg [15:0] count_rate;
    reg [3:0]  count_att;
    reg        hw_tick_rate;
    reg        hw_tick_att;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count_rate   <= 16'd0;
            count_att    <= 4'd0;
            hw_tick_rate <= 1'b0;
            hw_tick_att  <= 1'b0;
        end else begin
            hw_tick_rate <= 1'b0;
            hw_tick_att  <= 1'b0;
            
            if (count_rate >= 16'd49999) begin
                count_rate   <= 16'd0;
                hw_tick_rate <= 1'b1;
                
                if (count_att >= 4'd9) begin
                    count_att   <= 4'd0;
                    hw_tick_att <= 1'b1;
                end else begin
                    count_att <= count_att + 1'b1;
                end
            end else begin
                count_rate <= count_rate + 1'b1;
            end
        end
    end

    // Select hardware scheduler or testbench direct tick
    wire step_rate = (sim_fast_ticks) ? ext_step_rate : hw_tick_rate;
    wire step_att  = (sim_fast_ticks) ? ext_step_att  : hw_tick_att;

    assign tick_1khz  = step_rate;
    assign tick_100hz = step_att;

    // -------------------------------------------------------------------------
    // 2. Register Write Decoding per Axis
    // -------------------------------------------------------------------------
    wire wr_en_roll  = reg_wr_en && (reg_axis_sel == 2'b00);
    wire wr_en_pitch = reg_wr_en && (reg_axis_sel == 2'b01);
    wire wr_en_yaw   = reg_wr_en && (reg_axis_sel == 2'b10);
    wire wr_en_alt   = reg_wr_en && (reg_axis_sel == 2'b11);

    // -------------------------------------------------------------------------
    // 3. Roll Axis Channel (Cascaded Attitude -> Rate)
    // -------------------------------------------------------------------------
    wire wd_roll;
    pid_cascaded_axis u_roll_axis (
        .clk              (clk),
        .rst_n            (rst_n),
        .step_rate        (step_rate),
        .step_att         (step_att),
        .pilot_mode       (pilot_mode),
        .active_mode      (active_mode),
        .watchdog_tripped (wd_roll),
        .sensor_fresh     (imu_valid),
        .watchdog_timeout (watchdog_timeout),
        .safe_target      (16'h0000),         // 0 deg/s roll in failsafe
        .ramp_rate        (ramp_rate),
        .stick_angle_sp   (stick_roll),
        .stick_rate_sp    (stick_roll),
        .angle_meas       (roll_angle_meas),
        .rate_meas        (roll_rate_meas),
        .att_kp           (att_kp_roll),
        .att_ki           (16'h0000),
        .reg_wr_en        (wr_en_roll),
        .reg_wr_addr      (reg_wr_addr),
        .reg_wr_data      (reg_wr_data),
        .actuator_out     (actuator_roll),
        .done_rate        (),
        .done_att         ()
    );

    // -------------------------------------------------------------------------
    // 4. Pitch Axis Channel (Cascaded Attitude -> Rate)
    // -------------------------------------------------------------------------
    wire wd_pitch;
    pid_cascaded_axis u_pitch_axis (
        .clk              (clk),
        .rst_n            (rst_n),
        .step_rate        (step_rate),
        .step_att         (step_att),
        .pilot_mode       (pilot_mode),
        .active_mode      (),
        .watchdog_tripped (wd_pitch),
        .sensor_fresh     (imu_valid),
        .watchdog_timeout (watchdog_timeout),
        .safe_target      (16'h0000),         // 0 deg/s pitch in failsafe
        .ramp_rate        (ramp_rate),
        .stick_angle_sp   (stick_pitch),
        .stick_rate_sp    (stick_pitch),
        .angle_meas       (pitch_angle_meas),
        .rate_meas        (pitch_rate_meas),
        .att_kp           (att_kp_pitch),
        .att_ki           (16'h0000),
        .reg_wr_en        (wr_en_pitch),
        .reg_wr_addr      (reg_wr_addr),
        .reg_wr_data      (reg_wr_data),
        .actuator_out     (actuator_pitch),
        .done_rate        (),
        .done_att         ()
    );

    // -------------------------------------------------------------------------
    // 5. Yaw Axis Channel (Rate Controller)
    // -------------------------------------------------------------------------
    wire wd_yaw;
    pid_cascaded_axis u_yaw_axis (
        .clk              (clk),
        .rst_n            (rst_n),
        .step_rate        (step_rate),
        .step_att         (step_att),
        .pilot_mode       (pilot_mode),
        .active_mode      (),
        .watchdog_tripped (wd_yaw),
        .sensor_fresh     (imu_valid),
        .watchdog_timeout (watchdog_timeout),
        .safe_target      (16'h0000),         // 0 deg/s yaw rotation in failsafe
        .ramp_rate        (ramp_rate),
        .stick_angle_sp   (stick_yaw),
        .stick_rate_sp    (stick_yaw),
        .angle_meas       (16'h0000),
        .rate_meas        (yaw_rate_meas),
        .att_kp           (att_kp_yaw),
        .att_ki           (16'h0000),
        .reg_wr_en        (wr_en_yaw),
        .reg_wr_addr      (reg_wr_addr),
        .reg_wr_data      (reg_wr_data),
        .actuator_out     (actuator_yaw),
        .done_rate        (),
        .done_att         ()
    );

    // -------------------------------------------------------------------------
    // 6. Altitude Channel (Active in AltHold mode)
    //    In AltHold: Regulates altitude error to trim throttle.
    //    In Acro/Stabilize: Direct manual thrust pass-through.
    //    In Failsafe: Ramps thrust down to safe_thrust_target.
    // -------------------------------------------------------------------------
    wire [15:0] alt_pid_correction;
    pid_single_axis u_alt_pid (
        .clk             (clk),
        .rst_n           (rst_n),
        .step            (step_att),
        .req_mode        (pilot_mode),
        .active_mode     (),
        .transition_busy (),
        .setpoint        (stick_thrust),
        .measurement     (alt_meas),
        .out             (alt_pid_correction),
        .done            (),
        .reg_wr_en       (wr_en_alt),
        .reg_wr_addr     (reg_wr_addr),
        .reg_wr_data     (reg_wr_data),
        .integ_state     (),
        .err_state       (),
        .d_filt_state    ()
    );

    // Thrust combiner:
    // In AltHold (2'b10): thrust = manual base + altitude PID correction
    // In Acro/Stabilize: thrust = manual stick
    wire [15:0] raw_thrust = (active_mode == 2'b10) ? 
                             (stick_thrust + alt_pid_correction) : stick_thrust;

    // Failsafe ramp on thrust channel
    pid_failsafe_ramp u_thrust_failsafe_ramp (
        .clk             (clk),
        .rst_n           (rst_n),
        .step            (step_rate),
        .failsafe_active (active_mode == 2'b11),
        .raw_pid_out     (raw_thrust),
        .safe_target     (safe_thrust_target),
        .ramp_rate       (ramp_rate),
        .actuator_out    (actuator_thrust)
    );

    assign watchdog_tripped = wd_roll || wd_pitch || wd_yaw;

endmodule
