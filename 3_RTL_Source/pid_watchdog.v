`timescale 1ns / 1ps
// =============================================================================
// File        : pid_watchdog.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 8: Sensor Health Watchdog.
//               Monitors sensor_fresh pulses from the IMU.
//               If no fresh sample arrives within programmable timeout_cycles,
//               it trips watchdog_tripped and forces Failsafe mode.
//               Automatically recovers when valid data resumes.
// =============================================================================

module pid_watchdog (
    input  wire        clk,
    input  wire        rst_n,
    
    // Sensor status
    input  wire        sensor_fresh,    // 1-cycle pulse whenever new IMU sample arrives
    input  wire [31:0] timeout_cycles,  // Programmable threshold (e.g. 5ms = 250,000 cycles @ 50MHz)
    
    // Output fault status
    output reg         watchdog_tripped // 1 if sensor is stale, forcing Failsafe
);

    reg [31:0] timer_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer_count      <= 32'd0;
            watchdog_tripped <= 1'b0;
        end else begin
            if (sensor_fresh) begin
                // Reset timer whenever a valid, fresh sensor sample arrives
                timer_count      <= 32'd0;
                watchdog_tripped <= 1'b0;
            end else begin
                if (timer_count >= timeout_cycles) begin
                    // Threshold exceeded: force failsafe
                    watchdog_tripped <= 1'b1;
                end else begin
                    timer_count <= timer_count + 1'b1;
                end
            end
        end
    end

endmodule
