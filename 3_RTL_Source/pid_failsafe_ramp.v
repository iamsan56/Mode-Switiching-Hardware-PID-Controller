`timescale 1ns / 1ps
// =============================================================================
// File        : pid_failsafe_ramp.v
// Project     : PS-02 Mode-Switching Hardware PID Controller
// Description : Stage 8: Safe Output Ramp Generator.
//               When Failsafe mode is triggered, it freezes the last valid output
//               and smoothly ramps it toward a programmable safe_target value
//               (e.g., zero thrust or hover trim) at a programmable ramp_rate,
//               preventing abrupt actuator cuts or violent drops.
// =============================================================================

module pid_failsafe_ramp (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        step,             // Rate loop update tick
    
    // Mode status
    input  wire        failsafe_active,  // 1 when controller is in Failsafe
    
    // Inputs
    input  wire [15:0] raw_pid_out,      // Live PID output from rate controller (Q15)
    input  wire [15:0] safe_target,      // Safe target level (e.g. 16'h0000)
    input  wire [15:0] ramp_rate,        // Step change per rate-loop sample tick
    
    // Output toward actuator / PWM mixer
    output reg  [15:0] actuator_out
);

    reg signed [15:0] ramp_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ramp_reg     <= 16'h0000;
            actuator_out <= 16'h0000;
        end else begin
            if (!failsafe_active) begin
                // Normal flight: pass PID output directly, track for instant freeze
                ramp_reg     <= raw_pid_out;
                actuator_out <= raw_pid_out;
            end else begin
                // Failsafe flight: ramp smoothly toward safe_target
                if (step) begin
                    if ($signed(ramp_reg) > $signed(safe_target)) begin
                        if ($signed(ramp_reg) - $signed(safe_target) > $signed(ramp_rate))
                            ramp_reg <= ramp_reg - ramp_rate;
                        else
                            ramp_reg <= safe_target;
                    end else if ($signed(ramp_reg) < $signed(safe_target)) begin
                        if ($signed(safe_target) - $signed(ramp_reg) > $signed(ramp_rate))
                            ramp_reg <= ramp_reg + ramp_rate;
                        else
                            ramp_reg <= safe_target;
                    end
                end
                actuator_out <= ramp_reg;
            end
        end
    end

endmodule
