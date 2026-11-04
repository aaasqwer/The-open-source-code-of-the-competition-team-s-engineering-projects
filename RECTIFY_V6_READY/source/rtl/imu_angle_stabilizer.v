`timescale 1ns/1ps
// Gentle jitter suppression only in the image compensation path.
// 16-bit circular angle: 65536 counts/turn; fractional state retains 8 bits.
// Fixed 1 ms updates avoid depending on the IMU report rate.
// IIR gain 1/16 in adaptive quiet mode, 1/8 otherwise.
// Motion lag cap is 24 counts; adaptive quiet cap is 96 counts.
// Both paths use 2048-phase Schmitt quantization.
module imu_angle_stabilizer #(
 parameter integer CLK_HZ=27000000,TICK_HZ=1000,
 parameter integer MAX_LAG_COUNTS=24,HYSTERESIS_COUNTS=8,
 parameter integer ADAPTIVE_ENABLE=0,QUIET_WINDOW_COUNTS=128,
 parameter integer QUIET_MAX_LAG_COUNTS=96,QUIET_HYSTERESIS_COUNTS=12
)(
 input clk,rst_n,sample_valid,input signed [15:0] angle_in,
 output reg signed [15:0] angle_out
);
 localparam integer TICK_CYCLES=CLK_HZ/TICK_HZ;
 localparam signed [24:0] LAG_Q8=MAX_LAG_COUNTS*256;
 localparam signed [24:0] HOLD_Q8=(16+HYSTERESIS_COUNTS)*256;
 reg [31:0] tick_count;
 reg have_sample;
 reg signed [15:0] target;
 reg signed [23:0] filtered_q8;
 wire signed [15:0] target_now=sample_valid?angle_in:target;
 wire signed [23:0] target_q8={target_now,8'd0};
 // Subtract at circular width BEFORE extending; +/-180 crossings remain small.
 wire signed [23:0] delta_q8=target_q8-filtered_q8;
 wire signed [24:0] delta_ext={delta_q8[23],delta_q8};
 wire signed [24:0] magnitude=delta_ext[24]?-delta_ext:delta_ext;
 wire quiet_mode=ADAPTIVE_ENABLE && magnitude<=QUIET_WINDOW_COUNTS*256;
 wire signed [24:0] active_lag_q8=quiet_mode?QUIET_MAX_LAG_COUNTS*256:LAG_Q8;
 wire signed [24:0] active_hold_q8=quiet_mode?(16+QUIET_HYSTERESIS_COUNTS)*256:HOLD_Q8;
 wire signed [24:0] step_magnitude=quiet_mode?((magnitude+25'sd8)>>>4):((magnitude+25'sd4)>>>3);
 wire signed [24:0] step_q8=delta_ext[24]?-step_magnitude:step_magnitude;
 wire signed [23:0] candidate_q8=filtered_q8+step_q8;
 wire signed [23:0] remaining_q8=target_q8-candidate_q8;
 wire signed [24:0] remaining_ext={remaining_q8[23],remaining_q8};
 reg signed [23:0] next_q8;
 wire signed [23:0] phase_error_q8=next_q8-{angle_out,8'd0};
 wire signed [24:0] phase_error_ext={phase_error_q8[23],phase_error_q8};
 wire [23:0] rounded_q8=next_q8+24'd4096;
 wire [15:0] initial_rounded=angle_in+16'd16;
 always @* begin
  if(remaining_ext>active_lag_q8) next_q8=target_q8-active_lag_q8;
  else if(remaining_ext< -active_lag_q8) next_q8=target_q8+active_lag_q8;
  else next_q8=candidate_q8;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin tick_count<=0;have_sample<=0;target<=0;filtered_q8<=0;angle_out<=0;end
  else begin
   if(tick_count==TICK_CYCLES-1) tick_count<=0;else tick_count<=tick_count+1'b1;
   if(sample_valid) begin
    target<=angle_in;
    if(!have_sample) begin
     have_sample<=1;filtered_q8<={angle_in,8'd0};
     angle_out<={initial_rounded[15:5],5'd0};
    end
   end
   if(have_sample && tick_count==TICK_CYCLES-1) begin
    filtered_q8<=next_q8;
    // Hold until the active threshold is exceeded (24 motion, 28 quiet).
    // A larger move selects the nearest bin directly: no slew-rate limit.
    if(phase_error_ext>active_hold_q8 || phase_error_ext< -active_hold_q8)
     angle_out<={rounded_q8[23:13],5'd0};
   end
  end
 end
endmodule
