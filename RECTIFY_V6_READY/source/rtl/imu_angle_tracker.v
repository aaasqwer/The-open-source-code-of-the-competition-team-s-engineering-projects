`timescale 1ns/1ps
// Zero at the first valid attitude after reset. AXIS=3 selects and then locks
// the axis with the largest first movement (> 15 degrees); 0/1/2 force R/P/Y.
module imu_angle_tracker #(parameter integer AXIS=3)(
 input clk, rst_n, attitude_valid,
 input signed [15:0] roll_raw, pitch_raw, yaw_raw,
 output reg signed [15:0] angle_raw,
 output reg [1:0] selected_axis
);
 reg zero_valid, axis_locked;
 reg signed [15:0] r0,p0,y0;
 wire signed [15:0] dr=roll_raw-r0, dp=pitch_raw-p0, dy=yaw_raw-y0;
 wire [15:0] ar=dr[15] ? -dr : dr, ap=dp[15] ? -dp : dp, ay=dy[15] ? -dy : dy;
 reg [1:0] dominant;
 always @* begin
  if(ar>=ap && ar>=ay) dominant=0;
  else if(ap>=ay) dominant=1;
  else dominant=2;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   zero_valid<=0; axis_locked<=0; r0<=0;p0<=0;y0<=0;
   angle_raw<=0; selected_axis<=(AXIS==3)?0:AXIS;
  end else if(attitude_valid) begin
   if(!zero_valid) begin
    r0<=roll_raw;p0<=pitch_raw;y0<=yaw_raw;zero_valid<=1;angle_raw<=0;
   end else begin
    if(AXIS==3 && !axis_locked) begin
     selected_axis<=dominant;
     if(ar>2731 || ap>2731 || ay>2731) axis_locked<=1;
     case(dominant) 0:angle_raw<=dr;1:angle_raw<=dp;default:angle_raw<=dy;endcase
    end else case(selected_axis) 0:angle_raw<=dr;1:angle_raw<=dp;default:angle_raw<=dy;endcase
   end
  end
 end
endmodule
