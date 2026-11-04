`timescale 1ns/1ps
// Rigid-camera IMU planar leveling after the general host transform. Q14 c/s
// include the angle-dependent cover zoom. Center uses exact pixel centers.
module rect_imu_affine_pipe #(parameter W=1920,H=1080,TW=11)(
 input clk,rst_n,in_valid,in_bad,input signed [25:0] x,y,input [TW-1:0] in_tag,
 input signed [17:0] c,s,
 output reg out_valid,out_bad,output reg signed [25:0] ideal_x,ideal_y,output reg [TW-1:0] out_tag
);
 localparam signed [31:0] CX=(W-1)*2048,CY=(H-1)*2048;
 wire signed [26:0] dx=$signed({x[25],x})-CX,dy=$signed({y[25],y})-CY;
 reg signed [44:0] cx,sx,cy,sy;
 reg v0,bad0;reg [TW-1:0] tag0;
 wire signed [46:0] u=(($signed(cx)+$signed(sy)+47'sd8192)>>>14)+CX;
 wire signed [46:0] v=(($signed(cy)-$signed(sx)+47'sd8192)>>>14)+CY;
 always @(posedge clk)begin
  cx<=c*dx;sx<=s*dx;cy<=c*dy;sy<=s*dy;tag0<=in_tag;out_tag<=tag0;
  ideal_x<=u[25:0];ideal_y<=v[25:0];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin v0<=0;bad0<=0;out_valid<=0;out_bad<=0;end
  else begin v0<=in_valid;bad0<=in_bad;out_valid<=v0;out_bad<=bad0||u>33554431||u< -33554432||v>33554431||v< -33554432;end
 end
endmodule
