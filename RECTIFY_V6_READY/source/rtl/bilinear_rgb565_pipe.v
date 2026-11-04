`timescale 1ns/1ps
// Four source taps are supplied by the frame-memory client. Q12 fractions;
// horizontal sums keep their fractional bits, then round once after vertical.
// One pixel accepted each clock, including validity gaps. Three register stages.
module bilinear_rgb565_pipe(
 input clk,rst_n,in_valid,input [15:0] p00,p10,p01,p11,
 input [11:0] fraction_x,fraction_y,
 output reg out_valid,output reg [15:0] pixel
);
 function [17:0] blend;
  input [5:0] a,b;input [11:0] f;
  reg signed [6:0] difference;
  reg signed [20:0] value;
  begin difference=$signed({1'b0,b})-$signed({1'b0,a});
   value=$signed({1'b0,a,12'd0})+difference*$signed({1'b0,f});blend=value[17:0];end
 endfunction
 function [29:0] finish;
  input [17:0] a,b;input [11:0] f;
  reg signed [18:0] difference;
  reg signed [32:0] value;
  begin difference=$signed({1'b0,b})-$signed({1'b0,a});
   value=$signed({1'b0,a,12'd0})+difference*$signed({1'b0,f})+33'sd8388608;finish=value[29:0];end
 endfunction
 reg [17:0] rt,rb,gt,gb,bt,bb;
 reg [29:0] r,g,b;
 reg [11:0] fy;
 reg v1,v2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rt<=0;rb<=0;gt<=0;gb<=0;bt<=0;bb<=0;r<=0;g<=0;b<=0;fy<=0;v1<=0;v2<=0;out_valid<=0;pixel<=0;end
  else begin
   v1<=in_valid;v2<=v1;out_valid<=v2;
   fy<=fraction_y;
   rt<=blend({1'b0,p00[15:11]},{1'b0,p10[15:11]},fraction_x);rb<=blend({1'b0,p01[15:11]},{1'b0,p11[15:11]},fraction_x);
   gt<=blend(p00[10:5],p10[10:5],fraction_x);gb<=blend(p01[10:5],p11[10:5],fraction_x);
   bt<=blend({1'b0,p00[4:0]},{1'b0,p10[4:0]},fraction_x);bb<=blend({1'b0,p01[4:0]},{1'b0,p11[4:0]},fraction_x);
   r<=finish(rt,rb,fy);g<=finish(gt,gb,fy);b<=finish(bt,bb,fy);
   pixel<={r[28:24],g[29:24],b[28:24]};
  end
 end
endmodule
