`timescale 1ns/1ps
// HDMI branch only. One common Q10 gain preserves channel ratios; bounded
// shadow lift keeps black/white fixed. No spatial resampling or sharpness gain.
module hdmi_natural_tone(input clk,rst_n,vs,hs,de,input [11:0] x,y,input [23:0] rgb,
 output reg out_vs,out_hs,out_de,output reg [11:0] out_x,out_y,output reg [23:0] out_rgb);
 reg [10:0] gain_table[0:255];initial $readmemh("source/rtl/hdmi_natural_gain.hex",gain_table);
 wire [7:0] rg=rgb[23:16]>rgb[15:8]?rgb[23:16]:rgb[15:8];
 wire [7:0] value=rg>rgb[7:0]?rg:rgb[7:0];
 reg [10:0] gain;reg [23:0] p0;reg [18:0] r1,g1,b1;
 reg [2:0] s0,s1;reg [11:0] x0,x1,y0,y1;
 function [7:0] rounded;input [18:0] p;reg [19:0] q;
 begin q={1'b0,p}+20'd512;rounded=q[19:10]>255?255:q[17:10];end endfunction
 always @(posedge clk)begin
  gain<=gain_table[value];p0<=rgb;s0<={vs,hs,de};x0<=x;y0<=y;
  r1<=p0[23:16]*gain;g1<=p0[15:8]*gain;b1<=p0[7:0]*gain;s1<=s0;x1<=x0;y1<=y0;
  {out_vs,out_hs,out_de}<=s1;out_x<=x1;out_y<=y1;
  out_rgb<=s1[0]?{rounded(r1),rounded(g1),rounded(b1)}:24'd0;
  if(!rst_n)begin s0<=0;s1<=0;out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;end
 end
endmodule
