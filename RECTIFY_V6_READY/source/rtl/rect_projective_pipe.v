`timescale 1ns/1ps
// Shared normalized reciprocal; Brown/lens model remains downstream unchanged.
// Exact partial-product multipliers are pipelined to avoid long DSP cascades.
module rect_projective_pipe #(parameter TW=11)(
 input clk,rst_n,in_valid,input [10:0] x,y,input [TW-1:0] in_tag,
 input [287:0] inverse_matrix,
 output out_valid,out_bad,output signed [25:0] ideal_x,ideal_y,output [TW-1:0] out_tag
);
 wire signed [31:0] h00=inverse_matrix[0+:32],h01=inverse_matrix[32+:32],h02=inverse_matrix[64+:32];
 wire signed [31:0] h10=inverse_matrix[96+:32],h11=inverse_matrix[128+:32],h12=inverse_matrix[160+:32];
 wire signed [31:0] h20=inverse_matrix[192+:32],h21=inverse_matrix[224+:32],h22=inverse_matrix[256+:32];
 reg signed [43:0] ax,bx,ay,by,ad,bd;reg signed [31:0] tx,ty,td;
 reg signed [47:0] nx,ny,dn;reg v0,v1,v2;reg [TW-1:0] t0,t1,t2;
 // Register absolute values before priority encode/barrel shift/ROM address.
 // This cuts a measured long path without changing any arithmetic precision.
 reg [47:0] abs_x,abs_y,abs_d;reg sign_x,sign_y;
 function [5:0] leading;input [31:0] n;integer j;begin leading=0;for(j=0;j<32;j=j+1)if(n[j])leading=j;end endfunction
 wire [5:0] k=leading(abs_d[31:0]);
 wire [31:0] norm=(k<=30)?abs_d[31:0]<<(30-k):abs_d[31:0]>>(k-30);
 wire initial_bad=(abs_d<262144||abs_d>48'h0000ffffffff||abs_x[47:36]!=0||abs_y[47:36]!=0||
  (abs_x>>4)>=abs_d||(abs_y>>4)>=abs_d);
 reg [30:0] seed_rom[0:255];initial $readmemh("source/rtl/reciprocal_q30.hex",seed_rom);
 reg [31:0] normalized[0:11];
 // Stages: norm0, seed1, dp0[2..5], error6, rp0[7..10], r1=11,
 // dp1[12..15], error16, rp1[17..20], r2=21, xy[22..25], quotient26.
 localparam MW=TW+81;
 reg [MW-1:0] meta[0:26];reg [26:0] val;
 // meta = {tag,bad,sign_y,sign_x,k,abs_y[35:0],abs_x[35:0]}.
 reg [30:0] r0,r1,r2,r0_delay[0:4],r1_delay[0:4];
 wire [63:0] dp0,rp0,dp1,rp1;wire [67:0] xp,yp;
 reg [31:0] error0,error1;
 reg [25:0] qx,qy;reg range_bad;
 wire [67:0] shifted_x=xp>>(meta[25][77:72]+8),shifted_y=yp>>(meta[25][77:72]+8);
 rect_wide_mul_pipe md0(.clk(clk),.a(normalized[1]),.b({1'b0,r0}),.product(dp0));
 rect_wide_mul_pipe mr0(.clk(clk),.a({1'b0,r0_delay[4]}),.b(error0),.product(rp0));
 rect_wide_mul_pipe md1(.clk(clk),.a(normalized[11]),.b({1'b0,r1}),.product(dp1));
 rect_wide_mul_pipe mr1(.clk(clk),.a({1'b0,r1_delay[4]}),.b(error1),.product(rp1));
 rect_wide_mul_pipe #(.A(36)) mx(.clk(clk),.a(meta[21][35:0]),.b({1'b0,r2}),.product(xp));
 rect_wide_mul_pipe #(.A(36)) my(.clk(clk),.a(meta[21][71:36]),.b({1'b0,r2}),.product(yp));
 integer j;
 always @(posedge clk)begin
  ax<=h00*$signed({1'b0,x});bx<=h01*$signed({1'b0,y});tx<=h02;
  ay<=h10*$signed({1'b0,x});by<=h11*$signed({1'b0,y});ty<=h12;
  ad<=h20*$signed({1'b0,x});bd<=h21*$signed({1'b0,y});td<=h22;
  nx<=$signed(ax)+$signed(bx)+$signed(tx);ny<=$signed(ay)+$signed(by)+$signed(ty);dn<=$signed(ad)+$signed(bd)+$signed(td);
  abs_x<=nx[47]?-nx:nx;abs_y<=ny[47]?-ny:ny;abs_d<=dn[47]?-dn:dn;
  sign_x<=nx[47]^dn[47];sign_y<=ny[47]^dn[47];
  t0<=in_tag;t1<=t0;t2<=t1;
  normalized[0]<=norm;for(j=1;j<12;j=j+1)normalized[j]<=normalized[j-1];
  meta[0]<={t2,initial_bad,sign_y,sign_x,k,abs_y[35:0],abs_x[35:0]};
  for(j=1;j<27;j=j+1)meta[j]<=meta[j-1];
  r0<=seed_rom[normalized[0][29:22]];
  r0_delay[0]<=r0;r1_delay[0]<=r1;
  for(j=1;j<5;j=j+1)begin r0_delay[j]<=r0_delay[j-1];r1_delay[j]<=r1_delay[j-1];end
  error0<=32'h80000000-((dp0+64'd536870912)>>30);
  r1<=(rp0+64'd536870912)>>30;
  error1<=32'h80000000-((dp1+64'd536870912)>>30);
  r2<=(rp1+64'd536870912)>>30;
  qx<=shifted_x[25:0];qy<=shifted_y[25:0];
  range_bad<=shifted_x>=33554432||shifted_y>=33554432;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin v0<=0;v1<=0;v2<=0;val<=0;end
  else begin v0<=in_valid;v1<=v0;v2<=v1;val<={val[25:0],v2};end
 end
 assign ideal_x=meta[26][78]?-$signed(qx):$signed(qx);
 assign ideal_y=meta[26][79]?-$signed(qy):$signed(qy);
 assign out_valid=val[26];assign out_bad=meta[26][80]||range_bad;
 assign out_tag=meta[26][MW-1:81];
endmodule
