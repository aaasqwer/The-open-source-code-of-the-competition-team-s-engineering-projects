`timescale 1ns/1ps
// Unsigned A x 32 exact product, four register stages, one input per clock.
// Register 16x16 partial products rather than a long combinational APM cascade.
module rect_wide_mul_pipe #(parameter A=32)(input clk,input [A-1:0] a,input [31:0] b,output reg [A+31:0] product);
 wire [47:0] aa={{(48-A){1'b0}},a};
 reg [31:0] p00,p01,p10,p11,p20,p21;
 reg [47:0] row0,row1,row2,upper;
 reg [63:0] lower;
 always @(posedge clk)begin
  p00<=aa[15:0]*b[15:0];p01<=aa[15:0]*b[31:16];
  p10<=aa[31:16]*b[15:0];p11<=aa[31:16]*b[31:16];
  p20<=aa[47:32]*b[15:0];p21<=aa[47:32]*b[31:16];
  row0<={16'd0,p00}+{p01,16'd0};row1<={16'd0,p10}+{p11,16'd0};row2<={16'd0,p20}+{p21,16'd0};
  lower<={16'd0,row0}+{row1,16'd0};upper<=row2;
  product<={16'd0,lower}+{upper,32'd0};
 end
endmodule
