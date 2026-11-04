`timescale 1ns/1ps
// Explicit IO registers, using the vendor-supported SAME_EDGE ODDR primitive.
// Equal D0/D1 values give SDR data: both half cycles carry the same pixel.
// All 24 RGB and 3 sync bits share the same extra cycle. The existing
// inverted PLL forwarding clock remains unchanged at the board pin.
module hdmi_sdr_output(input clk,rst_n,input [23:0] rgb,input vs,hs,de,
 output [23:0] out_rgb,output out_vs,out_hs,out_de);
 wire [26:0] din,dout;assign din={vs,hs,de,rgb};
 genvar i;
 generate for(i=0;i<27;i=i+1)begin:io_register
  GTP_ODDR_E1 #(.ODDR_MODE("SAME_EDGE"),.GRS_EN("FALSE"),.RS_TYPE("ASYNC_RESET")) output_ff(
   .CLK(clk),.CE(1'b1),.RS(!rst_n),.D0(din[i]),.D1(din[i]),.Q(dout[i]));
 end endgenerate
 assign {out_vs,out_hs,out_de,out_rgb}=dout;
endmodule
