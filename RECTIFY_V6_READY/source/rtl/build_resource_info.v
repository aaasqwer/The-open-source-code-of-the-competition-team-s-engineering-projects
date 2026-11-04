`timescale 1ns/1ps
// One immutable dual-clock ROM: UART report and transparent display text.
// Its contents are finalized against the same post-route build by the release gate.
module build_resource_info(input clk,input [6:0] address,output reg [7:0] data,
 input pix_clk,input [8:0] text_address,output reg [7:0] text_char);
 reg [7:0] report[0:511] /* synthesis syn_ramstyle="block_ram" */;
 initial $readmemh("source/rtl/build_resource.hex",report);
 always @(posedge clk)data<=report[address];
 always @(posedge pix_clk)text_char<=report[text_address];
endmodule
