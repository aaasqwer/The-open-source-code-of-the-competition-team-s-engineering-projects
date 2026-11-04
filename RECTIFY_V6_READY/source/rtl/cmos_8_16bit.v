`timescale 1ns/1ps
module cmos_8_16bit(input pclk,rst_n,de_i,input [7:0] pdata_i,input vs_i,
 output pixel_clk, output reg de_o,output reg [15:0] pdata_o,
 output reg line_end,output reg line_active);
 reg phase,href_d;reg [7:0] high_byte;
 assign pixel_clk=pclk;
 always @(posedge pclk or negedge rst_n) begin
  if(!rst_n) begin phase<=0;href_d<=0;high_byte<=0;pdata_o<=0;de_o<=0;line_end<=0;line_active<=0;end
  else begin
   href_d<=de_i;de_o<=0;line_end<=href_d && !de_i && !vs_i;
   line_active<=de_i && !vs_i;
   if(vs_i || !de_i) phase<=0;
   else if(!phase) begin high_byte<=pdata_i;phase<=1;end
   else begin pdata_o<={high_byte,pdata_i};de_o<=1;phase<=0;end
  end
 end
endmodule
