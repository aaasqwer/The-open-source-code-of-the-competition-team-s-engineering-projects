`timescale 1ns/1ps
// Small upper-right report. Only opaque glyph pixels change; no background fill.
module resource_overlay #(parameter CONTROLLED=0)(input clk,rst_n,vs,hs,de,enable,input [11:0] x,y,input [23:0] rgb,
 output [8:0] text_address,input [7:0] text_char,
 output reg out_vs,out_hs,out_de,output reg [11:0] out_x,out_y,output reg [23:0] out_rgb);
 wire region=de&&(CONTROLLED==0||enable)&&x>=1384&&x<1896&&y>=10&&y<118;
 wire [11:0] dx=x-1384,dy=y-10;
 // Six 18px rows. The two separator scanlines must remain the original picture.
 wire [2:0] row=dy/18;wire [4:0] col=dx[8:4];wire [4:0] gy=dy-row*18;
 assign text_address=9'd128+{row,5'd0}+col;
 reg [15:0] font[0:2047] /* synthesis syn_ramstyle="block_ram" */;
 initial $readmemh("source/rtl/resource_font.hex",font);
 reg [15:0] bits;reg [3:0] fy1,gx1,gx2;reg on1,on2;
 reg [23:0] rgb1,rgb2;reg [11:0] x1,y1,x2,y2;reg [2:0] sync1,sync2;
 always @(posedge clk)begin
  fy1<=gy[3:0];gx1<=dx[3:0];gx2<=gx1;on1<=region&&gy<16;on2<=on1;
  bits<=font[{text_char[6:0],fy1}];rgb1<=rgb;rgb2<=rgb1;
  x1<=x;y1<=y;x2<=x1;y2<=y1;sync1<={vs,hs,de};sync2<=sync1;
  {out_vs,out_hs,out_de}<=sync2;out_x<=x2;out_y<=y2;
  out_rgb<=on2&&bits[15-gx2]?24'h00ffff:rgb2;
  if(!rst_n)begin on1<=0;on2<=0;sync1<=0;sync2<=0;out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;end
 end
endmodule
