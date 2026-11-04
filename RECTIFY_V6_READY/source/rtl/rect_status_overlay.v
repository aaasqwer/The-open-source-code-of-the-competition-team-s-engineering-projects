`timescale 1ns/1ps
// Small two-line caption. Full diagnostic numbers are opt-in, not on the
// default camera view. Pixel coordinates and sync share the font latency.
module rect_status_overlay(input clk,rst_n,enable,vs,hs,de,input [11:0] x,y,input [23:0] rgb,
 input [31:0] actual_geometry,input plane_held,plane_valid,mapping_fault,
 output reg out_vs,out_hs,out_de,output reg [11:0] out_x,out_y,output reg [23:0] out_rgb);
 reg [15:0] font[0:2047];reg [6:0] text[0:959];
 initial begin $readmemh("source/ui/caption_font.hex",font);$readmemh("source/ui/caption_text.hex",text);end
 wire row1=y>=36;wire area=enable&&de&&x>=16&&x<1552&&((y>=16&&y<32)||(y>=36&&y<52));
 wire [3:0] message=row1?(mapping_fault?4:plane_held&&!plane_valid?5:(actual_geometry[5]?8:6)+actual_geometry[6]):{2'd0,actual_geometry[1:0]};
 wire [6:0] column=(x-16)>>4;wire [3:0] row=row1?y-36:y-16;
 wire [9:0] text_address=area?message*96+column:0;
 reg [6:0] character;reg [3:0] row0,col0,col1;reg [15:0] glyph;reg on0,on1;
 reg [23:0] p0,p1;reg v0,v1,h0,h1,d0,d1;reg [11:0] x0,x1,y0,y1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_vs<=0;out_hs<=0;out_de<=0;out_x<=0;out_y<=0;out_rgb<=0;on0<=0;on1<=0;v0<=0;v1<=0;h0<=0;h1<=0;d0<=0;d1<=0;end
  else begin
   character<=text[text_address];row0<=row;col0<=x-16;glyph<=font[{character,row0}];col1<=col0;on0<=area;on1<=on0;
   p0<=rgb;p1<=p0;v0<=vs;v1<=v0;h0<=hs;h1<=h0;d0<=de;d1<=d0;x0<=x;x1<=x0;y0<=y;y1<=y0;
   out_vs<=v1;out_hs<=h1;out_de<=d1;out_x<=x1;out_y<=y1;out_rgb<=on1?(glyph[15-col1]?24'hfff2c9:24'h18202b):p1;
  end
 end
endmodule
