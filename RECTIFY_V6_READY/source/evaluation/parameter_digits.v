`timescale 1ns/1ps
// Format actual validated parameter registers, never a second set of constants.
// Stable boot-time bus is sampled only after synchronized ready.
module parameter_digits(input clk,rst_n,input ready,input [351:0] parameters,
 output reg done,input [7:0] address,output [7:0] character);
 reg [7:0] text[0:131];
 assign character=done&&address<132?text[address]:8'd45;
 reg r1,r2;reg [3:0] field,digit;reg [2:0] state;
 reg signed [31:0] value;reg [31:0] magnitude,decimal;
 reg [63:0] product;
 always @(posedge clk)begin
  if(!rst_n)begin done<=0;r1<=0;r2<=0;field<=0;state<=0;end
  else begin
   r1<=ready;r2<=r1;
   case(state)
    0:if(r2)begin value<=parameters[field*32+:32];state<=1;end
    1:begin magnitude<=value[31]?-value:value;text[field*12]<=value[31]?8'd45:8'd43;state<=2;end
    2:begin product<=magnitude*64'd1000000;state<=3;end
    3:begin decimal<=(field>=4&&field<=7)?(product>>24):(field==8||field==9)?(product>>30):(field==10)?(product>>20):(product>>16);digit<=11;state<=4;end
    4:begin
     if(digit==5)text[field*12+digit]<=8'd46;
     else begin text[field*12+digit]<=8'd48+(decimal%10);decimal<=decimal/10;end
     if(digit==1)begin if(field==10)begin done<=1;state<=5;end else begin field<=field+1'b1;state<=0;end end
     else digit<=digit-1'b1;
    end
    default:begin end
   endcase
  end
 end
endmodule
