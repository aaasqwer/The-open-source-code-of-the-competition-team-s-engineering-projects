`timescale 1ns/1ps
module measurement_digits(input clk,rst_n,valid,input [15:0] rms_q8,max_q8,input [8:0] used_rows,
 input [4:0] address,output [7:0] character);
 reg [7:0] text[0:14];reg [2:0] state;reg [2:0] digit;reg [1:0] field;
 reg [31:0] decimal;reg done;reg [15:0] previous_rms,previous_max;reg [8:0] previous_rows;
 wire [31:0] rms_scaled=({16'd0,rms_q8}*32'd100+128)>>8;
 wire [31:0] max_scaled=({16'd0,max_q8}*32'd100+128)>>8;
 wire [4:0] local_address=address<6?address:(address<12?address-6:address-12);
 assign character=valid&&done&&address<15?text[address]:
  (local_address==0?8'd78:local_address==1?8'd47:local_address==2?8'd65:8'd32);
 always @(posedge clk)begin
  if(!rst_n||!valid)begin done<=0;state<=0;field<=0;end
  else if(done&&(rms_q8!=previous_rms||max_q8!=previous_max||used_rows!=previous_rows))begin done<=0;state<=0;end
  else case(state)
   0:begin previous_rms<=rms_q8;previous_max<=max_q8;previous_rows<=used_rows;decimal<=rms_scaled;digit<=5;field<=0;state<=1;end
   1:begin
    if(digit==3)text[field*6+digit]<=8'd46;
    else begin text[field*6+digit]<=8'd48+decimal%10;decimal<=decimal/10;end
    if(digit==0)begin
     if(field==0)begin field<=1;decimal<=max_scaled;digit<=5;end
     else begin decimal<=used_rows;digit<=2;state<=2;end
    end else digit<=digit-1'b1;
   end
   2:begin text[12+digit]<=8'd48+decimal%10;decimal<=decimal/10;
    if(digit==0)begin done<=1;state<=3;end else digit<=digit-1'b1;
   end
   default:begin end
  endcase
 end
endmodule
