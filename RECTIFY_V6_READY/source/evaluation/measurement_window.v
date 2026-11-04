`timescale 1ns/1ps
// First accepted result is immediately visible; every seven accepted results
// update the exact median and range. No cross-mode history.
// Invalid detection clears both visible values and pending history immediately.
module measurement_window(input clk,rst_n,clear,sample_valid,input [15:0] rms_in,max_in,input [8:0] rows_in,
 output reg valid,output reg [15:0] rms_median,max_median,range_low,range_high,output reg [8:0] rows_min,output [2:0] samples);
 reg last_valid;reg [2:0] count,state,i,j;reg [15:0] rs[0:6],ms[0:6];reg [8:0] min_rows;
 assign samples=state==0?count:3'd7;
 wire event_in=sample_valid&&!last_valid;
 always @(posedge clk)begin
  if(!rst_n||clear)begin last_valid<=0;count<=0;state<=0;valid<=0;rms_median<=0;max_median<=0;range_low<=0;range_high<=0;rows_min<=0;min_rows<=511;i<=0;j<=0;end
  else begin
   last_valid<=sample_valid;
   case(state)
    0:if(event_in)begin
     rs[count]<=rms_in;ms[count]<=max_in;
     if(!valid)begin valid<=1;rms_median<=rms_in;max_median<=max_in;range_low<=rms_in;range_high<=rms_in;rows_min<=rows_in;end
     if(rows_in<min_rows)min_rows<=rows_in;
     if(count==6)begin i<=0;j<=0;state<=1;end else count<=count+1'b1;
    end
    1:begin
     if(rs[j]>rs[j+1])begin rs[j]<=rs[j+1];rs[j+1]<=rs[j];end
     if(ms[j]>ms[j+1])begin ms[j]<=ms[j+1];ms[j+1]<=ms[j];end
     if(j==5)begin j<=0;if(i==5)state<=2;else i<=i+1'b1;end else j<=j+1'b1;
    end
    2:begin rms_median<=rs[3];max_median<=ms[3];range_low<=rs[0];range_high<=rs[6];rows_min<=min_rows;valid<=1;count<=0;min_rows<=511;state<=0;end
    default:state<=0;
   endcase
  end
 end
endmodule
