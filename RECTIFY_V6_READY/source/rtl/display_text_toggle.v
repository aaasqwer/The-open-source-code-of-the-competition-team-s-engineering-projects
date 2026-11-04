`timescale 1ns/1ps
// One debounced press-and-release toggles camera text, committed at VS only.
module display_text_toggle #(parameter DEBOUNCE_CYCLES=1485000)(
 input clk,rst_n,key1_n,vs,hold_frame,output reg visible);
 (* ASYNC_REG="TRUE" *) reg k1,k2;
 reg candidate,stable,armed,requested,vs_d;reg [20:0] age;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin k1<=1;k2<=1;candidate<=1;stable<=1;armed<=0;requested<=1;visible<=1;age<=0;vs_d<=0;end
  else begin
   k1<=key1_n;k2<=k1;vs_d<=vs;
   if(k2!=candidate)begin candidate<=k2;age<=0;end
   else if(k2==stable)age<=0;
   else if(DEBOUNCE_CYCLES<=1||age==DEBOUNCE_CYCLES-1)begin
    stable<=k2;age<=0;
    if(!k2)armed<=1;
    else if(armed)begin requested<=!requested;armed<=0;end
   end else age<=age+1'b1;
   if(vs&&!vs_d&&!hold_frame)visible<=requested;
  end
 end
endmodule
