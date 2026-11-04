`timescale 1ns/1ps
// Bundled data mailbox: hold the complete angle word until the receiver ACKs.
module imu_angle_cdc(input src_clk,dst_clk,rst_n,input signed [15:0] src_angle,
 output reg signed [15:0] dst_angle);
 reg signed [15:0] held;
 reg request,ack,a1,a2,r1,r2;
 always @(posedge src_clk or negedge rst_n)
  if(!rst_n) begin held<=0;request<=0;a1<=0;a2<=0;end
  else begin
   a1<=ack;a2<=a1;
   if(request==a2 && held!=src_angle) begin held<=src_angle;request<=!request;end
  end
 always @(posedge dst_clk or negedge rst_n)
  if(!rst_n) begin r1<=0;r2<=0;ack<=0;dst_angle<=0;end
  else begin
   r1<=request;r2<=r1;
   if(r2!=ack) begin dst_angle<=held;ack<=r2;end
  end
endmodule
