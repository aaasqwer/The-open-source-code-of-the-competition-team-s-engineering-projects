`timescale 1ns/1ps
// Atomic telemetry mailbox. Hold data from request until destination ACK.
module imu_snapshot_cdc #(parameter W=640)(input src_clk,dst_clk,rst_n,
 input [W-1:0] src_data,output reg [W-1:0] dst_data);
 reg [W-1:0] held;
 reg request,ack,a1,a2,r1,r2;
 always @(posedge src_clk or negedge rst_n)
  if(!rst_n) begin held<=0;request<=0;a1<=0;a2<=0;end
  else begin
   a1<=ack;a2<=a1;
   if(request==a2 && held!=src_data) begin held<=src_data;request<=!request;end
  end
 always @(posedge dst_clk or negedge rst_n)
  if(!rst_n) begin r1<=0;r2<=0;ack<=0;dst_data<=0;end
  else begin
   r1<=request;r2<=r1;
   if(r2!=ack) begin dst_data<=held;ack<=r2;end
  end
endmodule
