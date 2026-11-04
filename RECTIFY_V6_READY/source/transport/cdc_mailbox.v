`timescale 1ns/1ps
// Bundled-data handshake: source holds payload until destination acknowledges.
module cdc_mailbox #(parameter WIDTH=64)(input src_clk,src_rst_n,dst_clk,dst_rst_n,
 input [WIDTH-1:0] src_data,output reg [WIDTH-1:0] dst_data);
 reg [WIDTH-1:0] hold_data;reg req,ack;reg a1,a2,r1,r2;
 always @(posedge src_clk or negedge src_rst_n)begin
 if(!src_rst_n)begin req<=0;a1<=0;a2<=0;hold_data<=0;end
 else begin a1<=ack;a2<=a1;if(req==a2)begin hold_data<=src_data;req<=!req;end end end
 always @(posedge dst_clk or negedge dst_rst_n)begin
 if(!dst_rst_n)begin ack<=0;r1<=0;r2<=0;dst_data<=0;end
 else begin r1<=req;r2<=r1;if(r2!=ack)begin dst_data<=hold_data;ack<=r2;end end end
endmodule
