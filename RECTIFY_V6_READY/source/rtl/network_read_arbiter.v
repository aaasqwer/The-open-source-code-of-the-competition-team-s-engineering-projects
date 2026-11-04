`timescale 1ns/1ps
// IDs 0..6 always belong to the renderer. ID 7 is borrowed for ONE network
// read only after the renderer's ID-7 response drains. Other renderer reads
// continue concurrently. With no network request all eight renderer IDs work.
// Display retains its independent IDs 8..15 and command priority outside here.
module network_read_arbiter #(parameter A=28)(
 input clk,rst_n,input [A-1:0] p_addr,n_addr,input [3:0] p_id,p_len,n_id,n_len,
 input p_valid,n_valid,output p_ready,n_ready,output reserve_net_id,
 output [A-1:0] addr,output [3:0] id,len,output valid,input ready,
 input [3:0] rid,input rvalid,rlast,output p_rvalid,n_rvalid);
 reg p7_outstanding,net_owner,locked,locked_net;
 wire p7_command=p_valid&&p_id[2:0]==7;
 // An already offered renderer ID-7 command gets its turn before borrowing,
 // including a backpressured command. It must never be withdrawn or blocked
 // behind an entire stream of network requests.
 wire choose_net=locked?locked_net:(n_valid&&!p7_outstanding&&!net_owner&&!p7_command);
 wire blocked_p7=net_owner&&p_id[2:0]==7;
 assign reserve_net_id=n_valid||net_owner||(locked&&locked_net);
 assign addr=choose_net?n_addr:p_addr;assign id=choose_net?4'd7:p_id;
 assign len=choose_net?n_len:p_len;
 assign valid=choose_net?n_valid:(p_valid&&!blocked_p7);
 assign n_ready=choose_net&&ready;
 assign p_ready=!choose_net&&!blocked_p7&&ready;
 wire net_admit=valid&&ready&&choose_net;
 wire p7_admit=valid&&ready&&!choose_net&&p_id[2:0]==7;
 wire network_reply=(net_owner||net_admit)&&rid[2:0]==7;
 assign n_rvalid=rvalid&&network_reply;
 assign p_rvalid=rvalid&&!network_reply;
 wire p7_last=p_rvalid&&rlast&&rid[2:0]==7;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin p7_outstanding<=0;net_owner<=0;locked<=0;locked_net<=0;end
  else begin
   if(valid&&!ready)begin locked<=1;locked_net<=choose_net;end
   else if(valid&&ready)locked<=0;
   if(p7_admit)p7_outstanding<=!p7_last||p7_outstanding;
   else if(p7_last)p7_outstanding<=0;
   if(net_admit)net_owner<=1;
   if(n_rvalid&&rlast)net_owner<=0;
  end
 end
endmodule
