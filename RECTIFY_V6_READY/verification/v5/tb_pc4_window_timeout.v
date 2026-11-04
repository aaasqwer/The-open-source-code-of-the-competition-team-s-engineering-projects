`timescale 1ns/1ps
module tb_pc4_window_timeout;
 reg clk=0,rst=0,cap=0,start=0,active=0,au=0;always #4 clk=~clk;
 reg [31:0]sid=1;reg [63:0]ack=0;wire busy,pending,abort,fault,en;wire [7:0]data;
 ethernet_photo_window #(.COALESCE_TICKS(2),.ACK_TICKS(100),.MAX_RETRIES(2)) dut(
 .clk(clk),.rst_n(rst),.capture(cap),.capture_sequence(sid),.ack_update(au),.ack(ack),
 .image_start(start),.image_data(8'h5a),.image_packet_active(active),.image_busy(busy),.image_pending(pending),.abort(abort),.fault(fault),.tx_en(en),.tx_data(data));
 task begin_photo;input [31:0]id;begin
  @(negedge clk);sid=id;cap=1;@(negedge clk);cap=0;repeat(3)@(negedge clk);
  if(fault||busy)$fatal(1,"capture did not clear fault");
  start=1;@(negedge clk);start=0;
 end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;begin_photo(1);
  wait(abort);@(negedge clk);
  if(!fault||pending||en)$fatal(1,"retry exhaustion did not release data");
  begin_photo(2);wait(en);wait(!en);
  @(negedge clk);ack={32'd0,32'd1};au=1;@(negedge clk);au=0;repeat(3)@(negedge clk);
  if(!pending||dut.base_seq!=0)$fatal(1,"old-session ACK retired recovered capture");
  ack={32'd0,32'd2};au=1;@(negedge clk);au=0;wait(!pending);
  if(fault)$fatal(1,"recovered capture faulted");
  $display("PASS window retry exhaustion, slot release, new capture and stale-session ACK rejection");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
