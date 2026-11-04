`timescale 1ns/1ps
module tb_photo_progress;
 reg clk=0,rst=0,capture=0,vs=0,invalid=0,stall=1,byte_busy=0;always #5 clk=~clk;
 wire busy=stall||byte_busy;
 wire request,abort_capture,start,transfer,packet,line_req;wire[7:0]data;wire[3:0]state;wire[2:0]reason;
 always @(posedge clk)byte_busy<=start;
 fast_snapshot_ctrl #(.CLK_HZ(1000),.WIDTH(32),.HEIGHT(2)) dut(.clk(clk),.rst_n(rst),.capture_pulse(capture),.invalid_frame(invalid),.abort_capture(abort_capture),
  .capture_sequence(32'd17),.freeze_request(request),.freeze_active_vin(request),.video_vs(vs),.mode_pix(2'd1),.model_pix(1'b0),
  .line_request_toggle(line_req),.line_done_toggle_pix(1'b0),.captured_row_pix(11'd0),.line_rd_addr(),.line_rd_data(16'd0),
  .uart_start(start),.uart_data(data),.uart_busy(busy),.packet_active(packet),.transfer_active(transfer),.capture_state(state),.completed_rows(),.abort_reason(reason));
 integer i;reg expected_abort=0;
 always @(negedge clk)if(abort_capture&&!expected_abort)$fatal(1,"active capture timed out despite progress");
 task boundary;begin repeat(5)@(negedge clk);vs=1;repeat(5)@(negedge clk);vs=0;end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;capture=1;@(negedge clk);capture=0;
  // Scaled clock: valid progress takes 4.5 seconds overall, each interval <2s.
  for(i=0;i<3;i=i+1)begin repeat(1495)@(negedge clk);boundary();end
  repeat(20)@(negedge clk);if(!request||!transfer||state==1)$fatal(1,"valid slow freezing did not reach INFO");
  expected_abort=1;wait(abort_capture);@(negedge clk);
  if(request||transfer||reason!=2)$fatal(1,"no-progress timeout not reported/released");
  repeat(30)@(negedge clk);if(reason!=2)$fatal(1,"abort reason erased by reset feedback");
  stall=0;capture=1;@(negedge clk);capture=0;boundary();boundary();boundary();boundary();
  wait(line_req);invalid=1;wait(abort_capture);@(negedge clk);
  if(reason!=1||request||transfer)$fatal(1,"invalid source did not abort");
  $display("PASS slow progressing capture exceeds old total deadline; true inactivity aborts, preserves reason and recovers");$finish;
 end
 initial begin #300000;$fatal(1,"progress watchdog timeout state=%0d reason=%0d watchdog=%0d req=%b line=%b stable=%0d fr=%b busy=%b",state,reason,dut.watchdog,request,line_req,dut.stable,dut.fr2,busy);end
endmodule
