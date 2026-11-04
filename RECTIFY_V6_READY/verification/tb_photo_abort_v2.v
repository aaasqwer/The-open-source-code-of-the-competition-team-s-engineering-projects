`timescale 1ns/1ps
module tb_photo_abort;
 reg clk=0,rst=0,capture=0,vs=0,invalid=0;always #4 clk=~clk;
 wire request,abort_capture,start,transfer,active,line_req;wire[7:0]data;
 reg uart_busy=0;always @(posedge clk)uart_busy<=start;
 fast_snapshot_ctrl #(.CLK_HZ(10000),.WIDTH(64),.HEIGHT(24)) dut(.clk(clk),.rst_n(rst),.capture_pulse(capture),.invalid_frame(invalid),.abort_capture(abort_capture),
 .capture_sequence(32'd123),.freeze_request(request),.freeze_active_vin(request),.video_vs(vs),.mode_pix(2'd1),.model_pix(1'b0),
 .line_request_toggle(line_req),.line_done_toggle_pix(1'b0),.captured_row_pix(11'd0),.line_rd_addr(),.line_rd_data(16'd0),
 .uart_start(start),.uart_data(data),.uart_busy(uart_busy),.packet_active(active),.transfer_active(transfer));
 integer bytes=0;
 always @(posedge clk)if(start)begin
  if(bytes==5&&data!=8'h81)$fatal(1,"invalid photo sent completion packet");bytes=bytes+1;
 end
 task boundary;begin repeat(10)@(negedge clk);vs=1;repeat(10)@(negedge clk);vs=0;end endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;capture=1;@(negedge clk);capture=0;
  boundary();boundary();boundary();boundary();
  wait(line_req);@(negedge clk);invalid=1;@(negedge clk);
  if(!abort_capture||request||transfer)$fatal(1,"bad source frame not aborted/released");
  invalid=0;repeat(4)@(negedge clk);capture=1;@(negedge clk);capture=0;
  if(!request||!transfer)$fatal(1,"new photo cannot recover after abort");
  $display("PASS invalid photo aborts before DONE, releases freeze, accepts recovery");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
