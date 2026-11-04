`timescale 1ns/1ps
module tb_raw_reset_grant;
 reg clk=0,eth=0,rst=0,nrst=0,hold=0,req=0,arready=0,rv=0;always #4 clk=~clk;always #5 eth=~eth;
 wire av,ready,leased,done;wire [27:0] address;wire [3:0] id,len;
 network_pair_reader #(.W(96),.H(24)) dut(.clk(clk),.rst_n(rst),.net_clk(eth),.net_rst_n(nrst),.hold_frame(hold),.request_toggle(req),
  .request_row(11'd0),.read_x(11'd0),.read_pixel(),.ready(ready),.done_toggle(done),.frame_id(),
  .source_valid(1'b1),.latest_bank(2'd2),.latest_id(32'd12),.lease_active(leased),.leased_bank(),
  .araddr(address),.arlen(len),.arid(id),.arvalid(av),.arready(arready),.rdata(256'd0),.rvalid(rv),.rlast(rv));
 reg [27:0] old_address;
 initial begin
  repeat(6)@(negedge clk);rst=1;nrst=1;hold=1;wait(ready);@(negedge eth);req=1;
  wait(av);old_address=address;repeat(3)@(negedge clk);nrst=0;hold=0;
  repeat(8)@(negedge clk);
  if(!av||address!==old_address||!leased)$fatal(1,"PHY reset abandoned stalled DDR AR or released its bank");
  arready=1;@(negedge clk);arready=0;repeat(6)@(negedge clk);
  if(!leased)$fatal(1,"PHY reset released bank before accepted response");
  rv=1;@(negedge clk);rv=0;repeat(6)@(negedge clk);
  if(leased||av||done)$fatal(1,"orphan read not drained or partial line published");
  nrst=1;repeat(30)@(negedge eth);if(ready)$fatal(1,"old lease replayed");
  // A second reset catches a prefetched outstanding read while pixels of
  // the current beat are already being expanded into the line RAM.
  hold=1;wait(ready);@(negedge eth);req=0;wait(av);
  @(negedge clk);arready=1;@(negedge clk);arready=0;repeat(2)@(negedge clk);rv=1;@(negedge clk);rv=0;
  wait(av);@(negedge clk);arready=1;@(negedge clk);arready=0;
  nrst=0;hold=0;repeat(8)@(negedge clk);if(!leased)$fatal(1,"prefetched read lost its lease");
  rv=1;@(negedge clk);rv=0;repeat(8)@(negedge clk);
  if(leased||done||av)$fatal(1,"prefetch reset leaked partial completion or bank");
  $display("PASS PHY reset retains stalled AR/lease, drains accepted read, suppresses partial completion");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
