`timescale 1ns/1ps
// Production dimensions, final row/column/address, repeated leases, cancellation.
module tb_raw_edges;
 reg ddr=0,eth=0,rst=0,nrst=0;always #4 ddr=~ddr;always #6 eth=~eth;
 reg hold=0,req=0;reg [10:0] row=0,x=0;wire [15:0] pixel;wire ready,done,leased;
 wire [1:0] bank;wire [31:0] frame;wire [27:0] addr;wire [3:0] len,id;wire valid;
 reg arready=0,rv=0;reg [255:0] data=0;integer delay_count=0,beat=0,cycles=0,requests=0;
 reg pending=0;reg [27:0] saved_addr;
 network_pair_reader dut(.clk(ddr),.rst_n(rst),.net_clk(eth),.net_rst_n(nrst),.hold_frame(hold),.request_toggle(req),
  .request_row(row),.read_x(x),.read_pixel(pixel),.ready(ready),.done_toggle(done),.frame_id(frame),
  .source_valid(1'b1),.latest_bank(2'd3),.latest_id(32'd123),.lease_active(leased),.leased_bank(bank),
  .araddr(addr),.arlen(len),.arid(id),.arvalid(valid),.arready(arready),.rdata(data),.rvalid(rv),.rlast(rv));
 function [15:0] value;input [27:0] a;input integer lane;
 begin value=((a>>3)*16+lane)^16'h96a5;end endfunction
 always @(posedge ddr)if(rst)begin
  cycles=cycles+1;
  if(valid&&arready)begin
   if(len!=0||id!=7||addr[2:0]!=0||addr[23:22]!=3)$fatal(1,"bad production address/id/bank");
   saved_addr=addr;pending=1;delay_count=9;requests=requests+1;
  end
 end
 always @(negedge ddr)begin
  arready=!pending&&cycles%5!=0;rv=0;
  if(pending)begin
   if(delay_count>0)delay_count=delay_count-1;
   else begin
    for(beat=0;beat<16;beat=beat+1)data[beat*16+:16]=value(saved_addr,beat);
    rv=1;pending=0;
   end
  end
 end
 task line;input [10:0] yy;reg previous;integer xx;reg [15:0] expected;
 begin
  @(negedge eth);row=yy;previous=done;req=~req;wait(done!=previous);
  for(xx=0;xx<1920;xx=xx+1)begin
   @(negedge eth);x=xx;repeat(2)@(negedge eth);
   expected=value((28'd3<<22)+yy*960+(xx/16)*8,xx%16);
   if(pixel!==expected)$fatal(1,"raw edge pixel row=%0d x=%0d",yy,xx);
  end
 end endtask
 integer before;
 initial begin
  repeat(6)@(negedge eth);rst=1;nrst=1;hold=1;wait(ready);
  if(frame!=123||bank!=3)$fatal(1,"bad lease provenance");
  line(0);line(1079);if(requests!=240)$fatal(1,"expected 120 DDR beats per row");
  before=requests;@(negedge eth);row=1080;req=~req;repeat(80)@(negedge eth);
  if(requests!=before)$fatal(1,"invalid row generated DDR read");
  // Cancel a live burst: retain ownership until its response/RAM writes drain.
  @(negedge eth);row=400;req=~req;wait(pending);@(negedge eth);hold=0;
  wait(!leased);if(dut.state!=0||pending)$fatal(1,"lease released with outstanding read");
  wait(!ready);@(negedge eth);hold=1;wait(ready);
  line(1078); // request phase now differs from completion phase after cancel
  // PHY reset also resets both mailbox ends and releases the raw bank.
  @(negedge eth);nrst=0;repeat(10)@(negedge eth);
  if(leased||ready)$fatal(1,"network reset did not release lease");
  hold=0;nrst=1;repeat(30)@(negedge eth);
  if(ready)$fatal(1,"old request replayed after network reset");
  $display("PASS 1920x1080 raw boundary: 5760 exact pixels, final bank/row/column, invalid row, odd-phase cancellation/handoff and PHY-reset lease release");$finish;
 end
 initial begin #10000000;$fatal(1,"raw edge watchdog state=%0d req=%b seen=%b done=%b requests=%0d hold=%b leased=%b pending=%b",dut.state,req,dut.seen,done,requests,hold,leased,pending);end
endmodule
