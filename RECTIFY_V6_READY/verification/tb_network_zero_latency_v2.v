`timescale 1ns/1ps
module tb_network_zero_latency;
 reg clk=0,eth=0,rst=0,hold=0,req=0,pv=0;always #4 clk=~clk;always #5 eth=~eth;
 wire ready,done,av,nready,nrv,prv,pready,reserve,valid;wire [27:0] na,addr;wire [3:0] ni,nl,id,len;wire [15:0] pixel;
 reg [10:0] x=0;wire [255:0] data;integer commands=0,renderer_replies=0,i;reg old_done;
 genvar lane;generate for(lane=0;lane<16;lane=lane+1)begin:g
  assign data[lane*16+:16]=(addr[15:0]*2+lane)^16'h1122;
 end endgenerate
 network_pair_reader #(.W(32),.H(1)) reader(.clk(clk),.rst_n(rst),.net_clk(eth),.net_rst_n(rst),.hold_frame(hold),.request_toggle(req),
  .request_row(11'd0),.read_x(x),.read_pixel(pixel),.ready(ready),.done_toggle(done),.frame_id(),.source_valid(1'b1),.latest_bank(2'd2),.latest_id(32'd1),
  .lease_active(),.leased_bank(),.araddr(na),.arlen(nl),.arid(ni),.arvalid(av),.arready(nready),.rdata(data),.rvalid(nrv),.rlast(1'b1));
 network_read_arbiter arbiter(.clk(clk),.rst_n(rst),.p_addr(28'd0),.n_addr(na),.p_id(4'd7),.p_len(4'd0),.n_id(ni),.n_len(nl),
  .p_valid(pv),.n_valid(av),.p_ready(pready),.n_ready(nready),.reserve_net_id(reserve),.addr(addr),.id(id),.len(len),.valid(valid),.ready(1'b1),
  .rid(id),.rvalid(valid),.rlast(1'b1),.p_rvalid(prv),.n_rvalid(nrv));
 always @(posedge clk)if(rst)begin if(valid)commands=commands+1;if(prv)renderer_replies=renderer_replies+1;end
 initial begin
  repeat(6)@(negedge clk);rst=1;pv=1;@(negedge clk);pv=0;hold=1;wait(ready);
  @(negedge eth);old_done=done;req=1;wait(done!=old_done);
  for(i=0;i<32;i=i+1)begin
   @(negedge eth);x=i;@(negedge eth);if(pixel!==(i^16'h1122))$fatal(1,"zero latency pixel mismatch");
  end
  if(commands!=3||renderer_replies!=1||arbiter.net_owner||arbiter.p7_outstanding)$fatal(1,"same-edge command/response ownership not drained");
  $display("PASS same-edge AR/R completion for borrowed network ID7 and renderer ID7, 32 exact pixels");$finish;
 end
 initial begin #100000;$fatal(1,"zero latency timeout");end
endmodule
