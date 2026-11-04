`timescale 1ns/1ps
module tb_freeze_scan;
 parameter W=64,H=48,FIFO_BITS=5,PREFILL=16,PAUSE_CYCLES=200,STARVE=0;
 reg dc=0,pc=0,rst=0,vs=0,de=0;always #4 dc=~dc;always #6.734 pc=~pc;
 reg freeze=0;wire frozen;reg [2:0] latest=4;wire [2:0] bank;wire signed [15:0] applied;
 wire [63:0] display_origin;
 wire [63:0] displayed_geometry;wire [34:0] displayed_lens;wire [383:0] displayed_params;
 wire [27:0] addr;wire [3:0] len,id;wire av,od,uf,shown_fault;wire [15:0] pixel;
 reg ready=0,rv=0,last=0;reg [3:0] rid=0;reg [255:0] data=0;
 integer cycle=0,wp=0,rp=0,remaining=0,at=0,due=0,active_id=0,lane,tmp,frame_bank=4;
 integer qa[0:255],qn[0:255],qi[0:255],qt[0:255];
 integer count=0,expected,x,y,i,j,frame_no=0,pause_left=0,background_count=0;reg pause_once=0,background_seen=0;
 function [15:0] colour;input integer b,px,py;begin colour=(py*W+px)^(b*16'h1234);end endfunction
 video_frame_scan #(.W(W),.H(H),.FIFO_BITS(FIFO_BITS),.PREFILL_WORDS(PREFILL)) dut(
  .latest_origin({32'h1234,29'd0,latest}),.display_origin(display_origin),.latest_geometry({61'd0,latest}),.display_geometry(displayed_geometry),.latest_lens({29'd0,latest,3'b110}),.latest_lens_params({381'd0,latest}),.display_lens(displayed_lens),.display_lens_params(displayed_params),
.freeze_display(freeze),.freeze_active(frozen), .clk(dc),.rst_n(rst),.pix_clk(pc),.vs(vs),.de(de),.latest_bank(latest),.frame_valid(1'b1),
 .latest_angle(16'sd8192),.display_bank(bank),.applied_angle(applied),
 .araddr(addr),.arlen(len),.arid(id),.arvalid(av),.arready(ready),
 .rdata(data),.rid(rid),.rvalid(rv),.rlast(last),.out_de(od),.out_pixel(pixel),.underflow(uf)
`ifdef USE_FRAME_HISTORY
 ,.last_frame_underflow(shown_fault)
`endif
 );
`ifndef USE_FRAME_HISTORY
 assign shown_fault=uf;
`endif
 always @(posedge dc) begin
  cycle=cycle+1;rv<=0;last<=0;
  if(pause_once && pause_left==0) begin pause_left=PAUSE_CYCLES;pause_once=0;end
  if(pause_left>0) pause_left=pause_left-1;
  ready<=(wp-rp<8 && cycle%5!=0 && pause_left==0);
  if(av && ready) begin
   if(!id[3] || addr[24:22]<4) $fatal(1,"scan address/ID");
   qa[wp%256]=addr;qn[wp%256]=len+1;qi[wp%256]=id;qt[wp%256]=cycle+64;wp=wp+1;
  end
  if(remaining==0 && rp<wp) begin
   if(wp-rp>=2 && cycle%3!=0) begin
    tmp=qa[rp%256];qa[rp%256]=qa[(rp+1)%256];qa[(rp+1)%256]=tmp;
    tmp=qn[rp%256];qn[rp%256]=qn[(rp+1)%256];qn[(rp+1)%256]=tmp;
    tmp=qi[rp%256];qi[rp%256]=qi[(rp+1)%256];qi[(rp+1)%256]=tmp;
    tmp=qt[rp%256];qt[rp%256]=qt[(rp+1)%256];qt[(rp+1)%256]=tmp;
   end
   at=qa[rp%256];remaining=qn[rp%256];active_id=qi[rp%256];due=qt[rp%256];rp=rp+1;
  end
  if(remaining>0 && cycle>=due && cycle%7!=0 && pause_left==0) begin
   for(lane=0;lane<16;lane=lane+1) begin
    x=(((at&((1<<22)-1))*2)+lane)%W;y=(((at&((1<<22)-1))*2)+lane)/W;
    if(y>=H) $fatal(1,"scan outside output frame");
    data[lane*16+:16]<=colour(at>>22,x,y);
   end
   rv<=1;rid<=active_id;last<=remaining==1;remaining=remaining-1;at=at+8;
  end
 end
 always @(negedge pc) if(od) begin
  expected=colour(frame_bank,count%W,count/W);
  if(STARVE && frame_no==2 && pixel===16'h2104) begin background_seen=1;background_count=background_count+1;end
  else if(background_seen) $fatal(1,"starved frame resumed with mixed output");
  else if(pixel!==expected[15:0]) $fatal(1,"SCAN frame=%d pixel=%d got=%h expected=%h",frame_no,count,pixel,expected);
  count=count+1;
 end
 task frame;input integer b;
 begin
  frame_no=frame_no+1;frame_bank=b;latest=freeze?6:b;count=0;background_count=0;background_seen=0;
  @(negedge pc);vs=1;repeat(6) @(negedge pc);vs=0;
  repeat((W==1920)?90000:(STARVE?40000:500)) @(negedge pc);
  // This is exactly when the HUD takes its VSYNC+40000 pixel-clock snapshot.
  if(STARVE && shown_fault!==(frame_no==3)) $fatal(1,"HUD missed previous-frame fault: frame=%d shown=%b expected=%b",frame_no,shown_fault,frame_no==3);
  if(displayed_geometry!=b)$fatal(1,"geometry/bank mismatch");
  if(bank!=b) $fatal(1,"frame boundary selection");
  if(displayed_lens[34:3]!=b || displayed_params[2:0]!=b)$fatal(1,"lens metadata does not match selected/frozen frame bank");
  for(j=0;j<H;j=j+1) begin
   for(i=0;i<W;i=i+1) begin
    @(negedge pc);de=1;
    if(j==H/2 && i==W/2) begin latest=(b==7)?4:b+1;if(!STARVE || frame_no==2) pause_once=1;end
   end
   @(negedge pc);de=0;repeat((W==1920)?279:31) @(negedge pc);
  end
  repeat(8) @(negedge pc);
  if(count!=W*H || uf!==(STARVE && frame_no==2)) $fatal(1,"scan count/underflow %d uf=%b",count,uf);
  if(STARVE && frame_no==2 && (background_count==0 || background_count>=W*H)) $fatal(1,"did not exercise valid prefix and starvation suffix");
  $display("COMPLETE_BANK_SCAN_REORDER_PAUSE_VSYNC_PASS frame=%d pixels=%d",b,count);$fflush();
 end endtask
 initial begin #25;rst=1;frame(4);freeze=1;frame(4);if(!frozen)$fatal(1,"freeze ACK missing");frame(4);freeze=0;frame(5);frame(6);frame(7);
  if(STARVE) begin
   @(negedge pc);vs=1;repeat(8) @(negedge pc);vs=0;repeat(40000) @(negedge pc);
   if(shown_fault) $fatal(1,"history did not clear after a clean frame");
   $display("UNDERFLOW_HISTORY_VISIBLE_AT_HUD_SNAPSHOT_AND_RECOVERY_PASS");
  end
  $display("FRAME_SCAN_ALL_PASS");$finish;end
 initial begin #1000000000;$fatal(1,"scan watchdog");end
endmodule
