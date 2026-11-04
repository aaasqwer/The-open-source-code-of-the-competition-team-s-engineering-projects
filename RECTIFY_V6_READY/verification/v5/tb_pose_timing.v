`timescale 1ns/1ps
module tb_capture_time;
 reg dc=0,cc=0;always #4 dc=~dc;always #11 cc=~cc;
 reg rst=0,vs=0,pv=0,le=0;reg [15:0] pixel=0;reg [31:0] tick=0;
 wire [31:0] tg=tick^(tick>>1);reg signed [15:0] pose=0;wire signed [15:0] applied_pose;wire [31:0] serial,start;wire [1:0] latest;wire valid,overflow,av;
 wire [27:0] addr;wire [3:0] aid,len;wire [255:0] data;wire [31:0] strb;
 reg [31:0] actual_first;integer x,y,id,lane,beats=0,bursts=0;reg [27:0] active_addr;reg [15:0] got[0:127];
 always @(posedge dc)if(!rst)tick<=0;else tick<=tick+1'b1;
 camera_frame_writer #(.W(32),.H(4),.COMMIT_GUARD(4),.POSE_SAMPLE_MID(1)) dut(.cam_clk(cc),.rst_n(rst),.vs(vs),.pixel_valid(pv),.line_end(le),.pixel(pixel),.frame_angle(pose),
  .ddr_clk(dc),.time_gray(tg),.latest_start_tick(start),.display_frame(2'd1),.network_frame_hold(1'b0),.network_frame(2'd0),
  .frame_serial(serial),.latest_frame(latest),.frame_valid(valid),.latest_frame_angle(applied_pose),.overflow(overflow),
  .awaddr(addr),.awid(aid),.awlen(len),.awvalid(av),.awready(1'b1),.wdata(data),.wstrb(strb),.wready(1'b1));
 always @(posedge dc)begin
  if(av)begin active_addr=addr;beats=0;bursts=bursts+1;end
  if(dut.state==3)begin
   for(lane=0;lane<16;lane=lane+1)got[((active_addr&28'h3fffff)+beats*8)*2+lane]=data[lane*16+:16];
   beats=beats+1;
  end
 end
 task sof;begin @(negedge cc);vs=1;@(negedge cc);vs=0;end endtask
 task image;input malformed;begin
  for(y=0;y<4;y=y+1)begin
   pose=y<2?100:y==2?200:300;
   for(x=0;x<32+(malformed&&y==1);x=x+1)begin
    @(negedge cc);pv=1;pixel=(y*32+x)^16'h1234;
    if(y==0&&x==0)begin @(posedge cc);#1;actual_first=tick;end
   end
   @(negedge cc);pv=0;le=1;@(negedge cc);le=0;repeat(4)@(negedge cc);
  end
 end endtask
 task check;input [31:0] wanted;begin
  wait(serial==wanted);#1;
  if(applied_pose!==200)$fatal(1,"frame must retain centre-row pose, not SOF/EOF/newest pose: %d",applied_pose);
  if(actual_first-start>10)$fatal(1,"camera timestamp not first pixel; actual=%d stamp=%d",actual_first,start);
  for(id=0;id<128;id=id+1)if(got[id] !== (id^16'h1234))$fatal(1,"pixel changed %0d",id);
 end endtask
 initial begin
  repeat(5)@(negedge dc);rst=1;repeat(5)@(negedge cc);
  sof;image(0);sof;check(1);image(1);sof;repeat(150)@(negedge dc);
  if(serial!=1)$fatal(1,"malformed frame published");
  image(0);sof;check(2);
  $display("PASS frame-owned centre-row IMU pose, first-pixel timestamp, malformed frame rejection and exact source pixels");$finish;
 end
 initial begin #1000000;$fatal(1,"capture timeout");end
endmodule
