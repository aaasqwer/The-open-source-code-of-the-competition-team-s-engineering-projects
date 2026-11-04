`timescale 1ns/1ps
module tb_lens_pipeline;
reg clk=0;always #4 clk=~clk;
reg rst=0,valid=0;reg signed [25:0] x=0,y=0;
wire ov,nv,ob,nb,oi,ni;wire signed [25:0] ox,oy,nx,ny;
rect_lens_field_before old_d(.clk(clk),.rst_n(rst),.boot_start(1'b0),.enhanced(1'b0),.calibration(320'd0),
 .in_valid(valid),.ideal_x(x),.ideal_y(y),.out_valid(ov),.out_bad(ob),.source_x(ox),.source_y(oy),.source_inside(oi));
rect_lens_field new_d(.clk(clk),.rst_n(rst),.boot_start(1'b0),.enhanced(1'b0),.calibration(320'd0),
 .in_valid(valid),.ideal_x(x),.ideal_y(y),.out_valid(nv),.out_bad(nb),.source_x(nx),.source_y(ny),.source_inside(ni));
integer i,count=0,bad_count=0;reg [31:0] rng=32'h1653a29b;reg [35:0] value;
reg dv=0,db=0,di=0;reg signed [25:0] dx=0,dy=0;
always @(posedge clk)begin
 #1;
 if(rst)begin
  if(nv!==dv)$fatal(1,"valid delay mismatch");
  if(nv)begin
   if(nx!==dx||ny!==dy||nb!==db||ni!==di)$fatal(1,"coordinate/flag mismatch at %0d: %0d,%0d vs %0d,%0d",count,nx,ny,dx,dy);
   count=count+1;if(nb)bad_count=bad_count+1;
  end
  dv=ov;db=ob;di=oi;dx=ox;dy=oy;
 end else dv=0;
end
initial begin
 repeat(3)@(negedge clk);rst=1;
 force old_d.ready=1;force new_d.ready=1;
 // All parity banks and their small distributed-RAM spill have identical,
 // nonuniform signed Q8 values. This exercises parity, fractional phase,
 // edge clamping, out-of-domain flags and signed arithmetic.
 for(i=0;i<8192;i=i+1)begin
  rng=rng*1664525+1013904223;value={rng[17:0],rng[27:10]};
  old_d.m00[i]=value;new_d.m00[i]=value;
  old_d.m01[i]=value^36'h12b143d5;new_d.m01[i]=value^36'h12b143d5;
  old_d.m10[i]=value^36'h23c254e6;new_d.m10[i]=value^36'h23c254e6;
  old_d.m11[i]=value^36'h34d365f7;new_d.m11[i]=value^36'h34d365f7;
 end
 for(i=0;i<36;i=i+1)begin
  rng=rng*1664525+1013904223;value={rng[17:0],rng[27:10]};
  old_d.t00[i]=value;new_d.t00[i]=value;
  old_d.t01[i]=value^36'h12b143d5;new_d.t01[i]=value^36'h12b143d5;
  old_d.t10[i]=value^36'h23c254e6;new_d.t10[i]=value^36'h23c254e6;
  old_d.t11[i]=value^36'h34d365f7;new_d.t11[i]=value^36'h34d365f7;
 end
 for(i=0;i<20000;i=i+1)begin
  @(negedge clk);rng=rng*1664525+1013904223;valid=(i%7!=0);
  x=$signed({1'b0,rng[23:0]})%('sd1940*4096)-'sd10*4096;
  rng=rng*1664525+1013904223;y=$signed({1'b0,rng[23:0]})%('sd1100*4096)-'sd10*4096;
  if(i%11==0)begin x=1919*4096;y=1079*4096;end
  if(i%13==0)begin x=0;y=0;end
 end
 @(negedge clk);valid=0;repeat(15)@(negedge clk);
 if(count!=17142||bad_count<50)$fatal(1,"insufficient valid/bad coverage %0d %0d",count,bad_count);
 $display("PASS exact old/new 1080P mesh pipeline equivalence: %0d coordinates, %0d invalid-domain flags, one cycle only",count,bad_count);$finish;
end
endmodule
