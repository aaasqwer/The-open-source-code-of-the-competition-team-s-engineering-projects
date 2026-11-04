`timescale 1ns/1ps
`include "calibration/profile_config.vh"
module tb_v6_mapping;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,start=0,enhanced=0;reg signed[23:0]u=0,v=0;
 localparam[383:0]DEFAULT=`RECTIFY_DEFAULT_PARAMS;
 reg[319:0]calibration;wire busy,done,bad;wire signed[23:0]x,y;
 rect_brown_node dut(.clk(clk),.rst_n(rst_n),.start(start),.enhanced(enhanced),.ideal_x(u),.ideal_y(v),.calibration(calibration),.busy(busy),.done(done),.bad(bad),.source_x(x),.source_y(y));
 reg[99:0]vectors[0:1185-1];integer k,cycles,maximum=0;
 initial begin
  $readmemh("verification/v6/node_vectors.hex",vectors);
  repeat(5)@(negedge clk);rst_n=1;
  for(k=0;k<1185;k=k+1)begin
   calibration=DEFAULT[319:0];enhanced=vectors[k][97:96]!=0;
   if(vectors[k][97:96]==2)calibration[128+:32]=calibration[128+:32]+1;
   u=vectors[k][95:72];v=vectors[k][71:48];start=1;@(negedge clk);start=0;cycles=0;
   while(!done)begin @(negedge clk);cycles=cycles+1;if(cycles>1200)$fatal(1,"node timeout %0d",k);end
   if(bad||x!==vectors[k][47:24]||y!==vectors[k][23:0])$fatal(1,"node mismatch %0d mode=%0d got=(%0d,%0d) expected=(%0d,%0d)",k,vectors[k][97:96],x,y,$signed(vectors[k][47:24]),$signed(vectors[k][23:0]));
   if(cycles>maximum)maximum=cycles;
   @(negedge clk);
  end
  $display("PASS 1185 exact Brown/shared cubic/radial nodes; foreign-profile guard; max cycles=%0d",maximum);$finish;
 end
 initial begin #100000000;$fatal(1,"watchdog");end
endmodule
