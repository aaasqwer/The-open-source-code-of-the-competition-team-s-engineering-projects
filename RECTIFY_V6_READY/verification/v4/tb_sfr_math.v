`timescale 1ns/1ps
module tb_sfr_math;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,cancel=0,start=0,fit_valid=1;
 reg signed[31:0]slope_q16,intercept_q16;reg[15:0]rms_q8=0;reg[8:0]rows=256;
 wire[14:0]rd_addr;reg[7:0]rd_data;wire busy,done,valid,curve_valid;wire[3:0]status;
 wire[15:0]mtf50_q16,sfr_quarter_q12,curve_q12;reg[5:0]curve_index=0;
 sfr_meter dut(.*);
 reg[7:0]pixels[0:294911];reg[31:0]slopes[0:8],intercepts[0:8];reg[15:0]expected[0:8];integer test=0,cycles=0,error;
 always @(posedge clk)rd_data<=pixels[test*32768+rd_addr];
 task launch;begin @(negedge clk);start=1;@(negedge clk);start=0;wait(done);@(negedge clk);end endtask
 initial begin
  $readmemh("verification/v4/sfr_pixels.hex",pixels);$readmemh("verification/v4/sfr_slopes.hex",slopes);
  $readmemh("verification/v4/sfr_intercepts.hex",intercepts);$readmemh("verification/v4/sfr_expected.hex",expected);
  repeat(5)@(negedge clk);rst_n=1;
  for(test=0;test<9;test=test+1)begin
   slope_q16=slopes[test];intercept_q16=intercepts[test];launch();
   error=$signed({1'b0,mtf50_q16})-$signed({1'b0,expected[test]});if(error<0)error=-error;
   if(!valid||error*100>expected[test]*7)$fatal(1,"SFR analytic error case=%0d status=%0d measured=%0d expected=%0d",test,status,mtf50_q16,expected[test]);
   $display("PASS SFR Gaussian case=%0d mtf50_q16=%0d expected=%0d",test,mtf50_q16,expected[test]);
  end
  test=0;slope_q16=0;launch();if(valid||status!=3)$fatal(1,"SFR axis-parallel edge accepted");
  slope_q16=slopes[0];rms_q8=256;launch();if(valid||status!=4)$fatal(1,"SFR bowed edge accepted");
  rms_q8=0;for(cycles=0;cycles<32768;cycles=cycles+1)pixels[cycles]=cycles%128<64?0:255;
  launch();if(valid||status!=5)$fatal(1,"SFR clipped edge accepted status=%0d",status);
  $display("PASS SFR invalid slant, bowing and clipping rejected");$finish;
 end
 initial begin #30000000;$fatal(1,"SFR watchdog");end
endmodule
