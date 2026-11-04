`timescale 1ns/1ps
module tb_live_roi;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,vs=0,de=0;reg[15:0]pixel=0;
 reg enabled=1,horizontal=0,editing=0;reg[2:0]position=0;reg[1:0]mode=1;reg[10:0]center_x=192,center_y=192;
 wire[3:0]status,sfr_status;wire[15:0]overlay_pixel,rms_q8,max_q8,mtf50_q16,sfr_quarter_q12,sfr_frame_rms,sfr_angle_q8;
 wire measurement_valid,endpoint_peak,graph_visible,sfr_graph_visible,sfr_valid,sfr_above,sfr_info_valid;wire[8:0]used_rows;wire[7:0]worst_row,coverage_percent;wire[2:0]sfr_samples;
 roi_evaluator dut(.*);
 reg[15:0]straight[0:32767],bowed[0:32767];integer x,y,f,phase=0,nfit=0,nsfr=0,cycle=0,first_fit=0,first_sfr=0;
 reg last_fit=0,last_sfr=0;
 always @(posedge clk)begin
  cycle=cycle+1;
  if(measurement_valid&&!last_fit)begin
   nfit=nfit+1;if(!first_fit)first_fit=cycle;
   if(phase==0&&rms_q8>64)$fatal(1,"straight ROI RMS %0d",rms_q8);
   if(phase==1&&rms_q8<180)$fatal(1,"bowed ROI flattened %0d",rms_q8);
  end
  if(sfr_valid&&!last_sfr)begin nsfr=nsfr+1;if(!first_sfr)first_sfr=cycle;end
  last_fit=measurement_valid;last_sfr=sfr_valid;
 end
 task frame;
 begin
  @(negedge clk);vs=1;de=0;repeat(8)@(negedge clk);vs=0;
  for(y=0;y<400;y=y+1)begin
   for(x=0;x<640;x=x+1)begin
    @(negedge clk);de=1;
    if(x>=128&&x<256&&y>=64&&y<320)pixel=phase?bowed[(y-64)*128+x-128]:straight[(y-64)*128+x-128];else pixel=16'h6b4d;
   end
   @(negedge clk);de=0;repeat(3)@(negedge clk);
  end
  repeat(16)@(negedge clk);
 end endtask
 initial begin
  $readmemh("verification/v4/roi_straight.hex",straight);$readmemh("verification/v4/roi_bowed.hex",bowed);
  repeat(6)@(negedge clk);rst_n=1;
  for(f=0;f<5;f=f+1)frame();
  if(nfit<2||nsfr<1)$fatal(1,"no fast valid ROI fit=%0d SFR=%0d status=%0d sf=%0d state=%0d",nfit,nsfr,status,sfr_status,dut.state);
  if(first_fit>520000||first_sfr>780000)$fatal(1,"first result delayed fit=%0d sfr=%0d",first_fit,first_sfr);
  $display("PASS first straight fit at %0d cycles; SFR at %0d; fits=%0d SFR=%0d",first_fit,first_sfr,nfit,nsfr);
  @(negedge clk);editing=1;repeat(5)@(negedge clk);phase=1;editing=0;nfit=0;nsfr=0;
  for(f=0;f<5;f=f+1)frame();
  if(nfit<2||nsfr!=0)$fatal(1,"curved acceptance fit=%0d SFR=%0d",nfit,nsfr);
  $display("PASS bowed edge measured without flattening; invalid SFR rejected, independent capture survives intervening VS");$finish;
 end
 initial begin #40000000;$fatal(1,"ROI watchdog");end
endmodule
