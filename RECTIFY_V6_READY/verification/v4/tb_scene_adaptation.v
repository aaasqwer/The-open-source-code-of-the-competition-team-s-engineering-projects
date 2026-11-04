`timescale 1ns/1ps
module tb_scene_adaptation;
 reg clk=0;always #5 clk=~clk;reg rstn=0,hold_frame=0,runtime_bypass=0,vs_in=0,de_in=0;
 reg[15:0]rgb_in=0;wire[7:0]mean_luma,contrast;wire[4:0]tone_step;
 reg[4:0]previous=0;reg[7:0]held_mean;integer f,i,level;reg[7:0]v;
 adaptive_preprocess_osd #(.GAMMA_POLICY(3),.FRAME_PIXELS(256),.REQUIRE_VALID_PIXELS(1)) dut(
  .clk(clk),.rstn(rstn),.hold_frame(hold_frame),.runtime_bypass(runtime_bypass),.osd_enable(1'b0),
  .vs_in(vs_in),.hs_in(1'b0),.de_in(de_in),.pixel_valid(1'b1),.x_in(12'd0),.y_in(12'd0),.rgb_in(rgb_in),
  .rotation_trace(64'd0),.lens_state_live(35'd0),.camera_fps_bcd_live(8'd0),.camera_fps_valid_live(1'b0),
  .camera_width_bcd_live(16'd0),.camera_height_bcd_live(16'd0),.camera_geometry_valid_live(1'b0),.camera_geometry_bad_live(1'b0),
  .camera_light_valid_live(1'b1),.camera_light_index_live(8'd128),.camera_light_error_live(8'd0),.camera_exposure_lines_live(16'd100),
  .camera_gain_q4_live(10'd16),.camera_light_metric_q4_live(12'd1024),.mean_luma(mean_luma),.contrast(contrast),.tone_step(tone_step));
 always @(posedge clk)begin #1;
  if(rstn&&tone_step!=previous&&de_in)$fatal(1,"Gamma changed inside active pixels");previous=tone_step;
 end
 task frame;input integer base,n;begin
  @(negedge clk);vs_in=1;de_in=0;repeat(4)@(negedge clk);vs_in=0;repeat(16)@(negedge clk);
  for(i=0;i<n;i=i+1)begin @(negedge clk);de_in=1;v=base+(i%16);rgb_in={v[7:3],v[7:2],v[7:3]};end
  @(negedge clk);de_in=0;repeat(32)@(negedge clk);
 end endtask
 initial begin
  repeat(5)@(negedge clk);rstn=1;
  for(f=0;f<36;f=f+1)frame(24,256);
  if(tone_step!=5||mean_luma<20||mean_luma>45)$fatal(1,"No adaptive shadow lift step=%0d mean=%0d",tone_step,mean_luma);
  held_mean=mean_luma;hold_frame=1;for(f=0;f<5;f=f+1)frame(192,256);
  if(tone_step!=5||mean_luma!=held_mean)$fatal(1,"Held capture changed Gamma state");
  hold_frame=0;for(f=0;f<64;f=f+1)frame(192,256);
  if(tone_step!=0||mean_luma<185)$fatal(1,"Gamma did not relax on bright scene step=%0d mean=%0d",tone_step,mean_luma);
  for(f=0;f<16;f=f+1)frame(24,128);
  if(tone_step!=0)$fatal(1,"Partial frames changed scene exposure");
  $display("PASS raw-scene adaptive Gamma: dark->step5, held state fixed, bright->neutral, partial frames rejected, active-pixel curve stable");$finish;
 end
endmodule
