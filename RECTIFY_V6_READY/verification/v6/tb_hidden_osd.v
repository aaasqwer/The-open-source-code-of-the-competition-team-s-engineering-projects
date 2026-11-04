`timescale 1ns/1ps
module tb_v6_hidden_osd;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;reg[11:0]x=0,y=0;reg[23:0]rgb=0;reg vs=0,hs=0,de=1;
 wire pv,ph,pd;wire[11:0]px,py;wire[23:0]pr;
 status_cn_overlay params(.clk(clk),.rst_n(rst_n),.hold_frame(1'b0),.enable(1'b0),.vs(vs),.hs(hs),.de(de),.x(x),.y(y),.rgb(rgb),
 .mean(8'd10),.contrast(8'd24),.gamma_step(5'd5),.bypass(1'b0),.lens(35'd0),.angles(64'd0),.fps(8'h15),.fps_valid(1'b1),
 .width_bcd(16'h1920),.height_bcd(16'h1080),.geometry_valid(1'b1),.geometry_bad(1'b0),.exposure(16'd400),.gain(10'd32),.light(8'd20),.light_error(8'd0),.light_valid(1'b1),
 .out_vs(pv),.out_hs(ph),.out_de(pd),.out_x(px),.out_y(py),.out_rgb(pr));
 wire ov,oh,od;wire[11:0]ox,oy;wire[23:0]o;wire[15:0]o565;
 imu_hud_overlay #(.RGB888_INPUT(1),.REFLOW_PAGE(1)) hud(.pix_clk(clk),.rst_n(rst_n),.hold_frame(1'b0),.osd_enable(1'b0),.data_page(1'b0),
 .video_de(pd),.video_vs(pv),.video_hs(ph),.pix_x(px),.pix_y(py),.rgb_in(16'd0),.rgb888_in(pr),.telemetry(640'd0),
 .rgb_out(o565),.rgb888_out(o),.de_out(od),.vs_out(ov),.hs_out(oh),.out_x(ox),.out_y(oy));
 function[23:0]pattern;input[11:0]xx,yy;reg[7:0]a,b,c;
 begin a=xx[7:0]^yy[7:0];b=xx[7:0]+yy[7:0];c=yy[7:0]*7;pattern={a,b,c};end endfunction
 integer i;
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  for(i=0;i<100000;i=i+1)begin
   @(negedge clk);x=i%1920;y=((i/1920)*23)%1080;rgb=pattern(x,y);
   @(posedge clk);#1;if(i>12&&od&&o!==pattern(ox,oy))$fatal(1,"hidden parameter chain changed RGB %h vs %h at %d,%d",o,pattern(ox,oy),ox,oy);
  end
  $display("PASS hidden top parameters and IMU HUD: 100000 exact RGB888 background pixels, including top/bottom parameter areas");$finish;
 end
 initial begin #2000000;$fatal(1,"watchdog");end
endmodule
