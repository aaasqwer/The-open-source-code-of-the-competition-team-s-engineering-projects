`timescale 1ns/1ps
module tb_v5_parameter_overlay;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,hold_frame=0,enable=0,vs=0,hs=0,de=1;
 reg[11:0] x=0,y=0;reg[23:0]rgb=0;wire ov,oh,od;wire[11:0]ox,oy;wire[23:0]o;
 status_cn_overlay dut(.clk(clk),.rst_n(rst_n),.hold_frame(hold_frame),.enable(enable),.vs(vs),.hs(hs),.de(de),.x(x),.y(y),.rgb(rgb),
  .mean(8'd100),.contrast(8'd80),.gamma_step(5'd5),.bypass(1'b0),.lens(35'd0),.angles(64'd0),.fps(8'h15),.fps_valid(1'b1),
  .width_bcd(16'h1920),.height_bcd(16'h1080),.geometry_valid(1'b1),.geometry_bad(1'b0),.exposure(16'd400),.gain(10'd32),.light(8'd90),.light_error(8'd0),.light_valid(1'b1),
  .out_vs(ov),.out_hs(oh),.out_de(od),.out_x(ox),.out_y(oy),.out_rgb(o));
 reg[23:0]prev;reg[11:0]px,py;reg pe;integer i,phase,changed=0;reg[31:0]rng=32'h12345678;
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  for(phase=0;phase<2;phase=phase+1)begin
   for(i=0;i<40000;i=i+1)begin
    @(negedge clk);rng=rng*1664525+1013904223;x=i%1920;y=(i/1920)*8;rgb=rng[23:0];enable=phase;
    @(posedge clk);#1;
    if(i>1)begin
     if(ox!==px||oy!==py)$fatal(1,"pixel coordinate latency mismatch");
     if(o!==prev)begin
      if(!pe||px<16||px>=784||py<16||py>=164||o!==24'hffff00)$fatal(1,"changed background or non-reference glyph colour");
      changed=changed+1;
     end
    end
    prev=rgb;px=x;py=y;pe=enable;
   end
  end
  if(changed<10)$fatal(1,"no parameter glyphs painted");
  $display("PASS 80000 reference parameter pixels: disabled exact RGB, enabled yellow glyphs only, sync/coordinates aligned");$finish;
 end
endmodule
