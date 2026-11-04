`timescale 1ns/1ps
module tb_gamma_exact;
 reg clk=0;always #4 clk=~clk;reg rst=0;reg [15:0] rgb=0;reg [4:0] step=0;
 reg [11:0] x=0,y=0;reg de=0,vs=0,hs=0;wire [15:0] old_rgb,new_rgb;wire [23:0] old888,new888;
 wire od,ov,oh,nd,nv,nh;wire [11:0] ox,oy,nx,ny;reg [66:0] previous;
 integer i,s,n=0;
 gamma_rgb_reference #(.ENHANCED_CURVES(3)) ref_gamma(.clk(clk),.rstn(rst),.step(step),.rgb_in(rgb),.vs_in(vs),.hs_in(hs),.de_in(de),.x_in(x),.y_in(y),
  .rgb_out(old_rgb),.rgb888_out(old888),.vs_out(ov),.hs_out(oh),.de_out(od),.x_out(ox),.y_out(oy));
 gamma_rgb_preserve #(.ENHANCED_CURVES(3),.INPUT_PIPELINE(1)) dut(.clk(clk),.rstn(rst),.step(step),.rgb_in(rgb),.vs_in(vs),.hs_in(hs),.de_in(de),.x_in(x),.y_in(y),
  .rgb_out(new_rgb),.rgb888_out(new888),.vs_out(nv),.hs_out(nh),.de_out(nd),.x_out(nx),.y_out(ny));
 always @(posedge clk)begin #1;
  if(rst)begin
   if(n>5&&{nv,nh,nd,nx,ny,new_rgb,new888} !== previous)$fatal(1,"gamma changed sample %0d",n);
   previous={ov,oh,od,ox,oy,old_rgb,old888};n=n+1;
  end
 end
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(s=0;s<=16;s=s+1)for(i=0;i<65536;i=i+1)begin
   @(negedge clk);step=s;rgb=i;x=i&4095;y=s;de=i%19!=7;vs=i%4000<8;hs=i%128<4;
  end
  @(negedge clk);de=0;repeat(8)@(negedge clk);
  $display("PASS exact original Gamma pixels/RGB888/sync for 1114112 RGB565/step combinations; one extra clock only");$finish;
 end
endmodule
