`timescale 1ns/1ps
module tb_dynamic_parameters;
reg clk=0;always #5 clk=~clk;
reg rst_n=0,hold_frame=0,enable=1,vs=0,hs=0,de=0;
reg [11:0] x=0,y=0;reg [23:0] rgb=24'h123456;
reg [7:0] mean=0,contrast=0,fps=0,light=0,light_error=0;
reg [4:0] gamma_step=0;reg bypass=0;
reg [34:0] lens=0;reg [63:0] angles=0;
reg fps_valid=0,geometry_valid=0,geometry_bad=0,light_valid=0;
reg [15:0] width_bcd=0,height_bcd=0,exposure=0;reg [9:0] gain=0;
status_cn_overlay dut(.clk(clk),.rst_n(rst_n),.hold_frame(hold_frame),.enable(enable),
 .vs(vs),.hs(hs),.de(de),.x(x),.y(y),.rgb(rgb),.mean(mean),.contrast(contrast),
 .gamma_step(gamma_step),.bypass(bypass),.lens(lens),.angles(angles),
 .fps(fps),.fps_valid(fps_valid),.width_bcd(width_bcd),.height_bcd(height_bcd),
 .geometry_valid(geometry_valid),.geometry_bad(geometry_bad),.exposure(exposure),.gain(gain),
 .light(light),.light_error(light_error),.light_valid(light_valid));
integer checks=0;
task frame;
 begin @(negedge clk);x=0;y=0;de=1;@(posedge clk);#1;de=0;end
endtask
task glyph;
 input integer row,col,code;
 begin @(negedge clk);x=16+16*col;y=16+22*row;de=1;#1;
 if(dut.code!==code[6:0]) $fatal(1,"row %0d col %0d expected %0d actual %0d",row,col,code,dut.code);
 checks=checks+1;end
endtask
task expect_a;
 begin
 glyph(1,4,11);glyph(1,5,2);glyph(1,11,12);glyph(1,12,3);
 glyph(1,18,2);glyph(1,19,8);glyph(1,24,2);
 glyph(2,6,2);glyph(2,11,1);glyph(2,16,2);glyph(2,21,9);glyph(2,28,16);
 glyph(3,7,2);glyph(3,10,5);glyph(3,15,6);glyph(3,23,10);glyph(3,31,14);glyph(3,34,1);
 glyph(4,6,3);glyph(4,7,5);glyph(4,12,2);glyph(4,13,3);glyph(4,14,9);glyph(4,15,1);
 glyph(4,17,1);glyph(4,18,8);glyph(4,19,3);glyph(4,20,1);glyph(4,25,2);
 glyph(5,4,2);glyph(5,5,4);glyph(5,6,6);glyph(5,7,8);
 glyph(5,13,3);glyph(5,14,5);glyph(5,15,7);glyph(5,21,6);glyph(5,22,11);glyph(5,27,2);glyph(5,28,4);
 end
endtask
task expect_b;
 begin
 glyph(1,4,4);glyph(1,5,13);glyph(1,11,5);glyph(1,12,14);
 glyph(1,18,1);glyph(1,19,7);glyph(1,24,1);
 glyph(2,6,1);glyph(2,11,2);glyph(2,16,1);glyph(2,21,8);glyph(2,28,15);
 glyph(3,7,16);glyph(3,10,13);glyph(3,15,12);glyph(3,23,8);glyph(3,31,4);glyph(3,34,1);
 glyph(4,6,1);glyph(4,7,10);glyph(4,12,1);glyph(4,13,7);glyph(4,14,5);glyph(4,15,1);
 glyph(4,17,1);glyph(4,18,5);glyph(4,19,9);glyph(4,20,1);glyph(4,25,1);
 glyph(5,4,3);glyph(5,5,5);glyph(5,6,7);glyph(5,7,9);
 glyph(5,13,2);glyph(5,14,6);glyph(5,15,8);glyph(5,21,12);glyph(5,22,7);glyph(5,27,1);glyph(5,28,6);
 end
endtask
initial begin
 repeat(3)@(posedge clk);@(negedge clk);rst_n=1;
 mean=8'ha1;contrast=8'hb2;gamma_step=5'h17;bypass=1;lens={32'h89abcdef,3'b101};angles=64'h123456789abcdef0;
 fps=8'h24;fps_valid=1;width_bcd=16'h1280;height_bcd=16'h0720;geometry_valid=1;geometry_bad=1;
 exposure=16'h1357;gain=10'h246;light=8'h5a;light_error=8'h13;light_valid=1;
 frame;expect_a;
 mean=8'h3c;contrast=8'h4d;gamma_step=5'h06;bypass=0;lens={32'h7654321e,3'b010};angles=64'hfedcba9876543210;
 fps=8'h09;width_bcd=16'h0640;height_bcd=16'h0480;geometry_bad=0;
 exposure=16'h2468;gain=10'h157;light=8'hb6;light_error=8'h05;
 // Mid-frame input changes must not tear the visible numbers.
 expect_a;frame;expect_b;
 // Frozen photo keeps its corresponding values until unfrozen.
 hold_frame=1;mean=8'h00;contrast=8'hff;fps=8'h99;gamma_step=5'h00;
 frame;expect_b;hold_frame=0;frame;
 glyph(1,4,1);glyph(1,5,1);glyph(1,11,16);glyph(1,12,16);glyph(1,19,1);glyph(4,6,10);glyph(4,7,10);
 fps_valid=0;geometry_valid=0;light_valid=0;frame;
 glyph(4,6,17);glyph(4,7,17);glyph(4,12,17);glyph(4,15,17);glyph(4,17,17);glyph(4,20,17);
 glyph(5,4,17);glyph(5,7,17);glyph(5,13,17);glyph(5,15,17);glyph(5,21,17);glyph(5,22,17);
 $display("PASS dynamic parameter glyphs %0d checks, frame latching, freeze and invalid markers",checks);$finish;
end
endmodule
