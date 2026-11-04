`timescale 1ns/1ps
module tb_projective;
reg clk=0,rst=0,valid=0;always #4 clk=~clk;
reg [10:0] x=0,y=0;reg [287:0] matrix=0;reg [15:0] tag=0;
wire v0,v1,b0,b1;wire signed [25:0] x0,y0,x1,y1;wire [15:0] t0,t1;
reg [309:0] vectors[0:32767];integer i,seen=0,fp;
rect_projective_pipe #(.TW(16)) opt(.clk(clk),.rst_n(rst),.in_valid(valid),.x(x),.y(y),.in_tag(tag),.inverse_matrix(matrix),.out_valid(v0),.out_bad(b0),.ideal_x(x0),.ideal_y(y0),.out_tag(t0));
always @(negedge clk)if(v0)begin
 $fdisplay(fp,"%d %d %d %d",t0,b0,x0,y0);seen=seen+1;
end
initial begin
 $readmemh("verification/reciprocal_stress_vectors.hex",vectors);fp=$fopen("verification/reciprocal_stress_points.txt","w");#23;rst=1;
 for(i=0;i<32768;i=i+1)begin
  @(negedge clk);valid=0;
  if(i%7==3)repeat(3)@(negedge clk);
  @(negedge clk);{matrix,y,x}=vectors[i];tag=i;valid=1;
 end
 @(negedge clk);valid=0;repeat(40)@(negedge clk);
 if(seen!=32768)$fatal(1,"missing outputs");$fclose(fp);$display("PASS reciprocal 32768 coordinates with validity gaps");$finish;
end
endmodule
