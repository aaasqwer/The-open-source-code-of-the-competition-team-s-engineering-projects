`timescale 1ns/1ps
// 64x64 source cache: 16 x-lane RAM banks. Two read ports give the upper/lower
// row simultaneously; adjacent x taps are selected after the RAM read. Cache
// loading and pixel sampling are separate phases. 16 pixels loaded per beat.
module rect_tile_sampler #(parameter W=1920,H=1080)(
 input clk,rst_n,load_valid,input [7:0] load_address,input [255:0] load_data,
 input sample_valid,sample_good,input [25:0] source_x,source_y,
 input [10:0] bbox_x,bbox_y,input [8:0] sample_tag,
 output pixel_valid,output reg pixel_good,output [15:0] pixel,output reg [8:0] pixel_tag
);
 wire [10:0] ix=source_x[22:12],iy=source_y[22:12];
 wire [10:0] ix1=ix==W-1?ix:ix+1'b1,iy1=iy==H-1?iy:iy+1'b1;
 wire [5:0] lx=ix-bbox_x,lx1=ix1-bbox_x,ly=iy-bbox_y,ly1=iy1-bbox_y;
 wire [7:0] address0={ly,lx[5:4]},address1={ly1,lx[5:4]};
 // At a 16-pixel boundary the next x tap belongs to the next cache word.
 wire [7:0] next0={ly,lx1[5:4]},next1={ly1,lx1[5:4]};
 wire [15:0] top[0:15],bottom[0:15];
 genvar k;
 generate for(k=0;k<16;k=k+1)begin:lanes
  reg [15:0] memory[0:255];reg [15:0] q0,q1;
  wire use_next=(k==0&&lx[3:0]==15);
  always @(posedge clk)begin
   if(load_valid)memory[load_address]<=load_data[k*16+:16];
   else q0<=memory[use_next?next0:address0];
   q1<=memory[use_next?next1:address1];
  end
  assign top[k]=q0;assign bottom[k]=q1;
 end endgenerate
 reg [3:0] lane0,lane1;reg [11:0] fx0,fy0,fx1,fy1;
 reg [15:0] p00,p10,p01,p11;
 reg [1:0] valid_pipe,good_pipe;reg [8:0] tags0,tags1,tags2,tags3;
 reg good2,good3;
 always @(posedge clk)begin
  lane0<=lx[3:0];lane1<=lx1[3:0];fx0<=source_x[11:0];fy0<=source_y[11:0];fx1<=fx0;fy1<=fy0;
  p00<=top[lane0];p10<=top[lane1];p01<=bottom[lane0];p11<=bottom[lane1];
  tags0<=sample_tag;tags1<=tags0;tags2<=tags1;tags3<=tags2;pixel_tag<=tags3;
 end
 wire sampler_valid;
 bilinear_rgb565_pipe sampler(.clk(clk),.rst_n(rst_n),.in_valid(valid_pipe[1]),.p00(p00),.p10(p10),.p01(p01),.p11(p11),
  .fraction_x(fx1),.fraction_y(fy1),.out_valid(sampler_valid),.pixel(pixel));
 assign pixel_valid=sampler_valid;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin valid_pipe<=0;good_pipe<=0;good2<=0;good3<=0;pixel_good<=0;end
  else begin valid_pipe<={valid_pipe[0],sample_valid};good_pipe<={good_pipe[0],sample_good};
   good2<=good_pipe[1];good3<=good2;pixel_good<=good3;end
 end
endmodule
