`timescale 1ns/1ps
// One frozen image, rows acquired in the order they become available.
// A completed row remains immutable until the next explicit request.
module fast_snapshot_line_buffer #(parameter H_ACTIVE=1920,V_ACTIVE=1080)(
 input pix_clk,pix_rst_n,video_vs,video_de,input [15:0] video_pixel,
 input request_toggle_sys,capture_epoch,output reg done_toggle_pix,
 output reg [10:0] captured_row_pix,input rd_clk,input [10:0] rd_addr,output reg [15:0] rd_data);
 reg [15:0] line_mem[0:H_ACTIVE-1] /* synthesis syn_ramstyle="block_ram" */;
 reg [V_ACTIVE-1:0] acquired;
 reg req1,req2,seen,ep1,ep2,ep_seen,vs_d,de_d,armed,capturing;
 reg [10:0] x,y;
 always @(posedge rd_clk)rd_data<=line_mem[rd_addr];
 always @(posedge pix_clk)begin
  req1<=request_toggle_sys;req2<=req1;ep1<=capture_epoch;ep2<=ep1;vs_d<=video_vs;de_d<=video_de;
  if(!pix_rst_n)begin req1<=0;req2<=0;seen<=0;ep1<=0;ep2<=0;ep_seen<=0;vs_d<=0;de_d<=0;
   acquired<=0;done_toggle_pix<=0;captured_row_pix<=0;x<=0;y<=0;armed<=0;capturing<=0;
  end else begin
   if(ep2!=ep_seen)begin ep_seen<=ep2;acquired<=0;armed<=0;capturing<=0;end
   if(req2!=seen)begin seen<=req2;armed<=1;capturing<=0;end
   if(video_vs&&!vs_d)begin x<=0;y<=0;end
   else begin
    if(video_de)begin
     if(x<H_ACTIVE && (capturing||(armed&&x==0&&!acquired[y])))begin
      line_mem[x]<=video_pixel;capturing<=1;
     end
     if(x<H_ACTIVE)x<=x+1'b1;
    end
    if(!video_de&&de_d)begin
     x<=0;
     if(capturing&&x==H_ACTIVE)begin
      captured_row_pix<=y;acquired[y]<=1;done_toggle_pix<=~done_toggle_pix;armed<=0;capturing<=0;
     end
     if(y<V_ACTIVE-1)y<=y+1'b1;
    end
   end
  end
 end
endmodule
