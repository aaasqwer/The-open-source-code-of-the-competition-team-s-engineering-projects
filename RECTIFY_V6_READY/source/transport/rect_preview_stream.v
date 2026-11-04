`timescale 1ns/1ps
// Pixel-center 3:2 bilinear reduction of an already corrected complete frame.
// Required Brown geometry is upstream; this removes nearest-sampling aliasing.
module rect_preview_stream #(parameter WIDTH=1280,HEIGHT=720,PROCESSED=0,LAYOUT_TAG=0)(
 input clk,rst_n,enable,input [31:0] session,
 output reg hold_frame,request_toggle,output reg [10:0] request_row,output [10:0] read_x,
 input ready,done_toggle,frame_model,pair_valid,input [31:0] frame_id,input [63:0] frame_geometry,
 input [15:0] pixel,pixel_next,pixel_previous,pixel_previous_next,
 output busy,tx_en,output [7:0] tx_data
);
 reg [3:0] state;reg [10:0] row,issue_x,source_x;reg seen,enc_start,tx_start;
 reg [31:0] previous_id,saved_id,saved_session;reg have_previous,saved_model;reg [1:0] saved_mode;
 wire enc_busy,enc_done,compressed,tx_busy,tx_done;
 wire [12:0] length;wire [31:0] row_crc;wire [10:0] enc_x;wire [11:0] src_addr;wire [7:0] encoded_byte;
 reg src_high;reg [15:0] line_memory[0:WIDTH-1];reg [15:0] line_pixel;
 wire cancel_frame=!enable||session!=saved_session||!ready;
 assign busy=state!=0;assign read_x=source_x;
 always @(posedge clk)line_pixel<=line_memory[enc_busy?enc_x:src_addr[11:1]];
 preview_line_encoder #(.WIDTH(WIDTH)) encoder(.clk(clk),.rst_n(rst_n),.start(enc_start),.row(row),
  .write_slot(1'b0),.read_slot(1'b0),.busy(enc_busy),.done(enc_done),.compressed(compressed),.length(length),.row_crc(row_crc),
  .line_addr(enc_x),.line_data(line_pixel),.encoded_addr(src_addr),.encoded_data(encoded_byte),.encoded_raw(1'b0));
 hd_preview_udp_tx #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) network(.clk(clk),.rst_n(rst_n),.start(tx_start),
  .session(saved_session),.frame(saved_id),.row(row),.mode(PROCESSED!=0?(LAYOUT_TAG!=0?saved_mode:2'd1):2'd0),.model(PROCESSED!=0?saved_model:1'b1),
  .compressed(compressed),.length(length),.row_crc(row_crc),.src_addr(src_addr),
  .src_data(compressed?encoded_byte:(src_high?line_pixel[15:8]:line_pixel[7:0])),
  .busy(tx_busy),.done(tx_done),.tx_en(tx_en),.tx_data(tx_data));
 function [7:0] horizontal;input [5:0] a,b;input odd;
  begin horizontal=odd?{2'd0,a}+{1'd0,b,1'b0}+{2'd0,b}:{1'd0,a,1'b0}+{2'd0,a}+{2'd0,b};end
 endfunction
 function [11:0] vertical;input [7:0] a,b;input odd;
  begin vertical=odd?{4'd0,a}+{3'd0,b,1'b0}+{4'd0,b}+12'd8:{3'd0,a,1'b0}+{4'd0,a}+{4'd0,b}+12'd8;end
 endfunction
 reg v0,v1,v2;reg [10:0] x0,x1,x2;reg [7:0] rt,rb,gt,gb,bt,bb;reg [11:0] rr,gg,bbv;
 always @(posedge clk)begin
  x0<=issue_x;x1<=x0;x2<=x1;
  rt<=horizontal({1'b0,pixel_previous[15:11]},{1'b0,pixel_previous_next[15:11]},x0[0]);
  rb<=horizontal({1'b0,pixel[15:11]},{1'b0,pixel_next[15:11]},x0[0]);
  gt<=horizontal(pixel_previous[10:5],pixel_previous_next[10:5],x0[0]);
  gb<=horizontal(pixel[10:5],pixel_next[10:5],x0[0]);
  bt<=horizontal({1'b0,pixel_previous[4:0]},{1'b0,pixel_previous_next[4:0]},x0[0]);
  bb<=horizontal({1'b0,pixel[4:0]},{1'b0,pixel_next[4:0]},x0[0]);
  rr<=vertical(rt,rb,row[0]);gg<=vertical(gt,gb,row[0]);bbv<=vertical(bt,bb,row[0]);
  if(v2&&state==12&&!cancel_frame)line_memory[x2]<={rr[8:4],gg[9:4],bbv[8:4]};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin v0<=0;v1<=0;v2<=0;end
  else if(cancel_frame)begin v0<=0;v1<=0;v2<=0;end
  else begin v0<=state==12&&issue_x<WIDTH;v1<=v0;v2<=v1;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;hold_frame<=0;request_toggle<=0;request_row<=0;row<=0;seen<=0;
   enc_start<=0;tx_start<=0;previous_id<=0;saved_id<=0;saved_session<=0;have_previous<=0;saved_model<=1;saved_mode<=0;src_high<=0;issue_x<=0;source_x<=0;end
  else begin
   enc_start<=0;tx_start<=0;src_high<=src_addr[0];
   case(state)
    0:if(enable&&!ready)begin hold_frame<=1;row<=0;saved_session<=session;state<=1;end
    1:if(!enable||session!=saved_session)begin hold_frame<=0;state<=8;end
      else if(ready)begin
       if(have_previous&&frame_id==previous_id)begin hold_frame<=0;state<=8;end
       else begin saved_id<=frame_id;previous_id<=frame_id;have_previous<=1;saved_model<=frame_model;saved_mode<=frame_geometry[1:0]==3?2:frame_geometry[1:0]==0?0:1;state<=2;end
      end
    2:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else begin seen<=done_toggle;request_row<=row+(row>>1);request_toggle<=~request_toggle;state<=3;end
    3:if(cancel_frame)begin hold_frame<=0;state<=8;end else if(done_toggle!=seen)state<=9;
    9:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else begin seen<=done_toggle;request_row<=row+(row>>1)+1'b1;request_toggle<=~request_toggle;state<=10;end
    10:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else if(done_toggle!=seen)begin
       if(!pair_valid)begin hold_frame<=0;state<=8;end
       else begin issue_x<=0;source_x<=0;state<=12;end
      end
    12:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else begin
       if(issue_x<WIDTH)begin issue_x<=issue_x+1'b1;source_x<=source_x+(issue_x[0]?2:1);end
       if(v2&&x2==WIDTH-1)state<=4;
      end
    4:if(cancel_frame)begin hold_frame<=0;state<=8;end else begin enc_start<=1;state<=5;end
    5:if(enc_done)begin if(cancel_frame)begin hold_frame<=0;state<=8;end else begin tx_start<=1;state<=6;end end
    6:if(tx_done)begin
       if(row==HEIGHT-1||cancel_frame)begin hold_frame<=0;state<=8;end
       else begin row<=row+1'b1;state<=2;end
      end
    8:if(!ready)state<=0;
    default:state<=0;
   endcase
  end
 end
endmodule
