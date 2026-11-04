`timescale 1ns/1ps
// Pull, encode, transmit. DDR and a leased camera bank absorb backpressure;
// there is no HDMI line deadline and no partially captured raster frame.
module raw_preview_stream #(parameter WIDTH=1280,HEIGHT=720,PROCESSED=0)(
 input clk,rst_n,enable,input [31:0] session,
 output reg hold_frame,output reg request_toggle,output [10:0] request_row,read_x,
 input ready,done_toggle,frame_model,input [31:0] frame_id,input [15:0] pixel,
 output busy,output tx_en,output [7:0] tx_data);
 reg [3:0] state;reg [10:0] row;reg seen,enc_start,tx_start;
 reg [31:0] previous_id,saved_id,saved_session;reg have_previous,saved_model;
 wire enc_busy,enc_done,compressed,tx_busy,tx_done;
 wire [12:0] length;wire [31:0] row_crc;
 wire [10:0] enc_x;wire [11:0] src_addr;wire [7:0] encoded_byte;
 reg src_high;
 wire cancel_frame=!enable||session!=saved_session||!ready;
 wire [10:0] source_x=enc_busy?enc_x:{1'b0,src_addr[11:1]};
 assign request_row=row+(row>>1);assign read_x=source_x+(source_x>>1);
 assign busy=state!=0;
 preview_line_encoder #(.WIDTH(WIDTH)) encoder(.clk(clk),.rst_n(rst_n),.start(enc_start),.row(row),
  .write_slot(1'b0),.read_slot(1'b0),.busy(enc_busy),.done(enc_done),.compressed(compressed),.length(length),.row_crc(row_crc),
  .line_addr(enc_x),.line_data(pixel),.encoded_addr(src_addr),.encoded_data(encoded_byte));
 hd_preview_udp_tx #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) network(.clk(clk),.rst_n(rst_n),.start(tx_start),
  .session(saved_session),.frame(saved_id),.row(row),.mode(PROCESSED!=0?2'd1:2'd0),.model(PROCESSED!=0?saved_model:1'b1),
  .compressed(compressed),.length(length),.row_crc(row_crc),.src_addr(src_addr),
  .src_data(compressed?encoded_byte:(src_high?pixel[15:8]:pixel[7:0])),
  .busy(tx_busy),.done(tx_done),.tx_en(tx_en),.tx_data(tx_data));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;hold_frame<=0;request_toggle<=0;row<=0;seen<=0;
   enc_start<=0;tx_start<=0;previous_id<=0;saved_id<=0;saved_session<=0;have_previous<=0;saved_model<=1;src_high<=0;end
  else begin
   enc_start<=0;tx_start<=0;src_high<=src_addr[0];
   case(state)
    0:if(enable&&!ready)begin hold_frame<=1;row<=0;saved_session<=session;state<=1;end
    1:if(!enable||session!=saved_session)begin hold_frame<=0;state<=8;end
     else if(ready)begin
     if(have_previous&&frame_id==previous_id)begin hold_frame<=0;state<=8;end
     else begin saved_id<=frame_id;previous_id<=frame_id;have_previous<=1;saved_model<=frame_model;state<=2;end
    end
    2:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else begin seen<=done_toggle;request_toggle<=~request_toggle;state<=3;end
    3:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else if(done_toggle!=seen)state<=4;
    4:if(cancel_frame)begin hold_frame<=0;state<=8;end
      else begin enc_start<=1;state<=5;end
    5:if(enc_done)begin
     if(cancel_frame)begin hold_frame<=0;state<=8;end
     else begin tx_start<=1;state<=6;end
    end
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
