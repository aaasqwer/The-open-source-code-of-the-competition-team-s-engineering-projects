`timescale 1ns/1ps
module fast_snapshot_ctrl #(parameter CLK_HZ=125000000,WIDTH=1920,HEIGHT=1080,LEASED_DDR=0)(
 input clk,rst_n,capture_pulse,invalid_frame,output reg abort_capture,input [31:0] capture_sequence,
 output reg freeze_request,input freeze_active_vin,video_vs,
 input [1:0] mode_pix,input model_pix,input [31:0] leased_frame_id,input [63:0] leased_geometry,
 output reg line_request_toggle,input line_done_toggle_pix,input [10:0] captured_row_pix,
 output [10:0] line_rd_addr,input[15:0] line_rd_data,
 output uart_start,output [7:0] uart_data,input uart_busy,
 output packet_active,output reg transfer_active,
 output [3:0] capture_state,output [10:0] completed_rows,output reg [2:0] abort_reason);
 reg [3:0] state;reg [1:0] stable,kind,mode1,mode2,saved_mode;reg model1,model2,saved_model;
 reg fr1,fr2,vs1,vs2,vl,ld1,ld2,ld_seen;reg[10:0] row1,row2,row;
 reg [31:0] sequence_id,frame_id,watchdog,frame_digest;reg [63:0] saved_geometry;
 reg [10:0] rows_sent;reg packet_start,encoder_start;
 wire packet_busy,packet_done,enc_done,enc_busy,compressed;
 wire [12:0] encoded_length;wire[31:0] row_crc;
 wire [10:0] encoder_line_addr;wire[15:0] payload_addr;reg[7:0] payload;
 wire [7:0] encoded_byte;
 wire [11:0] encoded_addr=payload_addr>=8?payload_addr-8:0;
 assign capture_state=state;assign completed_rows=rows_sent;
 // Abort is a one-cycle reset for packet/encoder state, not a feedback reset
 // of this controller: retain the reason until the next capture or link reset.
 wire data_rst_n=rst_n&&!abort_capture;
 assign line_rd_addr=enc_busy?encoder_line_addr:(payload_addr>=8?(payload_addr-8)>>1:0);
 rgb565_line_encoder #(.WIDTH(WIDTH)) encoder(.clk(clk),.rst_n(data_rst_n),.start(encoder_start),.row(row),
  .busy(enc_busy),.done(enc_done),.compressed(compressed),.length(encoded_length),.row_crc(row_crc),
  .line_addr(encoder_line_addr),.line_data(line_rd_data),.encoded_addr(encoded_addr),.encoded_data(encoded_byte));
 wire[7:0] packet_type=kind==0?8'h81:kind==1?8'h85:8'h83;
 wire[15:0] payload_length=kind==0?(LEASED_DDR!=0?16'd32:16'd24):kind==1?encoded_length+16'd8:16'd8;
 assign packet_active=packet_start||packet_busy;
 uart_packet_tx packet(.clk(clk),.rst_n(data_rst_n),.start(packet_start),.packet_type(packet_type),.sequence(sequence_id),
  .payload_length(payload_length),.payload_addr(payload_addr),.payload_data(payload),.uart_start(uart_start),.uart_data(uart_data),
  .uart_busy(uart_busy),.busy(packet_busy),.done(packet_done),.payload_sent(),.payload_sent_data(),.payload_sent_addr());
 wire[31:0] total=WIDTH*HEIGHT*2;
 wire[15:0] width_bytes=WIDTH*2;
 always @*begin
  payload=0;
  if(kind==0)case(payload_addr)
   0:payload=2;1:payload=2;2:payload=WIDTH&255;3:payload=WIDTH>>8;4:payload=HEIGHT&255;5:payload=HEIGHT>>8;
   6:payload=width_bytes[7:0];7:payload=width_bytes[15:8];
   8,9,10,11:payload=total>>((payload_addr-8)*8);
   16,17,18,19:payload=frame_id>>((payload_addr-16)*8);
   20:payload=saved_mode;21:payload=saved_model;22:payload=2;23:payload=1;
   24,25,26,27,28,29,30,31:if(LEASED_DDR!=0)payload=saved_geometry>>((payload_addr-24)*8);
   default:payload=0;
  endcase
  else if(kind==1)case(payload_addr)
   0:payload=row[7:0];1:payload={5'd0,row[10:8]};2:payload=compressed;3:payload=0;
   4,5,6,7:payload=row_crc>>((payload_addr-4)*8);
   default:payload=compressed?encoded_byte:(payload_addr[0]?line_rd_data[15:8]:line_rd_data[7:0]);
  endcase
  else if(payload_addr<4)payload=total>>(payload_addr*8);else payload=frame_digest>>((payload_addr-4)*8);
 end
 always @(posedge clk)begin
  fr1<=freeze_active_vin;fr2<=fr1;vs1<=video_vs;vs2<=vs1;vl<=vs2;
  ld1<=line_done_toggle_pix;ld2<=ld1;row1<=captured_row_pix;row2<=row1;
  mode1<=mode_pix;mode2<=mode1;model1<=model_pix;model2<=model1;
  packet_start<=0;encoder_start<=0;
  if(!rst_n)begin state<=0;freeze_request<=0;transfer_active<=0;line_request_toggle<=0;
   fr1<=0;fr2<=0;vs1<=0;vs2<=0;vl<=0;ld1<=0;ld2<=0;ld_seen<=0;
   watchdog<=0;stable<=0;saved_geometry<=0;frame_id<=0;rows_sent<=0;kind<=0;frame_digest<=0;row<=0;sequence_id<=0;abort_capture<=0;abort_reason<=0;
  end else begin
   // Two seconds WITHOUT PROGRESS, not two seconds for the whole image.
   // Reliable UART ACKs deliberately trade photo speed for retransmission.
   if(state==0 || (uart_start&&!uart_busy) || packet_done || enc_done ||
      (state==5&&ld2!=ld_seen) || (state==1&&fr2&&(LEASED_DDR!=0||(vs2&&!vl))))watchdog<=0;
   else watchdog<=watchdog+1'b1;
   abort_capture<=0;
   if(watchdog>=CLK_HZ*2 || (invalid_frame && state!=0 && state!=1 && state!=10))begin
    freeze_request<=0;transfer_active<=0;state<=0;watchdog<=0;abort_capture<=1;abort_reason<=watchdog>=CLK_HZ*2?3'd2:3'd1;
   end
   else case(state)
    0:if(capture_pulse)begin sequence_id<=capture_sequence;frame_id<=frame_id+1'b1;
     freeze_request<=1;transfer_active<=1;stable<=0;frame_digest<=0;rows_sent<=0;abort_reason<=0;state<=1;end
    1:if(fr2&&(LEASED_DDR!=0||(vs2&&!vl)))begin
     if(stable==2)begin saved_geometry<=leased_geometry;if(LEASED_DDR!=0)frame_id<=leased_frame_id;saved_mode<=mode2;saved_model<=model2;kind<=0;state<=2;end
     else stable<=stable+1'b1;
    end
    2:if(!packet_busy)begin packet_start<=1;state<=3;end
    3:if(packet_done)begin
     if(kind==2)begin freeze_request<=0;state<=10;end else state<=4;
    end
    4:begin ld_seen<=ld2;line_request_toggle<=~line_request_toggle;state<=5;end
    5:if(ld2!=ld_seen)state<=6;
    6:begin row<=row2;state<=7;end
    7:begin encoder_start<=1;state<=8;end
    8:if(enc_done)begin kind<=1;frame_digest<=frame_digest^row_crc;rows_sent<=rows_sent+1'b1;state<=9;end
    9:begin state<=2;if(rows_sent==HEIGHT)state<=11;end
    10:if(!fr2)begin transfer_active<=0;state<=0;end
    11:begin state<=2;end
    default:state<=0;
   endcase
   // The final row must be sent before switching to DONE.
   if(state==3&&packet_done&&kind==1&&rows_sent==HEIGHT)begin kind<=2;state<=2;end
  end
 end
endmodule
