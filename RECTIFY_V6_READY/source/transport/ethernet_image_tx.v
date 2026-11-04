`timescale 1ns/1ps
// IPv4 limited-broadcast UDP, port 8080. No ARP or PC address assignment.
// Images use Ethernet only; capture and ACK commands use the USB-UART.
// PGUD envelope: magic, capture u32, chunk u32, length u16, version u16,
// bytes[length], CRC32(envelope+bytes), all numeric envelope fields little-endian.
module ethernet_image_tx #(
 parameter CLK_HZ=125000000,parameter ACK_TICKS=12500000,parameter MAX_RETRIES=20,parameter REQUIRE_ACK=1,parameter CHUNK_BYTES=1024,parameter COALESCE_TICKS=0,parameter WINDOW=1
)(input clk,rst_n,input capture,input [31:0] capture_sequence,
 input ack_update,input [63:0] ack,
 input image_start,input [7:0] image_data,input image_packet_active,
 output image_busy,image_pending,output reg abort,output reg fault,
 output reg tx_en,output reg [7:0] tx_data);
 localparam COLLECT=0,APP_CRC=1,PREAMBLE=2,FRAME=3,FCS=4,GAP=5,WAIT_ACK=6,APP_CRC_FINISH=7;
 reg [2:0] state;
 reg [7:0] buffer[0:CHUNK_BYTES-1];
 reg [10:0] count,index;
 reg pending;reg [7:0] pending_data;
 reg [31:0] session,chunk,app_crc,app_crc_final,frame_crc,frame_crc_final;
 reg [31:0] timer;
 reg [15:0] collect_idle;
 reg [4:0] retries;
 reg ack_seen;
 reg [7:0] app_crc_byte;
 reg app_crc_pending,app_crc_last,frame_crc_pending;
 wire [15:0] app_length=20+count;
 wire [15:0] ip_length=28+app_length;
 wire [15:0] udp_length=8+app_length;
 wire [31:0] ip_sum=32'h00004500+ip_length+32'h4000+32'h4011+32'hc0a8+32'h010b+32'hffff+32'hffff;
 wire [16:0] ip_fold=ip_sum[15:0]+ip_sum[31:16];
 wire [15:0] ip_checksum=~(ip_fold[15:0]+ip_fold[16]);
 wire window_busy,window_pending,window_abort,window_fault,window_en;wire [7:0]window_data;
 assign image_busy=WINDOW==4?window_busy:(pending||state!=COLLECT);
 assign image_pending=WINDOW==4?window_pending:(pending||count!=0||state!=COLLECT);
 generate if(WINDOW==4)begin:fast
  ethernet_photo_window #(.CHUNK_BYTES(CHUNK_BYTES),.COALESCE_TICKS(COALESCE_TICKS),.ACK_TICKS(ACK_TICKS),.MAX_RETRIES(MAX_RETRIES)) engine(
   .clk(clk),.rst_n(rst_n),.capture(capture),.capture_sequence(capture_sequence),.ack_update(ack_update),.ack(ack),
   .image_start(image_start),.image_data(image_data),.image_packet_active(image_packet_active),.image_busy(window_busy),.image_pending(window_pending),
   .abort(window_abort),.fault(window_fault),.tx_en(window_en),.tx_data(window_data));
  always @*begin abort=window_abort;fault=window_fault;tx_en=window_en;tx_data=window_data;end
 end endgenerate
 function [31:0] crc_next;
 input [31:0] c;input [7:0] b;reg [31:0] v;integer k;
 begin v=c^b;for(k=0;k<8;k=k+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;crc_next=v;end
 endfunction
 // CRC consumes a registered byte. Header/payload selection is a separate
 // stage, avoiding a buffer-address -> byte mux -> CRC -> register path.
 wire [31:0] app_crc_step=crc_next(app_crc,app_crc_byte);
 wire [31:0] frame_crc_step=crc_next(frame_crc,tx_data);
 wire [31:0] completed_frame_crc=~frame_crc_step;
 function [7:0] app_byte;
 input [10:0] n;
 begin case(n)
 0:app_byte="P";1:app_byte="G";2:app_byte="U";3:app_byte="D";
 4,5,6,7:app_byte=session>>((n-4)*8);
 8,9,10,11:app_byte=chunk>>((n-8)*8);
 12:app_byte=count[7:0];13:app_byte={5'd0,count[10:8]};14:app_byte=REQUIRE_ACK?1:2;15:app_byte=0;
 default:if(n<16+count)app_byte=buffer[n-16];else app_byte=app_crc_final>>((n-16-count)*8);
 endcase end
 endfunction
 function [7:0] frame_byte;
 input [10:0] n;
 begin case(n)
 0,1,2,3,4,5:frame_byte=8'hff;
 6:frame_byte=8'h02;7:frame_byte=8'h67;8:frame_byte=8'h60;9:frame_byte=8'h10;10:frame_byte=0;11:frame_byte=1;
 12:frame_byte=8'h08;13:frame_byte=0;
 14:frame_byte=8'h45;15:frame_byte=0;16:frame_byte=ip_length[15:8];17:frame_byte=ip_length[7:0];
 18,19:frame_byte=0;20:frame_byte=8'h40;21:frame_byte=0;22:frame_byte=64;23:frame_byte=17;
 24:frame_byte=ip_checksum[15:8];25:frame_byte=ip_checksum[7:0];
 26:frame_byte=192;27:frame_byte=168;28:frame_byte=1;29:frame_byte=11;
 30,31,32,33:frame_byte=255;
 34,36:frame_byte=8'h1f;35,37:frame_byte=8'h90;
 38:frame_byte=udp_length[15:8];39:frame_byte=udp_length[7:0];40,41:frame_byte=0;
 default:frame_byte=app_byte(n-42);
 endcase end
 endfunction
 generate if(WINDOW!=4)begin:legacy
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 state<=COLLECT;count<=0;index<=0;pending<=0;pending_data<=0;session<=0;chunk<=0;
 app_crc<=0;app_crc_final<=0;frame_crc<=0;frame_crc_final<=0;timer<=0;retries<=0;
 abort<=0;fault<=0;tx_en<=0;tx_data<=0;ack_seen<=0;collect_idle<=0;
 app_crc_byte<=0;app_crc_pending<=0;app_crc_last<=0;frame_crc_pending<=0;
 end else begin
 abort<=0;tx_en<=0;
 app_crc_pending<=0;frame_crc_pending<=0;
 if(app_crc_pending)begin
  app_crc<=app_crc_step;
  if(app_crc_last)app_crc_final<=~app_crc_step;
 end
 if(frame_crc_pending)frame_crc<=frame_crc_step;
 if(ack_update&&ack=={chunk,session})ack_seen<=1;
 if(capture&&state==COLLECT&&count==0)begin session<=capture_sequence;chunk<=0;ack_seen<=0;fault<=0;end
 case(state)
 COLLECT:begin
  if(image_start&&!pending)begin pending<=1;pending_data<=image_data;collect_idle<=0;end
  else if(pending)begin
   collect_idle<=0;
   buffer[count]<=pending_data;pending<=0;count<=count+1'b1;
   if(count==CHUNK_BYTES-1)begin state<=APP_CRC;index<=0;app_crc<=32'hffffffff;ack_seen<=0;retries<=0;end
  end else if(!image_packet_active&&count!=0)begin
   if(COALESCE_TICKS==0||collect_idle==COALESCE_TICKS-1)begin
    state<=APP_CRC;index<=0;app_crc<=32'hffffffff;ack_seen<=0;retries<=0;collect_idle<=0;
   end else collect_idle<=collect_idle+1'b1;
  end else collect_idle<=0;
 end
 APP_CRC:begin
  app_crc_byte<=app_byte(index);app_crc_pending<=1;app_crc_last<=index==15+count;index<=index+1'b1;
  if(index==15+count)begin index<=0;state<=APP_CRC_FINISH;end
 end
 APP_CRC_FINISH:begin index<=0;state<=PREAMBLE;end
 PREAMBLE:begin
  tx_en<=1;tx_data<=index==7?8'hd5:8'h55;index<=index+1'b1;
  if(index==7)begin index<=0;state<=FRAME;frame_crc<=32'hffffffff;end
 end
 FRAME:begin
  tx_en<=1;tx_data<=frame_byte(index);frame_crc_pending<=1;index<=index+1'b1;
  if(index==41+app_length)begin index<=0;state<=FCS;end
 end
 // On FCS byte zero the last data byte is still in tx_data. Consume it and
 // transmit the completed low CRC byte on this same edge; no gap on the wire.
 FCS:begin tx_en<=1;tx_data<=index==0?completed_frame_crc[7:0]:(frame_crc_final>>(index[1:0]*8));index<=index+1'b1;
  if(index==0)frame_crc_final<=completed_frame_crc;
  if(index==3)begin index<=0;state<=GAP;end
 end
 GAP:begin index<=index+1'b1;if(index==11)begin
  if(REQUIRE_ACK)begin state<=WAIT_ACK;timer<=0;end
  else begin state<=COLLECT;count<=0;index<=0;chunk<=chunk+1'b1;ack_seen<=0;end
 end end
 WAIT_ACK:begin
  if(ack_seen)begin state<=COLLECT;count<=0;index<=0;chunk<=chunk+1'b1;ack_seen<=0;end
  else if(timer==ACK_TICKS-1)begin
   if(retries==MAX_RETRIES-1)begin state<=COLLECT;count<=0;pending<=0;abort<=1;fault<=1;chunk<=chunk+1'b1;end
   else begin retries<=retries+1'b1;index<=0;state<=PREAMBLE;end
  end else timer<=timer+1'b1;
 end
 default:state<=COLLECT;
 endcase
 end
 end endgenerate
endmodule
