`timescale 1ns/1ps
// Four-packet Go-Back-N; PGUD V1 remains compatible. Receiver ACK n means
// contiguous CRC-checked chunks through n; it never ACKs a chunk beyond a gap.
module ethernet_photo_window #(parameter CHUNK_BYTES=1400,COALESCE_TICKS=62500,
 ACK_TICKS=12500000,MAX_RETRIES=20)(
 input clk,rst_n,capture,input [31:0] capture_sequence,
 input ack_update,input [63:0] ack,input image_start,input [7:0] image_data,
 input image_packet_active,output image_busy,image_pending,
 output reg abort,fault,tx_en,output reg [7:0] tx_data);
 localparam IDLE=0,PREAMBLE=1,FRAME=2,APPCRC=3,FCS=4,GAP=5;
 reg [2:0] state;reg [31:0] session,base_seq,write_seq,send_seq,sent_high;
 reg [10:0] count,lengths[0:3],n;reg pending;reg [7:0] pending_data;
 reg [15:0] idle_count;reg [31:0] timer;reg [4:0] retries;
 reg [12:0] index;reg [31:0] app_crc,frame_crc,afinal,efinal;
 reg app_valid,frame_valid;reg [7:0] app_data;
 reg [7:0] memory[0:8191] /* synthesis syn_ramstyle="block_ram" */;
 reg [7:0] memory_q;
 // A delayed original ACK can retire a slot during its retransmission.
 // Do not reuse that RAM until the retransmitted MAC frame has drained.
 wire full=(write_seq-base_seq)>=4 || (state!=IDLE&&write_seq[1:0]==send_seq[1:0]);
 wire ack_progress=ack_update&&ack[31:0]==session&&ack[63:32]>=base_seq&&ack[63:32]<sent_high;
 assign image_busy=pending||full||fault;
 assign image_pending=pending||count!=0||write_seq!=base_seq||state!=IDLE;
 wire [15:0] ip_length=48+n,udp_length=28+n;
 wire [31:0] ip_sum=32'h4500+ip_length+32'h4000+32'h4011+32'hc0a8+32'h010b+32'hffff+32'hffff;
 wire [16:0] ip_fold=ip_sum[15:0]+ip_sum[31:16];
 wire [15:0] ip_checksum=~(ip_fold[15:0]+ip_fold[16]);
 wire [10:0] read_addr=index-13'd57;
 always @(posedge clk)begin
  if(rst_n&&pending&&!full&&!fault)memory[{write_seq[1:0],count}]<=pending_data;
  memory_q<=memory[{send_seq[1:0],read_addr}];
 end
 function [31:0] crc_next;input [31:0] c;input [7:0] b;reg [31:0] v;integer k;
 begin v=c^b;for(k=0;k<8;k=k+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;crc_next=v;end endfunction
 wire [31:0] app_step=crc_next(app_crc,app_data),frame_step=crc_next(frame_crc,tx_data);
 function [7:0] frame_byte;input [12:0] i;
 begin case(i)
 0,1,2,3,4,5:frame_byte=255;6:frame_byte=2;7:frame_byte='h67;8:frame_byte='h60;9:frame_byte='h10;10:frame_byte=0;11:frame_byte=1;
 12:frame_byte=8;13:frame_byte=0;14:frame_byte='h45;15:frame_byte=0;
 16:frame_byte=ip_length[15:8];17:frame_byte=ip_length[7:0];18,19:frame_byte=0;20:frame_byte='h40;21:frame_byte=0;22:frame_byte=64;23:frame_byte=17;
 24:frame_byte=ip_checksum[15:8];25:frame_byte=ip_checksum[7:0];26:frame_byte=192;27:frame_byte=168;28:frame_byte=1;29:frame_byte=11;30,31,32,33:frame_byte=255;
 34,36:frame_byte='h1f;35,37:frame_byte='h90;38:frame_byte=udp_length[15:8];39:frame_byte=udp_length[7:0];40,41:frame_byte=0;
 42:frame_byte="P";43:frame_byte="G";44:frame_byte="U";45:frame_byte="D";
 46,47,48,49:frame_byte=session>>((i-46)*8);
 50,51,52,53:frame_byte=send_seq>>((i-50)*8);
 54:frame_byte=n[7:0];55:frame_byte={5'd0,n[10:8]};56:frame_byte=1;57:frame_byte=0;
 default:frame_byte=memory_q;
 endcase end endfunction
 integer j;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;session<=0;base_seq<=0;write_seq<=0;send_seq<=0;sent_high<=0;
   count<=0;n<=0;pending<=0;pending_data<=0;idle_count<=0;timer<=0;retries<=0;
   index<=0;app_crc<=0;frame_crc<=0;afinal<=0;efinal<=0;
   app_valid<=0;frame_valid<=0;app_data<=0;abort<=0;fault<=0;tx_en<=0;tx_data<=0;
   for(j=0;j<4;j=j+1)lengths[j]<=0;
  end else begin
   abort<=0;tx_en<=0;app_valid<=0;frame_valid<=0;
   if(app_valid)app_crc<=app_step;
   if(frame_valid)frame_crc<=frame_step;
   if(ack_progress)begin base_seq<=ack[63:32]+1'b1;timer<=0;retries<=0;end
   else if(base_seq!=sent_high&&timer<ACK_TICKS-1)timer<=timer+1'b1;
   else if(base_seq==sent_high)timer<=0;
   if(image_start&&!image_busy)begin pending<=1;pending_data<=image_data;idle_count<=0;end
   else if(pending&&!full&&!fault)begin
    pending<=0;idle_count<=0;
    if(count==CHUNK_BYTES-1)begin lengths[write_seq[1:0]]<=count+1'b1;write_seq<=write_seq+1'b1;count<=0;end
    else count<=count+1'b1;
   end else if(!image_packet_active&&count!=0&&!fault)begin
    if(COALESCE_TICKS==0||idle_count==COALESCE_TICKS-1)begin
     lengths[write_seq[1:0]]<=count;write_seq<=write_seq+1'b1;count<=0;idle_count<=0;
    end else idle_count<=idle_count+1'b1;
   end else idle_count<=0;
   case(state)
    IDLE:if(!fault)begin
     if(ack_progress)begin if(send_seq<=ack[63:32])send_seq<=ack[63:32]+1'b1;end
     else if(base_seq!=sent_high&&timer==ACK_TICKS-1)begin
      timer<=0;
      if(retries==MAX_RETRIES-1)begin
       abort<=1;fault<=1;count<=0;pending<=0;base_seq<=0;write_seq<=0;send_seq<=0;sent_high<=0;
      end else begin retries<=retries+1'b1;send_seq<=base_seq;end
     end else if(send_seq<base_seq)send_seq<=base_seq;
     else if(send_seq<write_seq)begin n<=lengths[send_seq[1:0]];index<=0;state<=PREAMBLE;end
    end
    PREAMBLE:begin
     tx_en<=1;tx_data<=index==7?8'hd5:8'h55;
     if(index==7)begin index<=0;app_crc<=32'hffffffff;frame_crc<=32'hffffffff;state<=FRAME;end
     else index<=index+1'b1;
    end
    FRAME:begin
     tx_en<=1;tx_data<=frame_byte(index);frame_valid<=1;
     if(index>=42)begin app_data<=frame_byte(index);app_valid<=1;end
     if(index==57+n)begin index<=0;state<=APPCRC;end else index<=index+1'b1;
    end
    APPCRC:begin
     tx_en<=1;frame_valid<=1;
     if(index==0)begin afinal<=~app_step;tx_data<=~app_step;end else tx_data<=afinal>>(index*8);
     if(index==3)begin index<=0;state<=FCS;end else index<=index+1'b1;
    end
    FCS:begin
     tx_en<=1;
     if(index==0)begin efinal<=~frame_step;tx_data<=~frame_step;end else tx_data<=efinal>>(index*8);
     if(index==3)begin index<=0;state<=GAP;if(send_seq>=sent_high)sent_high<=send_seq+1'b1;end
     else index<=index+1'b1;
    end
    GAP:if(index==11)begin index<=0;send_seq<=send_seq+1'b1;state<=IDLE;end else index<=index+1'b1;
    default:state<=IDLE;
   endcase
   if(capture&&!image_pending)begin
    session<=capture_sequence;base_seq<=0;write_seq<=0;send_seq<=0;sent_high<=0;
    timer<=0;retries<=0;fault<=0;idle_count<=0;
   end
  end
 end
endmodule
