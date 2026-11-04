`timescale 1ns/1ps
// Linear scan of a completed DDR frame. Eight tagged bursts reorder into a
// FIFO_BITS-sized FIFO (64 KiB in the board top); its exposed Gray write pointer advances ONE word per clock.
module video_frame_scan #(
 parameter W=1920,H=1080,ADDR_W=28,FRAME_BIT=22,FIFO_BITS=13,PREFILL_WORDS=6144,
 parameter [15:0] BACKGROUND=16'h2104
)(
 input freeze_display,output reg freeze_active,
 input clk,rst_n,pix_clk,vs,de,input [2:0] latest_bank,input frame_valid,
 input signed [15:0] latest_angle,output reg [2:0] display_bank,
 input [63:0] latest_origin,output reg [63:0] display_origin,
 input [63:0] latest_geometry,output reg [63:0] display_geometry,
 input [34:0] latest_lens,input [383:0] latest_lens_params,
 output reg [34:0] display_lens,output reg [383:0] display_lens_params,
 output reg signed [15:0] applied_angle,
 output reg [ADDR_W-1:0] araddr,output reg [3:0] arlen,arid,output reg arvalid,input arready,
 input [255:0] rdata,input [3:0] rid,input rvalid,rlast,
 output reg out_de,output reg [15:0] out_pixel,output reg underflow,output display_ready,
 output reg out_data_valid,
 output reg last_frame_underflow,output read_urgent
);
 localparam PTR=FIFO_BITS+1,DEPTH=1<<FIFO_BITS,TOTAL_WORDS=W*H/16;
 localparam PREFILL=(TOTAL_WORDS<PREFILL_WORDS)?TOTAL_WORDS:PREFILL_WORDS;
 reg [255:0] fifo_mem[0:DEPTH-1];
 reg [PTR-1:0] read_ptr,read_gray,write_ptr,write_gray,reserve_ptr;
 reg [PTR-1:0] read_meta,read_sync,write_meta,write_sync;
 function [PTR-1:0] gray_to_binary;input [PTR-1:0] g;integer i;
 begin for(i=0;i<PTR;i=i+1) gray_to_binary[i]=^(g>>i);end endfunction
 wire [PTR-1:0] occupied=reserve_ptr-gray_to_binary(read_sync);
 wire [PTR-1:0] write_next=write_ptr+1'b1,read_next=read_ptr+1'b1;
 wire fifo_empty=read_gray==write_sync;
 reg [7:0] pixel_epoch,request_epoch,epoch_meta,epoch_sync,desired_epoch;
 reg request_toggle,request_meta,request_sync,request_seen,flushing;
 reg request_pending,ack_meta,ack_sync,request_settle1,request_settle2;
 reg ready_toggle,ready_meta,ready_sync,ready_seen,ready_pixel;
 reg [7:0] ready_epoch,ready_pixel_epoch;
 reg [7:0] owned,complete;
 reg [PTR-1:0] tag_base[0:7],tag_end[0:7];
 reg [3:0] tag_beat[0:7];reg [2:0] allocate_id,commit_id;
 reg [17:0] issued_words;
 reg [PTR-1:0] prefill_count;
 reg ready_sent,selected_valid;
 localparam URGENT_WORDS=(PREFILL<1024)?PREFILL:1024;
 wire [PTR-1:0] buffered_words=write_ptr-gray_to_binary(read_sync);
 // Protect the scan from new write bursts while its committed data is low.
 // Already accepted or backpressured commands must still finish normally.
 assign read_urgent=selected_valid && !flushing && buffered_words<URGENT_WORDS &&
                    (issued_words<TOTAL_WORDS || owned!=0 || arvalid);
 assign display_ready=ready_sent;
 reg vs_d,de_d,frame_ok,de_pipe,valid_pipe;
 reg [10:0] px,py;reg [3:0] lane_d;
 reg [255:0] fifo_q;
 wire [10:0] cur_x=de_d?px:11'd0;
 wire first_pixel=de && !de_d && py==0;
 wire frame_ready=ready_pixel && ready_pixel_epoch==pixel_epoch;
 wire current_ok=first_pixel?frame_ready:frame_ok;
 wire take_pixel=de && current_ok && !fifo_empty;
 integer k;
 always @(posedge pix_clk) fifo_q<=fifo_mem[read_ptr[FIFO_BITS-1:0]];
 always @(posedge pix_clk or negedge rst_n) begin
  if(!rst_n) begin
   read_ptr<=0;read_gray<=0;write_meta<=0;write_sync<=0;
   request_toggle<=0;request_epoch<=0;pixel_epoch<=0;
   request_pending<=0;ack_meta<=0;ack_sync<=0;
   ready_meta<=0;ready_sync<=0;ready_seen<=0;ready_pixel<=0;ready_pixel_epoch<=0;
   vs_d<=0;de_d<=0;px<=0;py<=0;frame_ok<=0;de_pipe<=0;valid_pipe<=0;lane_d<=0;
   out_de<=0;out_data_valid<=0;out_pixel<=BACKGROUND;underflow<=0;last_frame_underflow<=0;
  end else begin
   write_meta<=write_gray;write_sync<=write_meta;
   ack_meta<=request_seen;ack_sync<=ack_meta;
   ready_meta<=ready_toggle;ready_sync<=ready_meta;
   if(ready_sync!=ready_seen) begin ready_seen<=ready_sync;ready_pixel<=1;ready_pixel_epoch<=ready_epoch;end
   vs_d<=vs;de_d<=de;de_pipe<=de;out_de<=de_pipe;out_data_valid<=valid_pipe;
   lane_d<=cur_x[3:0];valid_pipe<=take_pixel;
   out_pixel<=valid_pipe?fifo_q[lane_d*16+:16]:BACKGROUND;
   if(vs && !vs_d) begin
    // The HUD snapshots during blanking. Preserve the completed frame's result
    // before clearing the live accumulator, otherwise the displayed flag is 0.
    last_frame_underflow<=underflow;
    pixel_epoch<=pixel_epoch+1'b1;request_pending<=1;
    read_ptr<=0;read_gray<=0;px<=0;py<=0;frame_ok<=0;ready_pixel<=0;underflow<=0;
   end else begin
    // Hold the bundled epoch until the destination acknowledges the request.
    if(request_pending && request_toggle==ack_sync) begin
     request_epoch<=pixel_epoch;request_toggle<=!request_toggle;request_pending<=0;
    end
    if(de) px<=cur_x+1'b1;else px<=0;
    if(de_d && !de) py<=py+1'b1;
    if(first_pixel) begin frame_ok<=frame_ready;if(!frame_ready && frame_valid) underflow<=1;end
    if(de && current_ok && fifo_empty) begin frame_ok<=0;underflow<=1;end
    if(take_pixel && cur_x[3:0]==15) begin read_ptr<=read_next;read_gray<=read_next^(read_next>>1);end
   end
  end
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin
   request_meta<=0;request_sync<=0;request_settle1<=0;request_settle2<=0;
   epoch_meta<=0;epoch_sync<=0;read_meta<=0;read_sync<=0;
  end
  else begin
   request_meta<=request_toggle;request_sync<=request_meta;epoch_meta<=request_epoch;epoch_sync<=epoch_meta;
   request_settle1<=request_sync;request_settle2<=request_settle1;
   read_meta<=read_gray;read_sync<=read_meta;
  end
 wire [PTR-1:0] return_address=tag_base[rid[2:0]]+tag_beat[rid[2:0]];
 always @(posedge clk) if(rvalid) fifo_mem[return_address[FIFO_BITS-1:0]]<=rdata;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   write_ptr<=0;write_gray<=0;reserve_ptr<=0;owned<=0;complete<=0;allocate_id<=0;commit_id<=0;
   issued_words<=0;prefill_count<=0;ready_sent<=0;ready_toggle<=0;ready_epoch<=0;
   request_seen<=0;desired_epoch<=0;flushing<=0;selected_valid<=0;
   display_geometry<=0;display_origin<=0;freeze_active<=0;display_bank<=4;applied_angle<=0;display_lens<=0;display_lens_params<=0;araddr<=0;arlen<=0;arid<=8;arvalid<=0;
   for(k=0;k<8;k=k+1) begin tag_base[k]<=0;tag_end[k]<=0;tag_beat[k]<=0;end
  end else begin
   if(request_settle2!=request_seen) begin request_seen<=request_settle2;desired_epoch<=epoch_sync;flushing<=1;end
   if(arvalid && arready) begin
    arvalid<=0;owned[allocate_id]<=1;complete[allocate_id]<=0;
    tag_base[allocate_id]<=reserve_ptr;tag_end[allocate_id]<=reserve_ptr+arlen+1'b1;tag_beat[allocate_id]<=0;
    reserve_ptr<=reserve_ptr+arlen+1'b1;issued_words<=issued_words+arlen+1'b1;allocate_id<=allocate_id+1'b1;
   end else if(!arvalid && !flushing && selected_valid && issued_words<TOTAL_WORDS &&
               !owned[allocate_id] && occupied<=DEPTH-16) begin
    araddr<={{(ADDR_W-3-FRAME_BIT){1'b0}},display_bank,{FRAME_BIT{1'b0}}}+issued_words*8;
    arlen<=(TOTAL_WORDS-issued_words>=16)?15:TOTAL_WORDS-issued_words-1;
    arid<={1'b1,allocate_id};arvalid<=1;
   end
   if(rvalid) begin
    tag_beat[rid[2:0]]<=tag_beat[rid[2:0]]+1'b1;
    if(rlast) complete[rid[2:0]]<=1;
   end
   if(complete[commit_id]) begin
    write_ptr<=write_next;write_gray<=write_next^(write_next>>1);
    if(prefill_count<PREFILL) prefill_count<=prefill_count+1'b1;
    if(!flushing && !ready_sent && prefill_count>=PREFILL-1) begin
     ready_epoch<=desired_epoch;ready_toggle<=!ready_toggle;ready_sent<=1;
    end
    if(write_next==tag_end[commit_id]) begin
     owned[commit_id]<=0;complete[commit_id]<=0;commit_id<=commit_id+1'b1;
    end
   end
   // Drain every old response before changing banks or resetting FIFO pointers.
   if(flushing && owned==0 && !arvalid && request_settle2==request_seen) begin
    write_ptr<=0;write_gray<=0;reserve_ptr<=0;allocate_id<=0;commit_id<=0;
    issued_words<=0;prefill_count<=0;ready_sent<=0;complete<=0;
    // Keep the owned display bank while photography reads successive rows.
    // The renderer already excludes display_bank when allocating a write bank.
    if(freeze_display && selected_valid) freeze_active<=1;
    else begin
     display_geometry<=latest_geometry;display_origin<=latest_origin;display_bank<=latest_bank;applied_angle<=latest_angle;display_lens<=latest_lens;display_lens_params<=latest_lens_params;selected_valid<=frame_valid;freeze_active<=0;
    end
    flushing<=0;
   end
  end
 end
endmodule
