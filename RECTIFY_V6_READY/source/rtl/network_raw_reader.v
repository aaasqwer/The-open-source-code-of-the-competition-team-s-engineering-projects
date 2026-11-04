`timescale 1ns/1ps
// Pin one COMPLETE sensor RGB565 frame before rotation/Brown/Gamma/OSD.
// Two beat registers overlap DDR latency with expansion into ONE line RAM.
module network_raw_reader #(parameter W=1920,H=1080,A=28,FRAME_BIT=22,BANK_BITS=2,SUPPORT_GAMMA=0,GAMMA_POLICY=3)(
 input clk,rst_n,net_clk,net_rst_n,hold_frame,request_toggle,
 input [10:0] request_row,read_x,output reg [15:0] read_pixel,
 output ready,done_toggle,gamma_bypass_frame,output [31:0] frame_id,
 input source_valid,input [BANK_BITS-1:0] latest_bank,input [31:0] latest_id,
 input select_alternate,alternate_valid,input [BANK_BITS-1:0] alternate_bank,input [31:0] alternate_id,
 input gamma_bypass,input [4:0] gamma_step,
 output reg lease_active,lease_processed,output reg [BANK_BITS-1:0] leased_bank,
 output reg [A-1:0] araddr,output [3:0] arlen,arid,
 output reg arvalid,input arready,input [255:0] rdata,input rvalid,rlast);
 wire mailbox_reset_n=rst_n&&net_rst_n;
 reg [1:0] client_live;
 always @(posedge clk or negedge mailbox_reset_n)
  if(!mailbox_reset_n)client_live<=0;else client_live<={client_live[0],1'b1};
 wire [13:0] command;
 cdc_mailbox #(.WIDTH(14)) requests(.src_clk(net_clk),.src_rst_n(mailbox_reset_n),
  .dst_clk(clk),.dst_rst_n(mailbox_reset_n),.src_data({SUPPORT_GAMMA!=0&&select_alternate,hold_frame,request_toggle,request_row}),.dst_data(command));
 reg done,seen,flushing,leased_bypass;reg [4:0] leased_step;reg [31:0] leased_id;wire [34:0] response;
 wire advertised_ready=lease_active&&!flushing&&client_live[1];
 cdc_mailbox #(.WIDTH(35)) replies(.src_clk(clk),.src_rst_n(mailbox_reset_n),
  .dst_clk(net_clk),.dst_rst_n(mailbox_reset_n),.src_data({advertised_ready,done,leased_bypass,leased_id}),.dst_data(response));
 assign {ready,done_toggle,gamma_bypass_frame,frame_id}=response;
 reg [15:0] memory[0:2047] /* synthesis syn_ramstyle="block_ram" */;
 always @(posedge net_clk)read_pixel<=memory[read_x];
 reg busy,read_pending,beat_valid,prefetch_valid;
 reg [10:0] x;reg [7:0] issued;reg [3:0] lane;
 reg [255:0] beat,prefetch;
 wire pop_prefetch=prefetch_valid&&(!beat_valid||lane==15);
 wire read_accept=arvalid&&arready;
 wire read_complete=(read_pending||read_accept)&&rvalid&&rlast;
 wire cancelling=flushing||!client_live[1]||!command[12];
 wire [1:0] state=!busy?0:arvalid?1:read_pending?2:3;
 assign arlen=0;assign arid=7;
 wire pixel_write;wire [11:0] pixel_x;wire [15:0] pixel_value;
 generate if(SUPPORT_GAMMA!=0)begin:gamma_path
  // Separate the 16-way beat mux from the Value-max / Gamma ROM address.
  // The first PC3 timing trial found this combined path missed 125MHz setup.
  reg [15:0] input_pixel;reg [11:0] input_x;reg input_de;
  always @(posedge clk)begin
   input_pixel<=beat[lane*16+:16];input_x<={1'b0,x};
   if(!rst_n||!busy||cancelling)input_de<=0;else input_de<=beat_valid;
  end
  gamma_rgb_preserve #(.ENHANCED_CURVES(GAMMA_POLICY)) curve(.clk(clk),.rstn(rst_n&&busy&&!cancelling),
   .step(leased_step),.rgb_in(input_pixel),.vs_in(1'b0),.hs_in(1'b0),.de_in(input_de),
   .x_in(input_x),.y_in(12'd0),.rgb_out(pixel_value),.rgb888_out(),.vs_out(),.hs_out(),.de_out(pixel_write),.x_out(pixel_x),.y_out());
 end else begin:no_gamma
  assign pixel_write=beat_valid;assign pixel_x={1'b0,x};assign pixel_value=beat[lane*16+:16];
 end endgenerate
 always @(posedge clk)
  if(rst_n&&busy&&!cancelling&&pixel_write)memory[pixel_x[10:0]]<=pixel_value;
 // PHY reset cancels the consumer, NOT a DDR command already offered or
 // accepted. Keep this engine/lease alive until its transaction drains.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;read_pending<=0;beat_valid<=0;prefetch_valid<=0;flushing<=1;
   lease_active<=0;lease_processed<=0;leased_bank<=0;leased_id<=0;leased_bypass<=1;leased_step<=0;issued<=0;
   araddr<=0;arvalid<=0;done<=0;seen<=0;x<=0;lane<=0;beat<=0;prefetch<=0;end
  else begin
   if(SUPPORT_GAMMA!=0&&pixel_write&&pixel_x==W-1&&busy&&!cancelling)begin done<=~done;busy<=0;end
   if(!client_live[1])flushing<=1;
   if(read_accept)begin arvalid<=0;read_pending<=1;issued<=issued+1'b1;end
   if(read_complete)begin
    read_pending<=0;
    if(!cancelling)begin prefetch<=rdata;prefetch_valid<=1;end
   end
   if(flushing||!client_live[1])begin
    beat_valid<=0;prefetch_valid<=0;
    if(!arvalid&&!read_pending)begin
     busy<=0;lease_active<=0;done<=0;seen<=command[11];
     if(client_live[1])flushing<=0;
    end
   end else begin
    if(!lease_active&&command[12]&&(command[13]?alternate_valid:source_valid))begin
     leased_bank<=command[13]?alternate_bank:latest_bank;leased_id<=command[13]?alternate_id:latest_id;
     lease_active<=1;lease_processed<=command[13];seen<=command[11];
     leased_bypass<=!command[13]||gamma_bypass;
     leased_step<=(command[13]&&!gamma_bypass)?gamma_step:0;
    end
    if(lease_active&&!command[12]&&!busy)lease_active<=0;
    if(!busy)begin
     if(lease_active&&command[12]&&command[11]!=seen)begin
      seen<=command[11];
      if(command[10:0]<H)begin
       x<=0;lane<=0;issued<=0;beat_valid<=0;prefetch_valid<=0;
       araddr<=({{(A-BANK_BITS-FRAME_BIT){1'b0}},leased_bank,{FRAME_BIT{1'b0}}})+command[10:0]*(W/2);
       arvalid<=1;busy<=1;
      end
     end
    end else if(!command[12])begin
     beat_valid<=0;prefetch_valid<=0;
     if(!arvalid&&!read_pending)busy<=0;
    end else begin
     if(pop_prefetch)begin beat<=prefetch;beat_valid<=1;prefetch_valid<=0;lane<=0;end
     if(beat_valid)begin
      x<=x+1'b1;
      if(lane==15)begin if(!prefetch_valid)beat_valid<=0;lane<=0;end
      else lane<=lane+1'b1;
      if(x==W-1)begin if(SUPPORT_GAMMA==0)begin done<=~done;busy<=0;end beat_valid<=0;prefetch_valid<=0;end
     end
     if(!arvalid&&!read_pending&&issued<W/16&&(!prefetch_valid||pop_prefetch))begin
      araddr<=araddr+8;arvalid<=1;
     end
    end
   end
  end
 end
endmodule
