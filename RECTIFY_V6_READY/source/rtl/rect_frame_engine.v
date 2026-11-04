`timescale 1ns/1ps
// Independent full-frame renderer. New mapping/sampling path; existing board
// DDR fabric is only the transport. 32x16 ping-pong coordinate tiles, 64x64
// lane cache, single bilinear resampling, masked tile writes. No full strip RAM.
// A completed output bank is published only after all writes and a drain guard.
module rect_frame_engine #(parameter W=1920,H=1080,A=28,FRAME_BIT=22,COMMIT_GUARD=4096,GAMMA_ENABLE=0,GAMMA_POLICY=3)(
 input clk,rst_n,source_valid,input [1:0] source_frame,input [31:0] source_serial,
 input [31:0] source_start_tick,output reg [63:0] latest_origin,
 input signed [15:0] angle,input [2:0] display_bank,
 input network_frame_hold,input [2:0] network_frame,
 input [416:0] lens_request,input freeze_config,input [5:0] gamma_request,
 input [1:0] request_mode,input request_imu,request_plane,plane_valid,
 input [31:0] control_sequence,input [287:0] normal_matrix,plane_matrix,
 output reg [34:0] latest_lens,output reg [383:0] latest_lens_params,
 output reg [1:0] held_source,output reg [2:0] latest_bank,
 output reg signed [15:0] latest_angle,output reg frame_valid,output busy,
 output reg [31:0] render_serial,output [31:0] debug_position,
 output reg [31:0] latest_geometry,latest_control_sequence,
 output reg [A-1:0] araddr,output reg [3:0] arid,output [3:0] arlen,
 output reg arvalid,input arready,input [255:0] rdata,input [3:0] rid,input rvalid,rlast,
 output reg [A-1:0] awaddr,output [3:0] awid,awlen,output reg awvalid,input awready,
 output reg [255:0] wdata,output reg [31:0] wstrb,input wready
);
 // Local registered bundle keeps atomic lens configuration and removes the
 // long CDC-header route from the high-fanout frame-load enable.
 reg [416:0] lens_snapshot;
 always @(posedge clk or negedge rst_n)if(!rst_n)lens_snapshot<=0;else lens_snapshot<=lens_request;
 localparam IDLE=0,SETUP=1,FIRST=2,WAIT_MAP=3,BOX=4,FETCH=5,SAMPLE_START=6,SAMPLE=7,DRAIN=8,
  WRITE_PREP=9,WRITE_ADDR=10,WRITE_DATA=11,NEXT=12,GUARD=13,PUBLISH=14,ABORT=15,RETRY=16;
 reg [4:0] state;
 reg [31:0] frame_start_tick;reg [5:0] frame_tone;
 reg [416:0] frame_config;reg [31:0] frame_source_serial,frame_control_sequence;
 reg [1:0] mode;reg [287:0] transform;reg signed [15:0] frame_angle;reg frame_plane,frame_imu;
 wire signed [17:0] cosine,sine;reg signed [17:0] cc,ss;
 imu_rotation_coeff rotation(.clk(clk),.angle(frame_angle),.cos_q(cosine),.sin_q(sine));
 reg [2:0] output_bank;reg [15:0] guard_count;
 reg [10:0] tile_x,tile_y,next_x,next_y;
 reg [5:0] tile_w,next_w;reg [4:0] tile_h,next_h,strip_h;
 reg bank,next_exists,next_requested;
 reg prepare_start,prepare_clear,prepare_bank;
 reg [10:0] prepare_x,prepare_y;reg [5:0] prepare_w;reg [4:0] prepare_h;
 wire [1:0] map_ready,map_any,map_bad;wire [87:0] map_bounds;wire map_busy;
 wire [43:0] box_bounds=bank?map_bounds[87:44]:map_bounds[43:0];
 wire [10:0] bx0=box_bounds[10:0],bx1=box_bounds[21:11],by0=box_bounds[32:22],by1=box_bounds[43:33];
 reg [8:0] read_address;wire [52:0] coordinate;
 wire field_ready,field_fault,field_busy;reg field_start,field_enhanced;
 reg [31:0] field_config_id;reg [319:0] field_config;
 wire field_in_valid,field_out_valid,field_out_bad,field_inside;
 wire signed [25:0] field_x,field_y,field_sx,field_sy;
 rect_lens_field #(.W(W),.H(H)) lens(.clk(clk),.rst_n(rst_n),.boot_start(field_start),.enhanced(field_enhanced),.calibration(field_config),
  .ready(field_ready),.fault(field_fault),.busy(field_busy),.in_valid(field_in_valid),.ideal_x(field_x),.ideal_y(field_y),
  .out_valid(field_out_valid),.out_bad(field_out_bad),.source_x(field_sx),.source_y(field_sy),.source_inside(field_inside));
 rect_coordinate_tiles #(.W(W),.H(H)) maps(.clk(clk),.rst_n(rst_n),.clear(prepare_clear),.start(prepare_start),.write_bank(prepare_bank),
  .tile_x(prepare_x),.tile_y(prepare_y),.tile_w(prepare_w),.tile_h(prepare_h),.mode(mode),.inverse_matrix(transform),.c(cc),.s(ss),
  .field_ready(field_ready),.field_fault(field_fault),.field_valid(field_in_valid),.field_x(field_x),.field_y(field_y),
  .field_out_valid(field_out_valid),.field_out_bad(field_out_bad),.field_inside(field_inside),.field_source_x(field_sx),.field_source_y(field_sy),
  .read_bank(bank),.read_address(read_address),.read_coordinate(coordinate),.ready(map_ready),.any_inside(map_any),.bad(map_bad),.bounds(map_bounds),.busy(map_busy));
 wire requested_valid=lens_snapshot[416]&&lens_snapshot[352];
 wire [1:0] wanted_mode=(request_plane&&plane_valid&&request_mode==0)?2'd1:request_mode;
 wire wanted_enhanced=wanted_mode!=2&&lens_snapshot[415:384]==1;
 assign busy=state!=IDLE;assign arlen=0;assign awlen=0;assign awid=0;
 assign debug_position={6'd0,field_fault,state,tile_y,tile_x[8:0]};
 reg [10:0] bbox_x,bbox_y,bbox_last_y,fetch_y;
 reg [2:0] row_beats,fetch_word;reg [6:0] pending;reg requests_done;
 reg [7:0] response_tag[0:6];
 wire accept_read=arvalid&&arready;
 wire [7:0] accepted_cache_address={(fetch_y-bbox_y),2'd0}+fetch_word;
 wire [7:0] response_address=(accept_read&&rid==arid&&!pending[rid[2:0]])?accepted_cache_address:response_tag[rid[2:0]];
 wire [2:0] id_after=arid==6?0:arid+1'b1;
 wire load_valid=rvalid&&state==FETCH;
 reg [5:0] ox;reg [4:0] oy;reg sample_valid,sample_good;reg [8:0] sample_tag;
 reg [9:0] pixels_returned;wire pixel_valid,pixel_good;wire [15:0] pixel;wire [8:0] pixel_tag;
 wire sample_pixel_valid,sample_pixel_good;wire [15:0] sample_pixel;wire [8:0] sample_pixel_tag;
 generate if(GAMMA_ENABLE!=0)begin:frame_gamma
  wire [11:0] tag_out,good_out;
  // Reuse BKB's exact curves during the resampling/rendering pass. A single
  // frame-owned curve is written into DDR; HDMI and PC never apply it twice.
  gamma_rgb_preserve #(.ENHANCED_CURVES(GAMMA_POLICY),.INPUT_PIPELINE(1)) gamma(
   .clk(clk),.rstn(rst_n),.step(frame_tone[5]?5'd0:frame_tone[4:0]),
   .rgb_in(sample_pixel_good?sample_pixel:16'd0),.de_in(sample_pixel_valid),.vs_in(1'b0),.hs_in(1'b0),
   .x_in({3'd0,sample_pixel_tag}),.y_in({11'd0,sample_pixel_good}),
   .rgb_out(pixel),.rgb888_out(),.de_out(pixel_valid),.vs_out(),.hs_out(),.x_out(tag_out),.y_out(good_out));
  assign pixel_tag=tag_out[8:0];assign pixel_good=good_out[0];
 end else begin:original_test_path
  assign pixel=sample_pixel;assign pixel_tag=sample_pixel_tag;
  assign pixel_valid=sample_pixel_valid;assign pixel_good=sample_pixel_good;
 end endgenerate
 rect_tile_sampler #(.W(W),.H(H)) sampler(.clk(clk),.rst_n(rst_n),.load_valid(load_valid),.load_address(response_address),.load_data(rdata),
  .sample_valid(sample_valid),.sample_good(coordinate[52]),.source_x(coordinate[51:26]),.source_y(coordinate[25:0]),
  .bbox_x(bbox_x),.bbox_y(bbox_y),.sample_tag(sample_tag),.pixel_valid(sample_pixel_valid),.pixel_good(sample_pixel_good),.pixel(sample_pixel),.pixel_tag(sample_pixel_tag));
 reg [255:0] pack;reg [255:0] tile_words[0:31] /* synthesis syn_ramstyle="distributed" */;
 wire [3:0] pixel_lane=tile_x[3:0]+pixel_tag[3:0];
 wire [4:0] pixel_word={pixel_tag[8:5],pixel_tag[4]};
 wire [15:0] rendered_pixel=pixel_good?pixel:16'd0;
 reg [255:0] updated_pack;
 always @*begin updated_pack=pack;updated_pack[pixel_lane*16+:16]=rendered_pixel;end
 reg [4:0] dma_row;reg dma_half;
 wire [4:0] dma_index={dma_row[3:0],dma_half};
 wire [5:0] valid_pixels_this_beat=tile_w>=16?16:tile_w;
 wire [31:0] lane_mask=(32'hffffffff>>(32-valid_pixels_this_beat*2))<<(tile_x[3:0]*2);
 integer k;
 always @(posedge clk)begin
  if(pixel_valid&&(state==SAMPLE||state==DRAIN))begin
   pack<=updated_pack;
   if(pixel_lane==15||pixel_tag[4:0]==tile_w-1)tile_words[pixel_word]<=updated_pack;
  end
  if(state==WRITE_PREP)wdata<=tile_words[dma_index];
 end
 function [5:0] aligned_width;
  input [10:0] x;
  begin if(x[4:0]==0)aligned_width=32;else if(x[3:0]==0)aligned_width=16;else if(x[2:0]==0)aligned_width=8;
   else if(x[1:0]==0)aligned_width=4;else if(x[0]==0)aligned_width=2;else aligned_width=1;end
 endfunction
 wire [11:0] candidate_next_x={1'b0,tile_x}+tile_w;
 wire has_next=candidate_next_x<W||tile_y+strip_h<H;
 wire [10:0] calculated_next_x=candidate_next_x>=W?0:candidate_next_x[10:0];
 wire [10:0] calculated_next_y=candidate_next_x>=W?tile_y+strip_h:tile_y;
 wire [5:0] calculated_next_w=(W-calculated_next_x<aligned_width(calculated_next_x))?W-calculated_next_x:aligned_width(calculated_next_x);
 wire [4:0] calculated_next_h=(H-calculated_next_y<strip_h)?H-calculated_next_y:strip_h;
 wire [10:0] halo_x=bx1==W-1?bx1:bx1+1'b1,halo_y=by1==H-1?by1:by1+1'b1;
 wire [10:0] aligned_x={bx0[10:4],4'd0};
 wire [11:0] needed_width={1'b0,halo_x}-aligned_x+1'b1;
 wire [11:0] needed_height={1'b0,halo_y}-by0+1'b1;
 wire [A-1:0] source_base={{(A-2-FRAME_BIT){1'b0}},held_source,{FRAME_BIT{1'b0}}};
 wire [A-1:0] output_base={{(A-3-FRAME_BIT){1'b0}},output_bank,{FRAME_BIT{1'b0}}};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;held_source<=0;latest_origin<=0;frame_start_tick<=0;frame_tone<=6'd32;latest_bank<=4;latest_angle<=0;frame_valid<=0;render_serial<=0;latest_lens<=0;latest_lens_params<=0;
   frame_config<=0;frame_source_serial<=32'hffffffff;frame_control_sequence<=32'hffffffff;mode<=0;transform<=0;frame_angle<=0;cc<=16384;ss<=0;
   latest_geometry<=0;latest_control_sequence<=0;frame_plane<=0;frame_imu<=0;
   output_bank<=5;guard_count<=0;tile_x<=0;tile_y<=0;tile_w<=32;tile_h<=16;strip_h<=16;bank<=0;
   next_x<=0;next_y<=0;next_w<=0;next_h<=0;next_exists<=0;next_requested<=0;
   prepare_start<=0;prepare_clear<=0;prepare_bank<=0;prepare_x<=0;prepare_y<=0;prepare_w<=0;prepare_h<=0;read_address<=0;
   field_start<=0;field_enhanced<=1;field_config_id<=0;field_config<=0;
   bbox_x<=0;bbox_y<=0;bbox_last_y<=0;fetch_y<=0;row_beats<=0;fetch_word<=0;pending<=0;requests_done<=0;
   araddr<=0;arid<=0;arvalid<=0;awaddr<=0;awvalid<=0;wstrb<=32'hffffffff;
   ox<=0;oy<=0;sample_valid<=0;sample_good<=0;sample_tag<=0;pixels_returned<=0;dma_row<=0;dma_half<=0;
   for(k=0;k<7;k=k+1)response_tag[k]<=0;
  end else begin
   prepare_start<=0;prepare_clear<=0;field_start<=0;sample_valid<=0;
   if(accept_read)begin
    response_tag[arid[2:0]]<=accepted_cache_address;pending[arid[2:0]]<=1;arvalid<=0;arid<=id_after;
    if(fetch_word==row_beats-1)begin fetch_word<=0;if(fetch_y==bbox_last_y)requests_done<=1;else fetch_y<=fetch_y+1'b1;end
    else fetch_word<=fetch_word+1'b1;
   end
   if(rvalid&&state==FETCH)pending[rid[2:0]]<=0;
   if(pixel_valid&&(state==SAMPLE||state==DRAIN))pixels_returned<=pixels_returned+1'b1;
   case(state)
    IDLE:begin
     if(!field_busy&&!field_start&&requested_valid&&
        (field_config_id!=lens_snapshot[415:384]||(wanted_mode!=0&&field_enhanced!=wanted_enhanced)))begin
      field_config<=lens_snapshot[319:0];field_config_id<=lens_snapshot[415:384];field_enhanced<=wanted_enhanced;field_start<=1;
     end else if(source_valid&&!freeze_config&&(!frame_valid||source_serial!=frame_source_serial||control_sequence!=frame_control_sequence||
       frame_config[415:384]!=lens_snapshot[415:384]))begin
      if(wanted_mode==0||!requested_valid||(field_ready&&!field_busy&&!field_start&&!field_fault))begin
       frame_config<=lens_snapshot;mode<=requested_valid?wanted_mode:0;held_source<=source_frame;frame_source_serial<=source_serial;frame_start_tick<=source_start_tick;frame_control_sequence<=control_sequence;
       transform<=(request_plane&&plane_valid)?plane_matrix:normal_matrix;
       frame_plane<=request_plane&&plane_valid;frame_imu<=request_imu;
       frame_angle<=request_imu?angle:0;frame_tone<=gamma_request;
       if(display_bank!=4&&latest_bank!=4&&!(network_frame_hold&&network_frame==4))output_bank<=4;
       else if(display_bank!=5&&latest_bank!=5&&!(network_frame_hold&&network_frame==5))output_bank<=5;
       else if(display_bank!=6&&latest_bank!=6&&!(network_frame_hold&&network_frame==6))output_bank<=6;
       else output_bank<=7;
       tile_x<=0;tile_y<=0;tile_w<=W<32?W:32;tile_h<=H<16?H:16;strip_h<=H<16?H:16;bank<=0;
       prepare_clear<=1;guard_count<=0;state<=SETUP;
      end
     end
    end
    SETUP:if(guard_count==3)begin cc<=cosine;ss<=sine;state<=FIRST;end else guard_count<=guard_count+1'b1;
    FIRST:begin prepare_bank<=bank;prepare_x<=tile_x;prepare_y<=tile_y;prepare_w<=tile_w;prepare_h<=tile_h;prepare_start<=1;next_requested<=0;state<=WAIT_MAP;end
    WAIT_MAP:if(map_ready[bank]&&!map_busy&&!prepare_start)state<=BOX;
    BOX:begin
     if(map_bad[bank])state<=ABORT;
     else if(map_any[bank]&&(needed_width>64||needed_height>64))begin
      if(tile_w>1)tile_w<=tile_w>>1;
      else if(tile_h>1)begin tile_h<=tile_h>>1;strip_h<=tile_h>>1;end
      else state<=ABORT;
      if(tile_w>1||tile_h>1)state<=RETRY;
     end else begin
      bbox_x<=aligned_x;bbox_y<=by0;bbox_last_y<=halo_y;fetch_y<=by0;fetch_word<=0;row_beats<=(needed_width+15)>>4;pending<=0;requests_done<=!map_any[bank];
      next_x<=calculated_next_x;next_y<=calculated_next_y;next_w<=calculated_next_w;next_h<=calculated_next_h;next_exists<=has_next;
      if(has_next)begin prepare_bank<=~bank;prepare_x<=calculated_next_x;prepare_y<=calculated_next_y;prepare_w<=calculated_next_w;prepare_h<=calculated_next_h;prepare_start<=1;next_requested<=1;end
      state<=FETCH;
     end
    end
    RETRY:state<=FIRST;
    FETCH:begin
     if(!requests_done&&!arvalid&&!pending[arid[2:0]])begin
      araddr<=source_base+fetch_y*(W/2)+(bbox_x>>1)+fetch_word*8;arvalid<=1;
     end
     if(requests_done&&pending==0&&!arvalid)state<=SAMPLE_START;
    end
    SAMPLE_START:begin ox<=0;oy<=0;read_address<=0;pixels_returned<=0;state<=SAMPLE;end
    SAMPLE:begin
     sample_valid<=1;sample_good<=coordinate[52];sample_tag<={oy[3:0],ox[4:0]};
     if(ox==tile_w-1)begin
      ox<=0;if(oy==tile_h-1)state<=DRAIN;else begin oy<=oy+1'b1;read_address<={(oy[3:0]+1'b1),5'd0};end
     end else begin ox<=ox+1'b1;read_address<=read_address+1'b1;end
    end
    DRAIN:if(pixels_returned==tile_w*tile_h)begin dma_row<=0;dma_half<=0;state<=WRITE_PREP;end
    WRITE_PREP:begin
     awaddr<=output_base+(tile_y+dma_row)*(W/2)+((tile_x>>4)+dma_half)*8;
     wstrb<=tile_w>=16?32'hffffffff:lane_mask;state<=WRITE_ADDR;
    end
    WRITE_ADDR:begin awvalid<=1;if(awvalid&&awready)begin awvalid<=0;state<=WRITE_DATA;end end
    WRITE_DATA:if(wready)begin
     if(tile_w==32&&!dma_half)begin dma_half<=1;state<=WRITE_PREP;end
     else if(dma_row==tile_h-1)state<=NEXT;
     else begin dma_half<=0;dma_row<=dma_row+1'b1;state<=WRITE_PREP;end
    end
    NEXT:if(next_exists)begin tile_x<=next_x;tile_y<=next_y;tile_w<=next_w;tile_h<=next_h;bank<=~bank;next_requested<=0;state<=WAIT_MAP;end
     else begin guard_count<=0;state<=GUARD;end
    GUARD:if(guard_count==COMMIT_GUARD-1)state<=PUBLISH;else guard_count<=guard_count+1'b1;
    PUBLISH:begin
     latest_bank<=output_bank;latest_angle<=frame_angle;latest_origin<={frame_source_serial,frame_start_tick};frame_valid<=1;render_serial<=render_serial+1'b1;
     latest_lens<={frame_config[415:384],frame_config[416],(mode!=0),1'b0};latest_lens_params<=frame_config[383:0];state<=IDLE;
     latest_geometry<={18'd0,frame_tone,field_enhanced,frame_imu,frame_plane,3'd0,mode};latest_control_sequence<=frame_control_sequence;
    end
    ABORT:begin
     latest_lens[0]<=1;prepare_clear<=1;
     // Keep the last completed frame. Retry only after a new source/control.
     state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
