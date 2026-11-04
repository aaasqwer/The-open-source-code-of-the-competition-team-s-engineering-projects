`timescale 1ns/1ps
// Four complete line slots with a Gray-pointer asynchronous descriptor FIFO.
// A slot stays owned by DDR until its final write beat has been accepted.
// Referenced FIFO ownership practice: alexforencich/verilog-axis axis_async_fifo.
// Camera-specific control below is implemented locally; no external IP needed.
// Only complete frames are published.
// Three external banks normally; a fourth bank when NETWORK_LEASE is enabled
// excludes the network frame as well as the renderer-held and latest frames.
module camera_frame_writer #(parameter integer W=1920,H=1080,ADDR_W=28,FRAME_BIT=22,COLUMN_BITS=10,
 parameter integer COMMIT_GUARD=4096,NETWORK_LEASE=0,POSE_SAMPLE_MID=0)(
 input cam_clk,rst_n,vs,pixel_valid,line_end,
 input [15:0] pixel,
 input signed [15:0] frame_angle,
 input ddr_clk,input [31:0] time_gray,output reg [31:0] latest_start_tick,
 input [1:0] display_frame,
 input network_frame_hold,input [1:0] network_frame,
 output reg [31:0] frame_serial,
 output reg [1:0] latest_frame,
 output reg frame_valid,
 output reg signed [15:0] latest_frame_angle,
 output reg overflow,
 output reg [ADDR_W-1:0] awaddr,
 output [3:0] awid,
 output reg [3:0] awlen,
 output reg awvalid,
 input awready,
 output [255:0] wdata,
 output [31:0] wstrb,
 input wready
);
 localparam integer BEATS=W/16;
 reg [1:0] cam_reset_pipe,ddr_reset_pipe;
 always @(posedge cam_clk or negedge rst_n)
  if(!rst_n) cam_reset_pipe<=0;else cam_reset_pipe<={cam_reset_pipe[0],1'b1};
 always @(posedge ddr_clk or negedge rst_n)
  if(!rst_n) ddr_reset_pipe<=0;else ddr_reset_pipe<={ddr_reset_pipe[0],1'b1};
 wire cam_rst_n=cam_reset_pipe[1],ddr_rst_n=ddr_reset_pipe[1];
 reg [255:0] line_mem[0:511];
 reg [255:0] pack;
 reg [10:0] x,y;
 reg [6:0] wb;
 reg vs_d,accept_line,overflow_cam,frame_seen;
 reg [7:0] generation;
 reg [2:0] wr_ptr,wr_gray,rd_ptr,rd_gray;
 reg [2:0] rd_gray_meta,rd_gray_sync,wr_gray_meta,wr_gray_sync;
 // Pose travels through the same owned descriptor slots as the frame lines.
 // End-of-frame is an ordered descriptor too. H good rows alone are not
 // evidence of a complete H-row frame; the next VS must confirm its geometry.
 reg [68:0] descriptor[0:3] /* synthesis syn_ramstyle="registers" */; // EOF[36], angle[35:20], generation[19:12], line/count[11:1], good[0]
 reg frame_bad_cam;
 (* async_reg="true" *) reg [31:0] capture_time_meta,capture_time_sync;
 reg [31:0] capture_start_tick,current_start_tick;
 function [31:0] binary_time;input [31:0] g;integer i;
 begin for(i=0;i<32;i=i+1)binary_time[i]=^(g>>i);end endfunction
 always @(posedge cam_clk or negedge cam_rst_n)begin
  if(!cam_rst_n)begin capture_time_meta<=0;capture_time_sync<=0;capture_start_tick<=0;end
  else begin capture_time_meta<=time_gray;capture_time_sync<=capture_time_meta;
   if(frame_seen&&!vs&&pixel_valid&&x==0&&y==0)capture_start_tick<=binary_time(capture_time_sync);
  end
 end
 reg signed [15:0] capture_angle;
 wire fifo_full=wr_gray==(rd_gray_sync^3'b110);
 wire fifo_empty=rd_gray==wr_gray_sync;
 wire take_line=(x==0)?!fifo_full:accept_line;
 wire [2:0] wr_next=wr_ptr+1'b1,rd_next=rd_ptr+1'b1;
 wire [68:0] head=descriptor[rd_ptr[1:0]];
 wire head_eof=head[36];
 wire [10:0] head_y=head[11:1];
 wire [7:0] head_generation=head[19:12];
 wire head_good=head[0];
 always @(posedge cam_clk or negedge cam_rst_n)
  if(!cam_rst_n) begin rd_gray_meta<=0;rd_gray_sync<=0;end
  else begin rd_gray_meta<=rd_gray;rd_gray_sync<=rd_gray_meta;end
 always @(posedge cam_clk or negedge cam_rst_n) begin
  if(!cam_rst_n) begin
   pack<=0;x<=0;y<=0;wb<=0;vs_d<=0;accept_line<=0;overflow_cam<=0;
   generation<=0;wr_ptr<=0;wr_gray<=0;capture_angle<=0;frame_seen<=0;frame_bad_cam<=0;
  end else begin
   vs_d<=vs;
   if(vs) begin
    x<=0;y<=0;wb<=0;accept_line<=0;capture_angle<=frame_angle;frame_seen<=1;
    if(!vs_d)begin
     // Empty VS epochs during sensor startup contain no captured image.
     // Any nonempty epoch must end explicitly, including malformed epochs.
     if(frame_seen && (y!=0 || x!=0 || frame_bad_cam))begin
      if(!fifo_full)begin
       descriptor[wr_ptr[1:0]]<={capture_start_tick,1'b1,capture_angle,generation,y,(y==H && x==0 && !frame_bad_cam)};
       wr_ptr<=wr_next;wr_gray<=wr_next^(wr_next>>1);
      end else overflow_cam<=1;
     end
     generation<=generation+1'b1;frame_bad_cam<=0;
    end
   end
   // Use the attitude near the temporal centre of the active raster, rather
   // than the much older VS attitude. This is not motion deblur or a change
   // to the BKB projection. The EOF descriptor owns the final sampled pose.
   // DDR may come out of reset while the sensor is already streaming. Until
   // VS, neither row zero nor its exposure pose is known. Ignore orphan rows
   // rather than publishing an unframed image with the reset angle of zero.
   else if(frame_seen) begin
    if(POSE_SAMPLE_MID && pixel_valid && x==0 && y==H/2)capture_angle<=frame_angle;
    if(pixel_valid) begin
     pack[x[3:0]*16 +:16]<=pixel;
     if(x==0) begin accept_line<=!fifo_full;if(fifo_full)begin overflow_cam<=1;frame_bad_cam<=1;end end
     // Saturate one pixel beyond W, so an overlong line is rejected too.
     if(x<W+1) x<=x+1'b1;
     if(x[3:0]==15 && x<W && take_line) begin
      line_mem[{wr_ptr[1:0],wb}]<={pixel,pack[239:0]};wb<=wb+1'b1;
     end
    end
    if(line_end) begin
     if(take_line) begin
      descriptor[wr_ptr[1:0]]<={capture_start_tick,1'b0,capture_angle,generation,y,(x==W)};
      wr_ptr<=wr_next;wr_gray<=wr_next^(wr_next>>1);
     end else begin overflow_cam<=1;frame_bad_cam<=1;end
     if(x!=W || y>=H)begin overflow_cam<=1;frame_bad_cam<=1;end
     x<=0;wb<=0;accept_line<=0;if(y<H+1) y<=y+1'b1;
    end
   end
  end
 end
 reg overflow_meta,overflow_sync;
 always @(posedge ddr_clk or negedge ddr_rst_n) begin
  if(!ddr_rst_n) begin wr_gray_meta<=0;wr_gray_sync<=0;overflow_meta<=0;overflow_sync<=0;end
  else begin wr_gray_meta<=wr_gray;wr_gray_sync<=wr_gray_meta;
   overflow_meta<=overflow_cam;overflow_sync<=overflow_meta;end
 end
 reg [2:0] state;
 reg [1:0] write_frame;
 reg [10:0] current_y,expected_y;
 reg [7:0] current_generation;
 reg signed [15:0] current_frame_angle;
 reg bad_frame;
 reg [6:0] rb;
 reg [3:0] burst_count;
 reg [255:0] q;
 reg [31:0] guard_count;
 wire [ADDR_W-1:0] next_address=
  {{(ADDR_W-2-FRAME_BIT){1'b0}},write_frame,{FRAME_BIT{1'b0}}}+current_y*(W/2)+rb*8;
 wire [COLUMN_BITS:0] column_beats=((1<<COLUMN_BITS)-{1'b0,next_address[COLUMN_BITS-1:0]})>>3;
 wire [7:0] remaining_beats=BEATS-rb;
 wire [7:0] limited_beats=(remaining_beats>16)?8'd16:remaining_beats;
 wire [7:0] burst_beats=(column_beats<limited_beats)?column_beats:limited_beats;
 wire [6:0] next_rb=(state==3 && wready)?rb+1'b1:rb;
 always @(posedge ddr_clk) q<=line_mem[{rd_ptr[1:0],next_rb}];
 assign wdata=q;assign wstrb=32'hffffffff;assign awid=0;
 always @(posedge ddr_clk or negedge ddr_rst_n) begin
  if(!ddr_rst_n) begin
   state<=0;rd_ptr<=0;rd_gray<=0;write_frame<=0;latest_frame<=1;frame_valid<=0;overflow<=0;
   awaddr<=0;awlen<=0;awvalid<=0;rb<=0;burst_count<=0;
   current_y<=0;current_generation<=0;expected_y<=0;bad_frame<=1;guard_count<=0;
   current_frame_angle<=0;latest_frame_angle<=0;frame_serial<=0;current_start_tick<=0;latest_start_tick<=0;
  end else begin
   case(state)
    0: if(!fifo_empty) begin
     if(head_eof)begin
      rd_ptr<=rd_next;rd_gray<=rd_next^(rd_next>>1);
      if(head_good && head_generation==current_generation && expected_y==H && !bad_frame)begin
       if(POSE_SAMPLE_MID)current_frame_angle<=head[35:20];
       guard_count<=0;state<=4;
      end else begin bad_frame<=1;overflow<=1;end
     end else begin
     current_y<=head_y;rb<=0;
     if(head_y==0) begin
      current_generation<=head_generation;expected_y<=0;bad_frame<=!head_good;
      current_frame_angle<=head[35:20];current_start_tick<=head[68:37];
      if(display_frame!=0 && latest_frame!=0 && (!NETWORK_LEASE || !network_frame_hold || network_frame!=0)) write_frame<=0;
      else if(display_frame!=1 && latest_frame!=1 && (!NETWORK_LEASE || !network_frame_hold || network_frame!=1)) write_frame<=1;
      else if(!NETWORK_LEASE || (display_frame!=2 && latest_frame!=2 && (!network_frame_hold || network_frame!=2))) write_frame<=2;
      else write_frame<=3;
     end else if(head_generation!=current_generation || head_y!=expected_y || !head_good) begin bad_frame<=1;overflow<=1;end
     if(head_y<H && head_good) state<=1;
     else begin
      bad_frame<=1;overflow<=1;
      rd_ptr<=rd_next;rd_gray<=rd_next^(rd_next>>1);
     end
     end
    end
    1: begin
     // HMIC wraps within a column instead of carrying into the next bank.
     // Split every burst at that boundary (UG042003 sections 2.8.1/2.8.2).
     awaddr<=next_address;
     awlen<=burst_beats-1'b1;
     awvalid<=1;state<=2;
    end
    2: if(awvalid && awready) begin awvalid<=0;burst_count<=0;state<=3;end
    3: if(wready) begin
     rb<=rb+1'b1;
     if(burst_count==awlen) begin
      if(rb==BEATS-1) begin
       expected_y<=current_y+1'b1;
       state<=0; // The ordered EOF descriptor decides whether this frame is complete.
       rd_ptr<=rd_next;rd_gray<=rd_next^(rd_next>>1);
      end else state<=1;
     end else burst_count<=burst_count+1'b1;
    end
    // WREADY is acceptance, not a physical write-completion response. Match the
    // output renderer's bounded guard before transferring ownership to a reader.
    4:if(COMMIT_GUARD<=1 || guard_count>=COMMIT_GUARD-1) begin
       latest_frame<=write_frame;latest_frame_angle<=current_frame_angle;latest_start_tick<=current_start_tick;frame_valid<=1;frame_serial<=frame_serial+1'b1;state<=0;
      end else guard_count<=guard_count+1'b1;
    default:state<=0;
   endcase
   if(overflow_sync) overflow<=1;
  end
 end
endmodule
