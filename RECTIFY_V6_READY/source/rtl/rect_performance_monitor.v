`timescale 1ns/1ps
// Digital latency: first accepted camera RGB565 pixel to first corrected HDMI
// pixel, BEFORE the single HDMI output register. Exposure/monitor response are
// excluded. A source ID travels with each owned frame; repeats never count as
// new output. Read the preserved probe with PDS ILA; no host protocol changes.
module rect_performance_monitor #(parameter CLK_HZ=125000000,W=1920,H=1080)(
 input clk,rst_n,cam_clk,cam_vs,pix_clk,pix_vs,pix_de,pix_underflow,
 input [31:0] source_id,render_id,source_start_tick,
 input [63:0] render_origin,display_origin,
 input render_busy,capture_overflow,
 output [31:0] time_gray,
 output [511:0] probe
);
 reg [31:0] tick;
 (* syn_keep=1,syn_preserve=1 *) reg [31:0] gray_tick;
 assign time_gray=gray_tick;
 function [31:0] binary;input [31:0] g;integer i;
 begin for(i=0;i<32;i=i+1)binary[i]=^(g>>i);end endfunction
 reg cam_vs_d,cam_toggle;
 always @(posedge cam_clk or negedge rst_n)begin
  if(!rst_n)begin cam_vs_d<=0;cam_toggle<=0;end
  else begin cam_vs_d<=cam_vs;if(cam_vs&&!cam_vs_d)cam_toggle<=!cam_toggle;end
 end
 reg cam_m,cam_s,cam_seen;
 (* async_reg="true" *) reg [31:0] pix_time_meta,pix_time_sync;
 wire [63:0] pix_origin;
 cdc_mailbox #(.WIDTH(64)) origin_cdc(.src_clk(clk),.src_rst_n(rst_n),.dst_clk(pix_clk),.dst_rst_n(rst_n),.src_data(display_origin),.dst_data(pix_origin));
 reg pvs_d,pde_d,pseen,pbad;
 reg [31:0] pixels,first_tick,first_id,first_start,event_sequence;
 reg [15:0] row_width,max_width,rows;
 reg [191:0] pix_record;
 // record = sequence, source ID, delay, pixel count, width/height, flags.
 always @(posedge pix_clk or negedge rst_n)begin
  if(!rst_n)begin
   pix_time_meta<=0;pix_time_sync<=0;pvs_d<=0;pde_d<=0;pseen<=0;pbad<=0;
   pixels<=0;first_tick<=0;first_id<=0;first_start<=0;event_sequence<=0;
   row_width<=0;max_width<=0;rows<=0;pix_record<=0;
  end else begin
   pix_time_meta<=gray_tick;pix_time_sync<=pix_time_meta;pvs_d<=pix_vs;pde_d<=pix_de;
   if(pix_vs&&!pvs_d)begin
    event_sequence<=event_sequence+1'b1;
    pix_record<={event_sequence+1'b1,first_id,first_tick-first_start,pixels,max_width,rows,
      29'd0,(pseen&&pixels==W*H&&max_width==W&&rows==H&&!pbad&&!pix_underflow),(first_id!=0),pbad|pix_underflow};
    pixels<=0;row_width<=0;max_width<=0;rows<=0;pseen<=0;pbad<=0;
   end else begin
    if(pix_de)begin
     pixels<=pixels+1'b1;row_width<=pde_d?row_width+1'b1:16'd1;
     if(!pseen)begin pseen<=1;first_tick<=binary(pix_time_sync);first_id<=pix_origin[63:32];first_start<=pix_origin[31:0];end
    end
    if(pde_d&&!pix_de)begin
     if(row_width>max_width)max_width<=row_width;
     rows<=rows+1'b1;row_width<=0;
    end
    if(pix_underflow)pbad<=1;
   end
  end
 end
 wire [191:0] event_core;
 cdc_mailbox #(.WIDTH(192)) event_cdc(.src_clk(pix_clk),.src_rst_n(rst_n),.dst_clk(clk),.dst_rst_n(rst_n),.src_data(pix_record),.dst_data(event_core));
 reg [31:0] seen_event,seen_source,seen_render,seen_display,window,window_seq;
 reg [15:0] c_sensor,c_capture,c_render,c_new,c_scan,c_repeat;
 reg [31:0] last_render_latency,last_hdmi_latency,min_hdmi_latency,max_hdmi_latency;
 reg busy_d,overflow_d;reg [31:0] render_begin,render_cycles,fault_frames,overflow_events;
 reg [31:0] dimensions,last_display,flags;
 (* syn_keep=1,syn_preserve=1,PAP_MARK_DEBUG="1" *) reg [511:0] snapshot;
 assign probe=snapshot;
 wire new_event=event_core[191:160]!=seen_event;
 wire event_valid=event_core[2]&&event_core[1];
 wire event_new=event_core[159:128]!=seen_display;
 wire [31:0] event_latency=event_core[127:96];
 wire [15:0] inc_sensor={15'd0,(cam_s!=cam_seen)},inc_capture={15'd0,(source_id!=seen_source)},inc_render={15'd0,(render_id!=seen_render)};
 wire [15:0] inc_scan={15'd0,new_event},inc_new={15'd0,(new_event&&event_valid&&event_new)},inc_repeat={15'd0,(new_event&&event_valid&&!event_new)};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   tick<=0;gray_tick<=0;cam_m<=0;cam_s<=0;cam_seen<=0;seen_event<=0;seen_source<=0;seen_render<=0;seen_display<=0;
   window<=0;window_seq<=0;c_sensor<=0;c_capture<=0;c_render<=0;c_new<=0;c_scan<=0;c_repeat<=0;
   last_render_latency<=0;last_hdmi_latency<=0;min_hdmi_latency<=32'hffffffff;max_hdmi_latency<=0;
   busy_d<=0;overflow_d<=0;render_begin<=0;render_cycles<=0;fault_frames<=0;overflow_events<=0;
   dimensions<=0;last_display<=0;flags<=0;snapshot<=0;
  end else begin
   tick<=tick+1'b1;gray_tick<=(tick+1'b1)^((tick+1'b1)>>1);
   cam_m<=cam_toggle;cam_s<=cam_m;cam_seen<=cam_s;
   busy_d<=render_busy;overflow_d<=capture_overflow;
   if(render_busy&&!busy_d)render_begin<=tick;
   if(!render_busy&&busy_d)render_cycles<=tick-render_begin;
   if(capture_overflow&&!overflow_d)overflow_events<=overflow_events+1'b1;
   if(source_id!=seen_source)seen_source<=source_id;
   if(render_id!=seen_render)begin seen_render<=render_id;last_render_latency<=tick-render_origin[31:0];end
   if(new_event)begin
    seen_event<=event_core[191:160];dimensions<=event_core[63:32];last_display<=event_core[159:128];
    if(!event_core[2]&&event_core[1])fault_frames<=fault_frames+1'b1;
    if(event_valid)begin
     seen_display<=event_core[159:128];
     if(event_new&&!event_latency[31])begin
      last_hdmi_latency<=event_latency;flags[1]<=1;
      if(event_latency<min_hdmi_latency)min_hdmi_latency<=event_latency;
      if(event_latency>max_hdmi_latency)max_hdmi_latency<=event_latency;
     end
    end
   end
   if(window==CLK_HZ-1)begin
    // Include boundary events in this window, avoiding a lost frame each second.
    snapshot<={32'h45563331,CLK_HZ,window_seq+1'b1,
      c_sensor+inc_sensor,c_capture+inc_capture,
      c_render+inc_render,c_new+inc_new,
      c_scan+inc_scan,c_repeat+inc_repeat,
      last_render_latency,last_hdmi_latency,min_hdmi_latency,max_hdmi_latency,render_cycles,
      fault_frames,overflow_events,last_display,dimensions,(flags|32'd1)};
    window<=0;window_seq<=window_seq+1'b1;c_sensor<=0;c_capture<=0;c_render<=0;c_new<=0;c_scan<=0;c_repeat<=0;flags[0]<=1;
   end else begin
    window<=window+1'b1;
    if(cam_s!=cam_seen)c_sensor<=c_sensor+1'b1;
    if(source_id!=seen_source)c_capture<=c_capture+1'b1;
    if(render_id!=seen_render)c_render<=c_render+1'b1;
    if(new_event)begin c_scan<=c_scan+1'b1;if(event_valid)begin if(event_new)c_new<=c_new+1'b1;else c_repeat<=c_repeat+1'b1;end end
   end
  end
 end
endmodule
