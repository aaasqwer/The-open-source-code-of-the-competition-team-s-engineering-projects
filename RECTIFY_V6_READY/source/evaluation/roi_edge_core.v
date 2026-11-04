`timescale 1ns/1ps
// V1.8.1: 4+4 pixel box-difference, gap merging and cross-row continuity.
// Cyan trace is previous-frame strongest edge, not an MTF result.
// Streaming engineering gate for a single near-vertical edge, NOT chart identity,
// SFR or straightness. Pixels sampled before any overlay. One result per frame.
module roi_edge_core(input clk,rst_n,vs,de,input [15:0] pixel,
 input enabled,input [2:0] position,input [1:0] mode,
 input [10:0] x,y,display_x,display_y,graph_x,graph_y,input [7:0] luminance,input external_cancel,
 output reg [3:0] status,output [15:0] overlay_pixel,
 output measurement_valid,output [15:0] rms_q8,max_q8,output [8:0] used_rows,output [7:0] worst_row,output [7:0] coverage_percent,output endpoint_peak,output signed [31:0] fit_slope_q16,fit_intercept_q16);
 reg last_de,last_vs;
 reg previous_enabled;reg [2:0] previous_position;reg [1:0] previous_mode;
 wire changed=external_cancel||enabled!=previous_enabled||position!=previous_position||mode!=previous_mode;
 wire [10:0] left=(position==1||position==3)?11'd96:
                      (position==2||position==4)?11'd1696:11'd896;
 wire [10:0] top=(position==1||position==2)?11'd336:
                     (position==3||position==4)?11'd728:11'd476;
 wire [8:0] roi_height=position==0?9'd128:9'd256;
 wire [8:0] minimum_rows=position==0?9'd122:9'd244;
 wire [15:0] expected_samples=position==0?16'd16384:16'd32768;
 wire [15:0] clipping_limit=position==0?16'd12288:16'd24576;
 wire inside_roi=x>=left&&x<left+128&&y>=top&&y<top+roi_height;
 wire active=enabled&&mode!=2;
 wire display_inside=display_x>=left&&display_x<left+128&&display_y>=top&&display_y<top+roi_height;
 wire border=display_inside&&(display_x<left+3||display_x>=left+125||display_y<top+3||display_y>=top+roi_height-3);
 reg [7:0] trace[0:255];reg trace_ready;
 wire [7:0] roi_y=y-top;wire [7:0] display_row=display_y-top;
 wire [7:0] trace_value=trace[display_row];
 wire [10:0] trace_x=left+{4'b0,trace_value[6:0]};
 wire on_trace=display_inside&&trace_ready&&trace_value[7]&&display_x+1>=trace_x&&display_x<=trace_x+1;
 // Residual graph: horizontal axis = scan row, vertical = signed display_x residual.
 // Shared fixed scale 8 screen pixels per image pixel, +/- 14 px with clipping.
 wire graph_area=graph_x>=680&&graph_x<1192&&graph_y>=820&&graph_y<1060;
 wire [7:0] graph_row=position==0?((graph_x-680)>>2):((graph_x-680)>>1);
 wire signed [15:0] graph_residual;wire graph_valid;
 wire signed [16:0] graph_offset=$signed(graph_residual)>>>5;
 wire signed [16:0] graph_clamped=graph_offset>112?17'sd112:graph_offset< -112?-17'sd112:graph_offset;
 wire signed [17:0] graph_py=18'sd940-$signed(graph_clamped);
 wire graph_point=graph_area&&graph_valid&&$signed({1'b0,graph_y})>=graph_py-1&&$signed({1'b0,graph_y})<=graph_py+1;
 wire graph_grid=graph_y==940||graph_y==876||graph_y==1004;
 wire [15:0] graph_pixel=graph_point?((graph_offset>112||graph_offset< -112)?16'hf800:16'h07ff):(graph_grid?16'h7bef:16'h0841);
 wire [7:0] fit_value;wire [6:0] worst_x;
 wire [10:0] marker_x=left+{4'd0,worst_x},marker_y=top+{3'd0,worst_row};
 wire marker_on=measurement_valid&&display_inside&&
 (((display_x+6>=marker_x&&display_x<=marker_x+6)&&(display_y+1>=marker_y&&display_y<=marker_y+1))||
 ((display_y+6>=marker_y&&display_y<=marker_y+6)&&(display_x+1>=marker_x&&display_x<=marker_x+1)));
 wire [16:0] rows_times_100={8'd0,used_rows}*17'd100;
 assign coverage_percent=position==0?((rows_times_100+64)>>7):((rows_times_100+128)>>8);
 assign endpoint_peak=worst_row<16||{1'b0,worst_row}>=roi_height-16;
 wire [10:0] fit_x=left+{4'd0,fit_value[6:0]};
 wire fit_on=display_inside&&measurement_valid&&fit_value[7]&&display_x==fit_x;
 assign overlay_pixel=active&&graph_area?graph_pixel:(active&&marker_on?16'hfd20:(active&&border?(status==2?16'h07e0:(status==1||status==6?16'hffe0:16'hf800)):(active&&fit_on?16'hf81f:(active&&on_trace?16'h07ff:pixel))));
 wire [7:0] r={pixel[15:11],pixel[15:13]},g={pixel[10:5],pixel[10:9]},b={pixel[4:0],pixel[4:2]};
 wire [9:0] total={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b};
 wire [7:0] lum=luminance;
 // Seven delayed pixels plus current pixel form adjacent four-pixel means.
 reg [7:0] d1,d2,d3,d4,d5,d6,d7;
 wire [9:0] sum_r={2'b0,lum}+{2'b0,d1}+{2'b0,d2}+{2'b0,d3};
 wire [9:0] sum_l={2'b0,d4}+{2'b0,d5}+{2'b0,d6}+{2'b0,d7};
 wire [9:0] response=sum_r>=sum_l?sum_r-sum_l:sum_l-sum_r;
 reg [7:0] row_min,row_max;
 reg row_pending,in_cluster;reg [2:0] gap;
 reg [9:0] cluster_peak,best_peak,second_peak;
 reg [6:0] cluster_x,best_x,second_x;
 reg [9:0] prev_response,cluster_l,cluster_r,best_l,best_r;
 reg cluster_positive,best_positive;reg [8:0] positive_rows,negative_rows;
 wire final_positive=response>cluster_peak?(sum_r>=sum_l):cluster_positive;
 wire [9:0] final_peak=response>cluster_peak?response:cluster_peak;
 wire [6:0] final_x=response>cluster_peak?x-left-3:cluster_x;
 reg [8:0] good_rows,multiple_rows;reg [7:0] min_peak,max_peak;
 reg [7:0] previous_row_x,previous_second_x;
 reg previous_row_valid,previous_second_valid;
 wire [7:0] row_drift=best_x>=previous_row_x?best_x-previous_row_x:previous_row_x-best_x;
 wire [7:0] second_drift=second_x>=previous_second_x?second_x-previous_second_x:previous_second_x-second_x;
 wire row_candidate=best_peak>=128&&row_max-row_min>=48&&best_x>=8&&best_x<=119;
 wire secondary=second_peak>=128&&second_peak>=(best_peak>>1);
 reg [15:0] sample_count,clipped;reg [22:0] lum_sum;
 reg [7:0] frame_min,frame_max,previous_center,previous_mean;
 reg [1:0] stable_frames;reg previous_good;
 wire [8:0] center_sum={1'b0,min_peak}+{1'b0,max_peak};
 wire [7:0] center=center_sum[8:1],mean_lum=position==0?lum_sum[21:14]:lum_sum[22:15];
 wire [7:0] drift=center>=previous_center?center-previous_center:previous_center-center;
 wire [7:0] light_drift=mean_lum>=previous_mean?mean_lum-previous_mean:previous_mean-mean_lum;
 // A missing row must not reset edge identity: resume against the last accepted point.
 reg [8:0] gap_rows,max_gap;reg chain_seen,chain_bad,last_polarity;reg [6:0] chain_x;
 wire [7:0] chain_drift=best_x>=chain_x?best_x-chain_x:chain_x-best_x;
 wire chain_step_ok=!chain_seen||(chain_drift<=6&&best_positive==last_polarity);
 wire chain_ok=!chain_bad&&max_gap<=4&&gap_rows<=4;
 wire frame_accepted=sample_count==expected_samples&&clipped<=clipping_limit&&frame_max-frame_min>=48&&
  multiple_rows<(position==0?24:48)&&!(positive_rows>=(position==0?16:32)&&negative_rows>=(position==0?16:32))&&good_rows>=minimum_rows&&max_peak-min_peak<=40&&
  chain_ok; // Spatial identity/coverage gates remain; first good ROI is measurable.
 wire engine_valid;
 assign measurement_valid=engine_valid&&active&&!changed&&status==2;
 straightness_meter meter(.clk(clk),.rst_n(rst_n),.cancel(changed||!active),
  .frame_start(vs&&!last_vs),.frame_ok(frame_accepted),
  .min_rows(minimum_rows),.point_we(row_pending&&row_candidate&&chain_step_ok),
  .point_y(roi_y),.point_x(best_x),.peak_l(best_l),.peak_c(best_peak),.peak_r(best_r),
  .display_y(display_row),.fitted_x(fit_value),.valid(engine_valid),.rms_q8(rms_q8),.max_q8(max_q8),.used_rows(used_rows),.worst_x(worst_x),.worst_y(worst_row),.curve_y(graph_row),.curve_q8(graph_residual),.curve_valid(graph_valid),.fit_slope_q16(fit_slope_q16),.fit_intercept_q16(fit_intercept_q16));
 always @(posedge clk)begin
  if(!rst_n)begin
   last_de<=0;last_vs<=0;status<=0;gap_rows<=0;max_gap<=0;chain_seen<=0;chain_bad<=0;chain_x<=0;last_polarity<=0;
   previous_enabled<=0;previous_position<=0;previous_mode<=0;
   good_rows<=0;multiple_rows<=0;min_peak<=127;max_peak<=0;sample_count<=0;clipped<=0;lum_sum<=0;
   frame_min<=255;frame_max<=0;row_pending<=0;stable_frames<=0;previous_good<=0;
   previous_center<=0;previous_mean<=0;row_min<=255;row_max<=0;
   d1<=0;d2<=0;d3<=0;d4<=0;d5<=0;d6<=0;d7<=0;
   in_cluster<=0;gap<=0;cluster_peak<=0;cluster_x<=0;best_peak<=0;second_peak<=0;best_x<=0;second_x<=0;prev_response<=0;cluster_l<=0;cluster_r<=0;best_l<=0;best_r<=0;
   positive_rows<=0;negative_rows<=0;cluster_positive<=0;best_positive<=0;previous_row_x<=0;previous_second_x<=0;previous_row_valid<=0;previous_second_valid<=0;trace_ready<=0;
  end else begin
   last_de<=de;last_vs<=vs;previous_enabled<=enabled;previous_position<=position;previous_mode<=mode;
   if(changed||!enabled||mode==2)begin
    status<=!enabled?0:(mode==2?7:1);stable_frames<=0;previous_good<=0;trace_ready<=0;
   end
   if(vs&&!last_vs||changed||!active)begin
    gap_rows<=0;max_gap<=0;chain_seen<=0;chain_bad<=0;
    good_rows<=0;multiple_rows<=0;min_peak<=127;max_peak<=0;sample_count<=0;clipped<=0;lum_sum<=0;
    frame_min<=255;frame_max<=0;row_pending<=0;positive_rows<=0;negative_rows<=0;previous_row_valid<=0;previous_second_valid<=0;
    if(vs&&!last_vs&&active&&!changed)begin
     previous_good<=0;stable_frames<=0;trace_ready<=sample_count==expected_samples;
     if(sample_count!=expected_samples)status<=1;
     else if(clipped>clipping_limit)status<=8;
     else if(frame_max-frame_min<48)status<=3;
     else if(multiple_rows>=(position==0?24:48)||(positive_rows>=(position==0?16:32)&&negative_rows>=(position==0?16:32)))status<=5;
     else if(!chain_ok)status<=9;
     else if(good_rows<minimum_rows||max_peak-min_peak>40)status<=4;
     else begin
      previous_center<=center;previous_mean<=mean_lum;previous_good<=1;
      if(previous_good&&drift<=3&&light_drift<=12)stable_frames<=stable_frames==3?3:stable_frames+1'b1;
      else stable_frames<=1;
      status<=2; // First valid spatial fit; temporal stability is a separate statistic.
     end
    end
   end else if(active)begin
    if(de&&inside_roi)begin
     sample_count<=sample_count+1'b1;lum_sum<=lum_sum+lum;
     if(lum<=4||lum>=251)clipped<=clipped+1'b1;
     if(lum<frame_min)frame_min<=lum;if(lum>frame_max)frame_max<=lum;
     d1<=lum;d2<=d1;d3<=d2;d4<=d3;d5<=d4;d6<=d5;d7<=d6;
     if(x==left)begin
      row_min<=lum;row_max<=lum;in_cluster<=0;gap<=0;
      cluster_peak<=0;cluster_x<=0;best_peak<=0;second_peak<=0;best_x<=0;second_x<=0;prev_response<=0;cluster_l<=0;cluster_r<=0;best_l<=0;best_r<=0;
     end else begin
      if(lum<row_min)row_min<=lum;if(lum>row_max)row_max<=lum;
      if(x>=left+7)begin
       prev_response<=response;
       if(in_cluster&&x-left-3==cluster_x+1)cluster_r<=response;
       if(!in_cluster)begin
        if(response>=96)begin in_cluster<=1;cluster_peak<=response;cluster_x<=x-left-3;cluster_positive<=sum_r>=sum_l;cluster_l<=prev_response;cluster_r<=0;gap<=0;end
       end else begin
        if(response>cluster_peak)begin cluster_peak<=response;cluster_x<=x-left-3;cluster_positive<=sum_r>=sum_l;cluster_l<=prev_response;cluster_r<=0;end
        if(response>=48)gap<=0;else if(gap<6)gap<=gap+1'b1;
       end
       // Join responses separated by <=6 low-response pixels; finalize each peak once.
       if(in_cluster&&((response<48&&gap==6)||x==left+127))begin
        in_cluster<=0;gap<=0;
        if(final_peak>best_peak)begin
         second_peak<=best_peak;second_x<=best_x;best_peak<=final_peak;best_x<=final_x;best_positive<=final_positive;best_l<=cluster_l;best_r<=cluster_r;
        end else if(final_peak>second_peak)begin second_peak<=final_peak;second_x<=final_x;end
       end
      end
     end
     if(x==left+127)row_pending<=1;
    end
    if(row_pending)begin
     row_pending<=0;
     if(row_candidate)begin
      if(!chain_step_ok)chain_bad<=1;
      chain_seen<=1;chain_x<=best_x;last_polarity<=best_positive;gap_rows<=0;
     end else begin
      gap_rows<=gap_rows+1'b1;
      if(gap_rows+1>max_gap)max_gap<=gap_rows+1'b1;
     end
     trace[y-top]<={row_candidate,best_x};
     if(row_candidate)begin
      if(best_positive)positive_rows<=positive_rows+1'b1;else negative_rows<=negative_rows+1'b1;
     end
     previous_row_x<={1'b0,best_x};previous_row_valid<=row_candidate;
     previous_second_x<={1'b0,second_x};previous_second_valid<=secondary;
     if(secondary&&previous_second_valid&&second_drift<=6)multiple_rows<=multiple_rows+1'b1;
     if(row_candidate&&chain_step_ok)begin
      good_rows<=good_rows+1'b1;
      if(best_x<min_peak)min_peak<=best_x;if(best_x>max_peak)max_peak<=best_x;
     end
    end
   end
  end
 end
endmodule
