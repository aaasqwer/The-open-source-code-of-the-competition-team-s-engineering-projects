`timescale 1ns/1ps
// Local measurement is deliberately a diagnostic, not a calibration verdict.
// Preserve RGB888 video outside painted pixels, including when Gamma is used.
module rect_evaluation_overlay(
 input clk,rst_n,vs,hs,de,input [11:0] x,y,input [23:0] rgb,
 input numbers_visible,enabled,horizontal,editing,input [2:0] position,input [10:0] center_x,center_y,
 input [31:0] actual_geometry,input calibration_ready,calibration_fault,input [351:0] parameters,
 output out_vs,out_hs,out_de,output [11:0] out_x,out_y,output [23:0] out_rgb
);
 wire [15:0] raw565={rgb[23:19],rgb[15:10],rgb[7:3]};
 wire [1:0] mode=actual_geometry[1:0]==3?2:actual_geometry[1:0]==0?0:1;
 wire [3:0] status,sfr_status;wire [15:0] evaluated,rms_q8,max_q8,mtf50,sfr_quarter,sfr_frame_rms,sfr_angle;
 wire valid,graph_visible,sfr_graph_visible,sfr_valid,sfr_above,sfr_info_valid,endpoint_peak;
 wire [8:0] rows;wire [7:0] worst,coverage;wire [2:0] samples;
 roi_evaluator evaluator(.clk(clk),.rst_n(rst_n),.vs(vs),.de(de),.pixel(raw565),
  .enabled(enabled&&calibration_ready),.position(position),.mode(mode),.horizontal(horizontal),.editing(editing),.center_x(center_x),.center_y(center_y),
  .status(status),.overlay_pixel(evaluated),.measurement_valid(valid),.rms_q8(rms_q8),.max_q8(max_q8),.used_rows(rows),.worst_row(worst),.coverage_percent(coverage),.endpoint_peak(endpoint_peak),
  .graph_visible(graph_visible),.sfr_graph_visible(sfr_graph_visible),.sfr_valid(sfr_valid),.sfr_above(sfr_above),.sfr_status(sfr_status),
  .mtf50_q16(mtf50),.sfr_quarter_q12(sfr_quarter),.sfr_info_valid(sfr_info_valid),.sfr_frame_rms(sfr_frame_rms),.sfr_angle_q8(sfr_angle),.sfr_samples(samples));
 wire median_valid;wire [15:0] median_rms,median_max,low,high;wire [8:0] window_rows;
 measurement_window window_stats(.clk(clk),.rst_n(rst_n),.clear(status!=2||!calibration_ready),.sample_valid(valid),.rms_in(rms_q8),.max_in(max_q8),.rows_in(rows),
  .valid(median_valid),.rms_median(median_rms),.max_median(median_max),.range_low(low),.range_high(high),.rows_min(window_rows),.samples());
 wire [15:0] painted;
 board_osd osd(.numbers_visible(numbers_visible),.clk(clk),.rst_n(rst_n),.vs(vs),.hs(hs),.de(de),.pixel(evaluated),.model(1'b1),.mode(mode),
  .evaluation_status(status),.evaluation_position(position),.calibration_ready(calibration_ready),.calibration_fault(calibration_fault),.parameters(parameters),
  .diagnostic_valid(valid),.diagnostic_max(max_q8),.coverage_percent(coverage),.worst_row(worst),.endpoint_peak(endpoint_peak),
  .measurement_valid(median_valid),.rms_q8(median_rms),.max_q8(median_max),.used_rows(window_rows),.range_low(low),.range_high(high),
  .graph_visible(graph_visible),.horizontal(horizontal),.editing(editing),.sfr_graph_visible(sfr_graph_visible),.sfr_valid(sfr_valid),.sfr_above(sfr_above),.sfr_status(sfr_status),
  .mtf50_q16(mtf50),.sfr_quarter_q12(sfr_quarter),.sfr_info_valid(sfr_info_valid),.sfr_frame_rms(sfr_frame_rms),.sfr_angle_q8(sfr_angle),.sfr_samples(samples),
  .out_vs(out_vs),.out_hs(out_hs),.out_de(out_de),.out_pixel(painted));
 reg [23:0] originals[0:2];reg [15:0] quantized[0:2];reg [11:0] xs[0:2],ys[0:2];integer i;
 always @(posedge clk)begin
  originals[0]<=rgb;quantized[0]<=raw565;xs[0]<=x;ys[0]<=y;
  for(i=1;i<3;i=i+1)begin originals[i]<=originals[i-1];quantized[i]<=quantized[i-1];xs[i]<=xs[i-1];ys[i]<=ys[i-1];end
 end
 assign out_x=xs[2];assign out_y=ys[2];
 assign out_rgb=painted==quantized[2]?originals[2]:{painted[15:11],painted[15:13],painted[10:5],painted[10:9],painted[4:0],painted[4:2]};
endmodule
