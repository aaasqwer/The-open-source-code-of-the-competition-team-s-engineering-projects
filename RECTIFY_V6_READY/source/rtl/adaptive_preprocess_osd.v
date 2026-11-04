`timescale 1ns / 1ps
// Policies: 0 legacy scene; 1 V20 balanced correction; 2 V21 shadow priority; 3 V22 bounded additive lift.
// Three registered pixel stages; matching sync/coordinates; frame-only adaptation.

module adaptive_preprocess_osd #(
    parameter GAMMA_POLICY=0, // 0: original illumination rendering; 1: V20 correction; 2: V21; 3: bounded additive shadows
    parameter FRAME_PIXELS = 2073600,
    parameter REQUIRE_VALID_PIXELS=0,
    parameter COMBINED_OSD=0,
    parameter SHOW_ROTATION_TRACE=0,
    parameter EXPOSURE_LOCKED_MODE=0,
    parameter SHOW_REFERENCE = 1'b0,
    parameter GAMMA_BYPASS = 1'b0
) (
    input hold_frame,runtime_bypass,osd_enable,
    input [34:0] lens_state_live,
    input             clk,
    input             rstn,
    input             vs_in,
    input             hs_in,
    input             de_in,
    input             pixel_valid,
    input      [11:0] x_in,
    input      [11:0] y_in,
    input      [15:0] rgb_in,
    input      [63:0] rotation_trace,
    input [7:0] camera_fps_bcd_live, input camera_fps_valid_live,
    input [15:0] camera_width_bcd_live, input [15:0] camera_height_bcd_live,
    input camera_geometry_valid_live, input camera_geometry_bad_live,
    input camera_light_valid_live,
    input [7:0] camera_light_index_live, camera_light_error_live,
    input [15:0] camera_exposure_lines_live,
    input [9:0] camera_gain_q4_live,
    input [11:0] camera_light_metric_q4_live,
    output wire [11:0] x_out, y_out,
    output reg        vs_out,
    output reg        hs_out,
    output reg        de_out,
    output reg [15:0] rgb_out,
    output reg [23:0] rgb888_out,
    output wire [15:0] clean_rgb,
    output reg [7:0]  mean_luma,
    output reg [8:0]  gain_q8,
    output reg [7:0]  contrast,
    output wire [1:0] gamma_mode,
    output reg [3:0] gamma_step,
    output reg [4:0] tone_step
);

    reg [7:0] camera_fps_bcd;
    reg [0:0] camera_fps_valid;
    reg [15:0] camera_width_bcd;
    reg [15:0] camera_height_bcd;
    reg [0:0] camera_geometry_valid;
    reg [0:0] camera_geometry_bad;
    reg [0:0] camera_light_valid;
    reg [7:0] camera_light_index;
    reg [7:0] camera_light_error;
    reg [15:0] camera_exposure_lines;
    reg [9:0] camera_gain_q4;
    reg [11:0] camera_light_metric_q4;
    always @(posedge clk) begin
      if(!rstn) begin
        camera_fps_bcd<=0;
        camera_fps_valid<=0;
        camera_width_bcd<=0;
        camera_height_bcd<=0;
        camera_geometry_valid<=0;
        camera_geometry_bad<=0;
        camera_light_valid<=0;
        camera_light_index<=0;
        camera_light_error<=0;
        camera_exposure_lines<=0;
        camera_gain_q4<=0;
        camera_light_metric_q4<=0;
      end else if(!hold_frame) begin
        camera_fps_bcd<=camera_fps_bcd_live;
        camera_fps_valid<=camera_fps_valid_live;
        camera_width_bcd<=camera_width_bcd_live;
        camera_height_bcd<=camera_height_bcd_live;
        camera_geometry_valid<=camera_geometry_valid_live;
        camera_geometry_bad<=camera_geometry_bad_live;
        camera_light_valid<=camera_light_valid_live;
        camera_light_index<=camera_light_index_live;
        camera_light_error<=camera_light_error_live;
        camera_exposure_lines<=camera_exposure_lines_live;
        camera_gain_q4<=camera_gain_q4_live;
        camera_light_metric_q4<=camera_light_metric_q4_live;
      end
    end
    // Freeze the diagnostic bundle at the first active pixel. It then stays
    // stable through every text glyph of this display frame.
    reg [63:0] rotation_trace_frame;
    reg [34:0] lens_state_frame;
    always @(posedge clk or negedge rstn)
      if(!rstn)lens_state_frame<=0;
      else if(!hold_frame && de_in && x_in==0 && y_in==0)lens_state_frame<=lens_state_live;
    always @(posedge clk or negedge rstn)
      if(!rstn) rotation_trace_frame<=0;
      else if(!hold_frame && de_in && x_in==0 && y_in==0) rotation_trace_frame<=rotation_trace;

    // A single common RGB scale changes HSV Value, preserving channel ratios.
    // Prefer exposure-normalized sensor telemetry; otherwise use image luma.
    // Both are relative image-scene proxies, not calibrated ambient lux.
    reg [15:0] filtered_mean_q8;
    reg [1:0] adapt_count;
    reg filter_valid;
    assign gamma_mode = (GAMMA_POLICY>=2) ? (tone_step==0 ? 2'd1 : 2'd2) : GAMMA_POLICY ? ((tone_step<8)?2'd2:(tone_step>8)?2'd0:2'd1) :
                        ((gamma_step<4)?2'd0:(gamma_step>4)?2'd2:2'd1);
    reg [4:0] tone_pending;
    reg [21:0] shadow_count;
    reg backlit_scene, tone_noise_risk;
    reg [21:0] dark_detail_count,deep_detail_count,nonblack_count;
    reg shadow_detail,deep_detail,any_detail,black_dominated;
    function [4:0] shadow_target;
      input [7:0] mean;
      input dark_present,deep_present,content_present,noise_present,meter_valid;
      reg [4:0] wanted;
      begin
        if(mean<80)wanted=16;
        else if(mean<96)wanted=14;
        else if(mean<112)wanted=12;
        else if(mean<128)wanted=8;
        else if(mean<144)wanted=4;
        else if(mean<160)wanted=2;
        else wanted=0;
        // Meter actual nonzero dark pixels, so a white window does not
        // switch off all shadow lift. Completely black data is not recoverable.
        if(dark_present && wanted<12)wanted=12;
        if(deep_present)wanted=16;
        if(noise_present && wanted>8)wanted=8;
        // V22 is conservative even when telemetry is unavailable. Preserve
        // V21 as an explicit option; do not use invalid gain as evidence of low noise.
        if(GAMMA_POLICY==3) begin
          if(wanted>5)wanted=5;
          if((noise_present || !meter_valid) && wanted>2)wanted=2;
        end
        shadow_target=content_present ? wanted : 0;
      end
    endfunction
    wire [4:0] shadow_direct_target=shadow_target(mean_luma,shadow_detail,deep_detail,any_detail,tone_noise_risk,camera_light_valid);
    wire [4:0] shadow_filtered_target=shadow_target(filtered_next[15:8],shadow_detail,deep_detail,any_detail,tone_noise_risk,camera_light_valid);
    wire [7:0] adapt_luma=GAMMA_POLICY ? mean_luma : control_luma;
    wire [7:0] tone_up=8'd72+({3'd0,tone_pending}<<3);
    wire [7:0] tone_down=8'd60+({3'd0,tone_pending}<<3);
    function [7:0] rise_threshold;
        input [3:0] step;
        begin
            case(step)
                0:rise_threshold=48; 1:rise_threshold=72;
                2:rise_threshold=96; 3:rise_threshold=112;
                4:rise_threshold=144; 5:rise_threshold=168;
                6:rise_threshold=192; default:rise_threshold=216;
            endcase
        end
    endfunction
    reg vs_d,frame_bad;
    reg [31:0] sum_luma, mean_remainder;
    reg [21:0] pixel_count;
    reg [21:0] highlight_count;
    reg highlight_risk;
    reg [7:0] min_luma, max_luma, mean_work;
    reg [2:0] mean_bit;
    reg mean_busy;
    wire [7:0] r8 = {rgb_in[15:11], rgb_in[15:13]};
    wire [7:0] g8 = {rgb_in[10:5], rgb_in[10:9]};
    wire [7:0] b8 = {rgb_in[4:0], rgb_in[4:2]};
    // Integer luminance approximation: (5R + 9G + 2B + 8) / 16.
    wire [11:0] luma_sum = ({4'd0,r8} << 2) + r8 +
                            ({4'd0,g8} << 3) + g8 +
                            ({4'd0,b8} << 1) + 12'd8;
    wire [7:0] luma_now = luma_sum[11:4];
    wire vs_rise = vs_in & ~vs_d;
    wire [31:0] mean_threshold = FRAME_PIXELS << mean_bit;
    wire mean_take_bit = mean_remainder >= mean_threshold;
    wire [7:0] mean_next = mean_take_bit ?
                              (mean_work | (8'd1 << mean_bit)) : mean_work;
    wire [7:0] control_luma;
    wire reference_ready,light_result_valid,light_result_from_ae;
    wire [11:0] reference_q4;
    scene_light_reference reference_control(
        .clk(clk),.rstn(rstn),.sample_pulse(!hold_frame && mean_busy && mean_bit==0),
        .telemetry_valid(camera_light_valid),.metric_q4(camera_light_metric_q4),
        .raw_light(camera_light_index),.image_mean(mean_next),.image_contrast(contrast),
        .gain_q4(camera_gain_q4),.reference_ready(reference_ready),.reference_q4(reference_q4),
        .result_valid(light_result_valid),.result_luma(control_luma),.result_from_ae(light_result_from_ae));
    wire [18:0] filter_sum = ({3'd0,filtered_mean_q8} << 3) -
                             filtered_mean_q8 + {3'd0,adapt_luma,8'd0} + 19'd4;
    wire [15:0] filtered_next = filter_valid ? filter_sum[18:3] : {adapt_luma,8'd0};

    // Divide by the actual pixel count over eight blanking cycles. Accept
    // only complete frames, so startup/short frames cannot change exposure.
    // Curves preserve black/white; no extra clipping gain is applied.
    always @(posedge clk) begin
        if (!rstn) begin
            vs_d <= 0; frame_bad<=0; sum_luma <= 0; pixel_count <= 0;
            highlight_count <= 0; highlight_risk <= 0;
            shadow_count<=0;backlit_scene<=0;tone_noise_risk<=0;tone_step<=(GAMMA_POLICY>=2 ? 0 : 8);tone_pending<=(GAMMA_POLICY>=2 ? 0 : 8);
            dark_detail_count<=0;deep_detail_count<=0;nonblack_count<=0;
            shadow_detail<=0;deep_detail<=0;any_detail<=0;black_dominated<=0;
            min_luma <= 255; max_luma <= 0;
            mean_luma <= 128; contrast <= 0;
            gamma_step <= 4; gain_q8 <= 256;
            filtered_mean_q8 <= 16'd32768; adapt_count <= 0; filter_valid <= 0;
            mean_remainder <= 0; mean_work <= 0;
            mean_bit <= 0; mean_busy <= 0;
        end else begin
            vs_d <= vs_in;
            if(!hold_frame) begin
            if (vs_rise) begin
                if (pixel_count == FRAME_PIXELS && !frame_bad && (GAMMA_POLICY<2 || !de_in)) begin
                    mean_remainder <= sum_luma;
                    mean_work <= 0;
                    mean_bit <= 7;
                    mean_busy <= 1;
                    contrast <= max_luma - min_luma;
                    any_detail<=nonblack_count!=0;
                    black_dominated<=nonblack_count<=(FRAME_PIXELS-(FRAME_PIXELS+3)/4);
                    if(dark_detail_count>=((FRAME_PIXELS+7)/8))shadow_detail<=1;
                    else if(dark_detail_count<((FRAME_PIXELS+15)/16))shadow_detail<=0;
                    if(deep_detail_count>=((FRAME_PIXELS+15)/16))deep_detail<=1;
                    else if(deep_detail_count<((FRAME_PIXELS+31)/32))deep_detail<=0;
                    backlit_scene <= (shadow_count >= ((FRAME_PIXELS+3)/4)) &&
                                     (highlight_count >= ((FRAME_PIXELS+15)/16));
                    tone_noise_risk <= camera_light_valid && camera_gain_q4>=128;
                    // Commit only at a frame boundary, never on a DE pixel.
                    if(GAMMA_POLICY && !de_in) begin tone_step<=tone_pending;gamma_step<=tone_pending>>1;end
                    highlight_risk <= highlight_count >= ((FRAME_PIXELS+15)/16);
                end
                sum_luma <= 0; pixel_count <= 0; frame_bad<=0;
                highlight_count <= 0;shadow_count<=0;
                dark_detail_count<=0;deep_detail_count<=0;nonblack_count<=0;
                min_luma <= 255; max_luma <= 0;
            end else if (de_in) begin
                if(REQUIRE_VALID_PIXELS && !pixel_valid) frame_bad<=1;
                if (pixel_count <= FRAME_PIXELS)
                    pixel_count <= pixel_count + 1'b1;
                if (pixel_count <= FRAME_PIXELS && luma_now >= 240)
                    highlight_count <= highlight_count + 1'b1;
                if(pixel_count<=FRAME_PIXELS && luma_now<=48)shadow_count<=shadow_count+1'b1;
                if(pixel_count<=FRAME_PIXELS && luma_now!=0)begin
                    nonblack_count<=nonblack_count+1'b1;
                    if(luma_now<=48)dark_detail_count<=dark_detail_count+1'b1;
                    if(luma_now<=16)deep_detail_count<=deep_detail_count+1'b1;
                end
                sum_luma <= sum_luma + luma_now;
                if (luma_now < min_luma) min_luma <= luma_now;
                if (luma_now > max_luma) max_luma <= luma_now;
            end
            if (mean_busy) begin
                if (mean_take_bit)
                    mean_remainder <= mean_remainder - mean_threshold;
                mean_work <= mean_next;
                if (mean_bit != 0) mean_bit <= mean_bit - 1'b1;
                else begin
                    mean_busy <= 0;
                    mean_luma <= mean_next;
                end
            end
            if (light_result_valid) begin
                filtered_mean_q8 <= filtered_next;
                filter_valid <= 1;
                adapt_count <= adapt_count + 1'b1;
                // One curve per four complete frames. Avoid extra brightening
                // when at least 1/16 of the input has luma >= 240.
                if (adapt_count == 3) begin
                  if(GAMMA_POLICY>=2) begin
                    if(tone_pending<shadow_direct_target && tone_pending<shadow_filtered_target)
                      tone_pending<=tone_pending+1'b1;
                    else if(tone_pending>shadow_direct_target && tone_pending>shadow_filtered_target)
                      tone_pending<=tone_pending-1'b1;
                  end else if(GAMMA_POLICY) begin
                    // Neutral band 124..135, filtered and direct measurements
                    // must agree. 17 steps, <=1 step / 4 accepted frames.
                    // Backlighting allows mild shadow lift even with white areas.
                    if((backlit_scene && tone_pending<6) || (tone_noise_risk && tone_pending<4))tone_pending<=tone_pending+1'b1;
                    else if(backlit_scene && tone_pending>6)tone_pending<=tone_pending-1'b1;
                    else if(tone_pending<16 && !backlit_scene &&
                            filtered_next[15:8]>=tone_up && mean_luma>=tone_up)
                      tone_pending<=tone_pending+1'b1;
                    else if(tone_pending>0 && !(backlit_scene && tone_pending<=6) && !(tone_noise_risk && tone_pending<=4) &&
                            filtered_next[15:8]<tone_down && mean_luma<tone_down)
                      tone_pending<=tone_pending-1'b1;
                  end else begin
                    if (highlight_risk && gamma_step > 4)
                        gamma_step <= gamma_step - 1'b1;
                    else if (gamma_step < 8 && !(highlight_risk && gamma_step >= 4) &&
                             filtered_next[15:8] >= rise_threshold(gamma_step)+8'd4 &&
                             control_luma >= rise_threshold(gamma_step)+8'd4)
                        gamma_step <= gamma_step + 1'b1;
                    else if (gamma_step > 0 && filtered_next[15:8] < rise_threshold(gamma_step-1'b1)-8'd4 &&
                             control_luma < rise_threshold(gamma_step-1'b1)-8'd4)
                        gamma_step <= gamma_step - 1'b1;
                  end
                end
            end
        end
        end // held scene adaptation
    end

    wire [15:0] corrected_rgb;
    wire [23:0] corrected_rgb888;
    wire vs_gamma, hs_gamma, de_gamma;
    wire [11:0] x_gamma, y_gamma;
    assign x_out=x_gamma;
    assign y_out=y_gamma;
    gamma_rgb_preserve #(.ENHANCED_CURVES(GAMMA_POLICY)) gamma_pixels (
        .clk(clk), .rstn(rstn), .step(GAMMA_POLICY ? ((GAMMA_BYPASS || runtime_bypass) ? (GAMMA_POLICY>=2 ? 5'd0 : 5'd8) : tone_step) : ((GAMMA_BYPASS || runtime_bypass) ? 5'd4 : {1'b0,gamma_step})),
        .rgb_in(rgb_in), .vs_in(vs_in), .hs_in(hs_in), .de_in(de_in),
        .x_in(x_in), .y_in(y_in), .rgb_out(corrected_rgb),.rgb888_out(corrected_rgb888),
        .vs_out(vs_gamma), .hs_out(hs_gamma), .de_out(de_gamma),
        .x_out(x_gamma), .y_out(y_gamma)
    );

    function [7:0] hex_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 10)
                hex_ascii = 8'h30 + nibble;
            else
                hex_ascii = 8'h41 + (nibble - 10);
        end
    endfunction

    // 5x7 font.  Unsupported characters render as a blank cell.
    function [4:0] glyph_row;
        input [7:0] ch;
        input [2:0] row;
        begin
            glyph_row = 5'b00000;
            case (ch)
                8'h2E: case(row) 5:glyph_row=5'b00110;6:glyph_row=5'b00110;default:glyph_row=0;endcase
                8'h3C: case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00100;2:glyph_row=5'b01000;3:glyph_row=5'b10000;4:glyph_row=5'b01000;5:glyph_row=5'b00100;6:glyph_row=5'b00010;endcase
                8'h25: case(row) 0:glyph_row=5'b11001;1:glyph_row=5'b11010;2:glyph_row=5'b00100;3:glyph_row=5'b01000;4:glyph_row=5'b10110;5:glyph_row=5'b00110;6:glyph_row=5'b00000;endcase
                8'h42: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;endcase
                8'h30: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h31: case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110; endcase
                8'h32: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;6:glyph_row=5'b11111; endcase
                8'h33: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110; endcase
                8'h34: case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;6:glyph_row=5'b00010; endcase
                8'h35: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110; endcase
                8'h36: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h37: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b01000; endcase
                8'h38: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h39: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b01110; endcase
                8'h41: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001; endcase
                8'h43: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h44: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110; endcase
                8'h45: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111; endcase
                8'h46: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000; endcase
                8'h47: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h49: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b11111; endcase
                8'h4B: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10010;2:glyph_row=5'b10100;3:glyph_row=5'b11000;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001; endcase
                8'h4C: case(row) 0:glyph_row=5'b10000;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111; endcase
                8'h4D: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11011;2:glyph_row=5'b10101;3:glyph_row=5'b10101;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001; endcase
                8'h4E: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11001;2:glyph_row=5'b10101;3:glyph_row=5'b10011;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001; endcase
                8'h4F: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h50: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000; endcase
                8'h52: case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001; endcase
                8'h53: case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110; endcase
                8'h54: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100; endcase
                8'h55: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110; endcase
                8'h56: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100; endcase
                8'h58: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b01010;5:glyph_row=5'b10001;6:glyph_row=5'b10001; endcase
                8'h59: case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100; endcase
                8'h5A: case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b10000;6:glyph_row=5'b11111; endcase
                8'h2B: case(row) 0:glyph_row=5'b00000;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b11111;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00000; endcase
                8'h2D: case(row) 0:glyph_row=5'b00000;1:glyph_row=5'b00000;2:glyph_row=5'b00000;3:glyph_row=5'b11111;4:glyph_row=5'b00000;5:glyph_row=5'b00000;6:glyph_row=5'b00000; endcase
                8'h3F: case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00000;6:glyph_row=5'b00100; endcase
                default: glyph_row = 5'b00000;
            endcase
        end
    endfunction

    wire [12:0] shadow_boost_numerator={8'd0,tone_step}*13'd300+13'd8;
    wire [9:0] shadow_boost_centi=(GAMMA_BYPASS || runtime_bypass) ? 100 : 100+shadow_boost_numerator[12:4];
    wire [7:0] tone_exponent=(GAMMA_BYPASS || runtime_bypass) ? 8'd100 : 8'd60+tone_step*8'd5;
    function [7:0] osd_char;
        input [2:0] line_no;
        input [5:0] index;
        begin
            osd_char = 8'h20;
            case (line_no)
                3'd0: case(index)
                    0: osd_char=8'h41;
                    1: osd_char=8'h44;
                    2: osd_char=8'h41;
                    3: osd_char=8'h50;
                    4: osd_char=8'h54;
                    5: osd_char=8'h49;
                    6: osd_char=8'h56;
                    7: osd_char=8'h45;
                    9: osd_char=8'h56;
                    10: osd_char=(GAMMA_POLICY==3)?"5":(GAMMA_POLICY>=2)?"4":GAMMA_POLICY?"3":(COMBINED_OSD?"2":8'h38);
                    12: osd_char=8'h31;
                    13: osd_char=8'h30;
                    14: osd_char=8'h38;
                    15: osd_char=8'h30;
                    16: osd_char=8'h50;
                    17: osd_char=8'h33;
                    18: osd_char=8'h30;
                    20:osd_char=(GAMMA_POLICY>=2)?"B":" ";
                    21:osd_char=(GAMMA_POLICY>=2)?"L":" ";
                    22:osd_char=(GAMMA_POLICY>=2)?"K":" ";
                    23:osd_char=(GAMMA_POLICY>=2)?":":" ";
                    24:osd_char=(GAMMA_POLICY>=2)?("0"+black_dominated):" ";
                    26:osd_char=(GAMMA_POLICY>=2)?"D":" ";
                    27:osd_char=(GAMMA_POLICY>=2)?":":" ";
                    28:osd_char=(GAMMA_POLICY>=2)?("0"+shadow_detail):" ";
                    default: osd_char=8'h20;
                endcase
                3'd1: case(index)
                    0:osd_char="M";1:osd_char="E";2:osd_char="A";3:osd_char="N";
                    4:osd_char="0";5:osd_char="X";
                    6:osd_char=hex_ascii(mean_luma[7:4]);7:osd_char=hex_ascii(mean_luma[3:0]);
                    9:osd_char=(GAMMA_POLICY>=2)?"B":"E";10:osd_char=(GAMMA_POLICY>=2)?"S":"X";11:osd_char=(GAMMA_POLICY>=2)?"T":"P";
                    12:osd_char=(GAMMA_POLICY>=2)?("0"+shadow_boost_centi/100):GAMMA_POLICY ? (tone_exponent>=100?"1":"0") : (gamma_step <= 4 || GAMMA_BYPASS) ? "1" : "0";
                    13:osd_char=".";
                    14:if(GAMMA_POLICY>=2)osd_char="0"+(shadow_boost_centi%100)/10;else if(GAMMA_POLICY)osd_char="0"+((tone_exponent%100)/10);else case((GAMMA_BYPASS || runtime_bypass) ? 4'd4 : gamma_step)
                        0:osd_char="1";1,2,3,4:osd_char="0";
                        5,6:osd_char="9";default:osd_char="8";endcase
                    15:if(GAMMA_POLICY>=2)osd_char="0"+shadow_boost_centi%10;else if(GAMMA_POLICY)osd_char="0"+(tone_exponent%10);else case((GAMMA_BYPASS || runtime_bypass) ? 4'd4 : gamma_step)
                        1:osd_char="8";2,5,7:osd_char="5";3:osd_char="3";
                        default:osd_char="0";endcase
                    17:osd_char=(GAMMA_BYPASS || runtime_bypass) ? "B" : (GAMMA_POLICY==3 ? "A" : "R");
                    18:osd_char=(GAMMA_BYPASS || runtime_bypass) ? "Y" : (GAMMA_POLICY==3 ? "D" : "G");
                    19:osd_char=(GAMMA_BYPASS || runtime_bypass) ? "P" : (GAMMA_POLICY==3 ? "D" : "B");
                    21:osd_char=camera_light_valid ? "A" : "I";
                    22:osd_char=camera_light_valid ? "E" : "M";
                    23:osd_char=camera_light_valid ? " " : "G";
                    25:osd_char="L";26:osd_char="0";27:osd_char="X";
                    28:osd_char=camera_light_valid ? hex_ascii(control_luma[7:4]) : "-";
                    29:osd_char=camera_light_valid ? hex_ascii(control_luma[3:0]) : "-";
                    default:osd_char=" ";
                endcase
                3'd2: case(index)
                    0:osd_char="L";
                    1:osd_char="E";
                    2:osd_char="N";
                    3:osd_char="S";
                    4:osd_char=":";
                    5:osd_char=lens_state_frame[0]?"E":lens_state_frame[1]?"O":lens_state_frame[2]?"B":"N";
                    6:osd_char=lens_state_frame[0]?"R":lens_state_frame[1]?"N":lens_state_frame[2]?"Y":"A";
                    7:osd_char=lens_state_frame[0]?"R":lens_state_frame[2]&&!lens_state_frame[1]?"P":" ";
                    9:osd_char="I";10:osd_char="D";11:osd_char=":";
                    12:osd_char=hex_ascii(lens_state_frame[34-:4]);
                    13:osd_char=hex_ascii(lens_state_frame[30-:4]);
                    14:osd_char=hex_ascii(lens_state_frame[26-:4]);
                    15:osd_char=hex_ascii(lens_state_frame[22-:4]);
                    16:osd_char=hex_ascii(lens_state_frame[18-:4]);
                    17:osd_char=hex_ascii(lens_state_frame[14-:4]);
                    18:osd_char=hex_ascii(lens_state_frame[10-:4]);
                    19:osd_char=hex_ascii(lens_state_frame[6-:4]);
                    default:osd_char=" ";
                endcase
                3'd3: case(index)
                    0:osd_char="P";
                    1:osd_char="A";
                    2:osd_char="R";
                    3:osd_char="A";
                    4:osd_char="M";
                    5:osd_char="S";
                    6:osd_char=":";
                    7:osd_char="P";
                    8:osd_char="C";
                    9:osd_char=" ";
                    10:osd_char=" ";
                    11:osd_char=" ";
                    12: osd_char=8'h43;
                    13: osd_char=8'h4F;
                    14: osd_char=8'h4E;
                    15: osd_char=8'h30;
                    16: osd_char=8'h58;
                    17: osd_char=hex_ascii(contrast[7:4]);
                    18: osd_char=hex_ascii(contrast[3:0]);
                    20: osd_char=8'h4D;
                    21: osd_char=8'h54;
                    22: osd_char=8'h46;
                    23: osd_char=8'h35;
                    24: osd_char=8'h30;
                    26: osd_char=8'h4E;
                    27: osd_char=8'h41;
                    29: osd_char=8'h53;
                    30: osd_char=8'h46;
                    31: osd_char=8'h52;
                    33: osd_char=8'h4E;
                    34: osd_char=8'h41;
                    default: osd_char=8'h20;
                endcase
                3'd4: case(index)
                    0:osd_char="R";
                    1:osd_char="E";
                    2:osd_char="S";
                    3:osd_char=":";
                    4:osd_char="P";
                    5:osd_char="D";
                    6:osd_char="S";
                    7:osd_char=" ";
                    8:osd_char="R";
                    9:osd_char="E";
                    10:osd_char="P";
                    11:osd_char="O";
                    12:osd_char="R";
                    13:osd_char="T";
                    24:osd_char=SHOW_ROTATION_TRACE?"R":" ";
                    25:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[63:60]):" ";
                    26:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[59:56]):" ";
                    27:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[55:52]):" ";
                    28:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[51:48]):" ";
                    30:osd_char=SHOW_ROTATION_TRACE?"C":" ";
                    31:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[47:44]):" ";
                    32:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[43:40]):" ";
                    33:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[39:36]):" ";
                    34:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[35:32]):" ";
                    36:osd_char=SHOW_ROTATION_TRACE?"O":" ";
                    37:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[31:28]):" ";
                    38:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[27:24]):" ";
                    39:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[23:20]):" ";
                    40:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[19:16]):" ";
                    42:osd_char=SHOW_ROTATION_TRACE?"A":" ";
                    43:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[15:12]):" ";
                    44:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[11:8]):" ";
                    45:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[7:4]):" ";
                    46:osd_char=SHOW_ROTATION_TRACE?hex_ascii(rotation_trace_frame[3:0]):" ";
                    default:osd_char=" ";
                endcase
                3'd5: case(index)
                    0: osd_char=8'h43;
                    1: osd_char=8'h41;
                    2: osd_char=8'h4D;
                    3: osd_char=camera_fps_valid ? hex_ascii(camera_fps_bcd[7:4]) : 8'h2D;
                    4: osd_char=camera_fps_valid ? hex_ascii(camera_fps_bcd[3:0]) : 8'h2D;
                    6: osd_char=8'h46;
                    7: osd_char=8'h50;
                    8: osd_char=8'h53;
                    10: osd_char=8'h53;
                    11: osd_char=8'h49;
                    12: osd_char=8'h5A;
                    13: osd_char=8'h45;
                    14: osd_char=camera_geometry_valid ? hex_ascii(camera_width_bcd[15:12]) : 8'h2D;
                    15: osd_char=camera_geometry_valid ? hex_ascii(camera_width_bcd[11:8]) : 8'h2D;
                    16: osd_char=camera_geometry_valid ? hex_ascii(camera_width_bcd[7:4]) : 8'h2D;
                    17: osd_char=camera_geometry_valid ? hex_ascii(camera_width_bcd[3:0]) : 8'h2D;
                    18: osd_char=8'h58;
                    19: osd_char=camera_geometry_valid ? hex_ascii(camera_height_bcd[15:12]) : 8'h2D;
                    20: osd_char=camera_geometry_valid ? hex_ascii(camera_height_bcd[11:8]) : 8'h2D;
                    21: osd_char=camera_geometry_valid ? hex_ascii(camera_height_bcd[7:4]) : 8'h2D;
                    22: osd_char=camera_geometry_valid ? hex_ascii(camera_height_bcd[3:0]) : 8'h2D;
                    24: osd_char=8'h42;
                    25: osd_char=8'h41;
                    26: osd_char=8'h44;
                    27: osd_char=camera_geometry_valid ? (camera_geometry_bad ? 8'h31 : 8'h30) : 8'h2D;
                    default: osd_char=8'h20;
                endcase
                3'd6: case(index)
                    0:osd_char=(COMBINED_OSD && EXPOSURE_LOCKED_MODE)?"L":"A";1:osd_char=(COMBINED_OSD && EXPOSURE_LOCKED_MODE)?"K":"E";3:osd_char="T";
                    4:osd_char="0";5:osd_char="X";
                    6:osd_char=hex_ascii(camera_exposure_lines[15:12]);
                    7:osd_char=hex_ascii(camera_exposure_lines[11:8]);
                    8:osd_char=hex_ascii(camera_exposure_lines[7:4]);
                    9:osd_char=hex_ascii(camera_exposure_lines[3:0]);
                    11:osd_char="G";12:osd_char="0";13:osd_char="X";
                    14:osd_char=hex_ascii({2'b00,camera_gain_q4[9:8]});
                    15:osd_char=hex_ascii(camera_gain_q4[7:4]);
                    16:osd_char=hex_ascii(camera_gain_q4[3:0]);
                    18:osd_char="E";19:osd_char="R";20:osd_char="R";
                    21:osd_char=hex_ascii(camera_light_error[7:4]);
                    22:osd_char=hex_ascii(camera_light_error[3:0]);
                    24:osd_char="R";25:osd_char="E";26:osd_char="F";
                    27:osd_char="0";28:osd_char="X";
                    29:osd_char=reference_ready ? hex_ascii(reference_q4[11:8]) : "-";
                    30:osd_char=reference_ready ? hex_ascii(reference_q4[7:4]) : "-";
                    31:osd_char=reference_ready ? hex_ascii(reference_q4[3:0]) : "-";
                    33:osd_char="H";34:osd_char=highlight_risk ? "1" : "0";
                    default:osd_char=" ";
                endcase
                default: osd_char=8'h20;
            endcase
        end
    endfunction

    reg [2:0] line_no;
    reg [5:0] char_index;
    reg [2:0] glyph_col, glyph_row_index;
    reg [7:0] char_code;
    reg glyph_on, panel_on, text_row_valid;
    reg [4:0] glyph_bits;
    reg [15:0] reference_rgb;
    assign clean_rgb=de_gamma?corrected_rgb:16'd0;
    always @* begin
        line_no = 0;
        glyph_row_index = 0;
        text_row_valid = 1;
        // 2x font scale: seven explicit 14-pixel bands, no row truncation.
        if      (y_gamma >= 16  && y_gamma < 30)  begin line_no=0; glyph_row_index=(y_gamma-16)>>1;  end
        else if (y_gamma >= 44  && y_gamma < 58)  begin line_no=1; glyph_row_index=(y_gamma-44)>>1;  end
        else if (y_gamma >= 72  && y_gamma < 86)  begin line_no=2; glyph_row_index=(y_gamma-72)>>1;  end
        else if (y_gamma >= 100 && y_gamma < 114) begin line_no=3; glyph_row_index=(y_gamma-100)>>1; end
        else if (y_gamma >= 128 && y_gamma < 142) begin line_no=4; glyph_row_index=(y_gamma-128)>>1; end
        else if (y_gamma >= 156 && y_gamma < 170) begin line_no=5; glyph_row_index=(y_gamma-156)>>1; end
        else if (y_gamma >= 184 && y_gamma < 198) begin line_no=6; glyph_row_index=(y_gamma-184)>>1; end
        else text_row_valid = 0;
        char_index = x_gamma[9:4];
        glyph_col = x_gamma[3:1];
        char_code = osd_char(line_no, char_index);
        glyph_bits = glyph_row(char_code, glyph_row_index);
        panel_on = (x_gamma < 800) && (y_gamma >= 10) && (y_gamma < 208);
        glyph_on = 0;
        if (panel_on && text_row_valid && glyph_col < 5)
            glyph_on = glyph_bits[4-glyph_col];
        // Known colors bypass the camera, DDR and Gamma paths for diagnosis.
        if      (x_gamma < 240)  reference_rgb=16'hffff;
        else if (x_gamma < 480)  reference_rgb=16'hffe0;
        else if (x_gamma < 720)  reference_rgb=16'h07ff;
        else if (x_gamma < 960)  reference_rgb=16'h07e0;
        else if (x_gamma < 1200) reference_rgb=16'hf81f;
        else if (x_gamma < 1440) reference_rgb=16'hf800;
        else if (x_gamma < 1680) reference_rgb=16'h001f;
        else                 reference_rgb=16'h0000;
    end
    always @* begin
        vs_out = vs_gamma;
        hs_out = hs_gamma;
        de_out = de_gamma;
        if (!de_gamma) rgb_out = 0;
        else if (SHOW_REFERENCE && y_gamma >= 1048 && y_gamma < 1080)
            rgb_out = reference_rgb;
        else if (osd_enable && glyph_on) rgb_out = 16'hffe0;
        else if (osd_enable && panel_on) rgb_out = 16'h1082;
        else rgb_out = corrected_rgb;
        if (!de_gamma) rgb888_out = 0;
        else if (SHOW_REFERENCE && y_gamma >= 1048 && y_gamma < 1080)
            rgb888_out = {reference_rgb[15:11],reference_rgb[15:13],
                          reference_rgb[10:5],reference_rgb[10:9],
                          reference_rgb[4:0],reference_rgb[4:2]};
        else if (osd_enable && glyph_on) rgb888_out = 24'hffff00;
        else if (osd_enable && panel_on) rgb888_out = 24'h101010;
        else rgb888_out = corrected_rgb888;
    end
endmodule
