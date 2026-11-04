`timescale 1ns/1ps
// Diagnostic measurement only. It does not change capture/frame ownership.
// The geometry bundle is held until its toggle request is acknowledged.
// FPS counts camera VS edges over 30 complete HDMI frame intervals (1 s
// for this 1080p30 design); integer count resolution is one frame/window.
module video_input_monitor #(
    parameter EXPECT_WIDTH = 1920,
    parameter EXPECT_HEIGHT = 1080,
    parameter REFERENCE_FRAMES = 30
) (
    input rstn,
    input camera_clk, input camera_vs, input camera_line, input camera_valid,
    input display_clk, input display_vs,
    output reg [7:0] fps_bcd,
    output reg fps_valid,
    output reg [15:0] width_bcd, output reg [15:0] height_bcd,
    output reg geometry_valid, output reg geometry_bad
);
    reg source_reset_meta, source_ready;
    reg display_reset_meta, display_ready;
    always @(posedge camera_clk or negedge rstn) begin
        if (!rstn) begin source_reset_meta<=0; source_ready<=0; end
        else begin source_reset_meta<=1; source_ready<=source_reset_meta; end
    end
    always @(posedge display_clk or negedge rstn) begin
        if (!rstn) begin display_reset_meta<=0; display_ready<=0; end
        else begin display_reset_meta<=1; display_ready<=display_reset_meta; end
    end

    reg vs_d, line_d, frame_seen, bad_frame;
    reg [11:0] line_pixels, frame_lines, first_width;
    reg [24:0] snapshot;
    reg request_toggle;
    reg ack_meta, ack_sync;
    reg ack_toggle;
    wire source_boundary = camera_vs && !vs_d;
    always @(posedge camera_clk or negedge rstn) begin
        if (!rstn) begin
            vs_d<=0;line_d<=0;frame_seen<=0;bad_frame<=0;
            line_pixels<=0;frame_lines<=0;first_width<=0;
            snapshot<=0;request_toggle<=0;ack_meta<=0;ack_sync<=0;
        end else if (!source_ready) begin
            vs_d<=0;line_d<=0;frame_seen<=0;bad_frame<=0;
            line_pixels<=0;frame_lines<=0;first_width<=0;
            snapshot<=0;request_toggle<=0;ack_meta<=0;ack_sync<=0;
        end else begin
            vs_d<=camera_vs; line_d<=camera_line;
            ack_meta<=ack_toggle; ack_sync<=ack_meta;
            if (source_boundary) begin
                if (frame_seen && request_toggle==ack_sync) begin
                    snapshot <= {bad_frame || frame_lines!=EXPECT_HEIGHT || camera_line,
                                 first_width,frame_lines};
                    request_toggle <= ~request_toggle;
                end
                frame_seen<=1;bad_frame<=0;frame_lines<=0;
                line_pixels<=0;first_width<=0;
            end else if (frame_seen) begin
                if (camera_line && camera_valid) begin
                    if (line_pixels!=4095) line_pixels<=line_pixels+1'b1;
                    else bad_frame<=1;
                end
                if (line_d && !camera_line) begin
                    if (frame_lines==0) first_width<=line_pixels;
                    if (line_pixels!=EXPECT_WIDTH) bad_frame<=1;
                    if (frame_lines!=4095) frame_lines<=frame_lines+1'b1;
                    else bad_frame<=1;
                    line_pixels<=0;
                end
            end
        end
    end

    reg vs_meta,vs_sync,vs_last,display_vs_d;
    reg req_meta,req_sync,req_settled;
    reg [24:0] bundle_meta,bundle_sync;
    reg window_armed;
    reg [5:0] reference_count;
    reg [7:0] frame_count_bcd;
    wire camera_edge=vs_sync && !vs_last;
    wire display_edge=display_vs && !display_vs_d;
    wire window_end=display_edge && window_armed && reference_count==REFERENCE_FRAMES-1;
    function [7:0] increment_bcd;
        input [7:0] value;
        begin
            if (value==8'h99) increment_bcd=value;
            else if (value[3:0]==9) increment_bcd={value[7:4]+4'd1,4'd0};
            else increment_bcd=value+1'b1;
        end
    endfunction
    reg convert_start, geometry_busy, convert_height;
    reg [11:0] convert_input, pending_height;
    reg [15:0] pending_width_bcd;
    reg pending_bad;
    wire convert_done;
    wire [15:0] converted_bcd;
    bin12_bcd converter(.clk(display_clk),.rstn(rstn && display_ready),
        .start(convert_start),.value(convert_input),.bcd(converted_bcd),.done(convert_done));
    always @(posedge display_clk or negedge rstn) begin
        if (!rstn) begin
            vs_meta<=0;vs_sync<=0;vs_last<=0;display_vs_d<=0;
            req_meta<=0;req_sync<=0;req_settled<=0;
            bundle_meta<=0;bundle_sync<=0;ack_toggle<=0;
            window_armed<=0;reference_count<=0;frame_count_bcd<=0;
            fps_bcd<=0;fps_valid<=0;width_bcd<=0;height_bcd<=0;
            geometry_valid<=0;geometry_bad<=0;
            convert_start<=0;geometry_busy<=0;convert_height<=0;
            convert_input<=0;pending_height<=0;pending_bad<=0;pending_width_bcd<=0;
        end else if (!display_ready) begin
            vs_meta<=0;vs_sync<=0;vs_last<=0;display_vs_d<=0;
            req_meta<=0;req_sync<=0;req_settled<=0;
            bundle_meta<=0;bundle_sync<=0;ack_toggle<=0;
            window_armed<=0;reference_count<=0;frame_count_bcd<=0;
            fps_bcd<=0;fps_valid<=0;width_bcd<=0;height_bcd<=0;
            geometry_valid<=0;geometry_bad<=0;
            convert_start<=0;geometry_busy<=0;convert_height<=0;
            convert_input<=0;pending_height<=0;pending_bad<=0;pending_width_bcd<=0;
        end else begin
            vs_meta<=camera_vs;vs_sync<=vs_meta;vs_last<=vs_sync;
            display_vs_d<=display_vs;
            req_meta<=request_toggle;req_sync<=req_meta;req_settled<=req_sync;
            bundle_meta<=snapshot;bundle_sync<=bundle_meta;
            if (display_edge) begin
                if (!window_armed) begin
                    window_armed<=1;reference_count<=0;frame_count_bcd<=0;
                end else if (window_end) begin
                    fps_bcd<=camera_edge ? increment_bcd(frame_count_bcd) : frame_count_bcd;
                    fps_valid<=1;reference_count<=0;frame_count_bcd<=0;
                end else reference_count<=reference_count+1'b1;
            end
            if (window_armed && camera_edge && !window_end)
                frame_count_bcd<=increment_bcd(frame_count_bcd);
            convert_start<=0;
            if (req_settled!=ack_toggle && !geometry_busy) begin
                convert_input<=bundle_sync[23:12];
                pending_height<=bundle_sync[11:0];pending_bad<=bundle_sync[24];
                ack_toggle<=req_settled;
                convert_start<=1;convert_height<=0;geometry_busy<=1;
            end
            if (convert_done && geometry_busy) begin
                if (!convert_height) begin
                    pending_width_bcd<=converted_bcd;convert_height<=1;
                    convert_input<=pending_height;convert_start<=1;
                end else begin
                    width_bcd<=pending_width_bcd;height_bcd<=converted_bcd;geometry_bad<=pending_bad;
                    geometry_valid<=1;geometry_busy<=0;
                end
            end
        end
    end
endmodule

// Twelve shift/add-3 steps per conversion; no divider, ROM or DSP.
module bin12_bcd(input clk,input rstn,input start,input [11:0] value,
    output reg [15:0] bcd,output reg done);
    reg [27:0] shift;
    reg [3:0] bit_count;
    reg busy;
    reg [27:0] adjusted,next_shift;
    integer digit;
    always @* begin
        adjusted=shift;
        for (digit=0;digit<4;digit=digit+1)
            if (shift[12+digit*4 +:4]>=5)
                adjusted[12+digit*4 +:4]=shift[12+digit*4 +:4]+4'd3;
        next_shift={adjusted[26:0],1'b0};
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin shift<=0;bit_count<=0;busy<=0;bcd<=0;done<=0;end
        else begin
            done<=0;
            if (start && !busy) begin shift<={16'd0,value};bit_count<=0;busy<=1;end
            else if (busy) begin
                shift<=next_shift;
                if (bit_count==11) begin bcd<=next_shift[27:12];done<=1;busy<=0;end
                else bit_count<=bit_count+1'b1;
            end
        end
    end
endmodule
