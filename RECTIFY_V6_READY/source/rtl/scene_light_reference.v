`timescale 1ns/1ps
// Relative scene reference, not a lux calibration. Establish once after a
// stable, sufficiently exposed scene, then hold across subsequent light changes.
module scene_light_reference #(
    parameter STABLE_FRAMES=30
)(
    input clk,rstn,sample_pulse,
    input telemetry_valid,
    input [11:0] metric_q4,
    input [7:0] raw_light,image_mean,image_contrast,
    input [9:0] gain_q4,
    output reg reference_ready,
    output reg [11:0] reference_q4,
    output reg result_valid,
    output reg [7:0] result_luma,
    output reg result_from_ae
);
    reg [15:0] stable_count;
    reg [11:0] candidate;
    reg busy;
    reg [19:0] remainder,denominator;
    reg [7:0] quotient;
    reg [2:0] bit_index;
    wire qualified=telemetry_valid && metric_q4>=16 && metric_q4<4080 && image_mean>=48 &&
                   image_mean<=224 && image_contrast>=80 && gain_q4>=16 && gain_q4<=64;
    wire [12:0] difference=metric_q4>candidate ? {1'b0,metric_q4}-{1'b0,candidate} :
                                                               {1'b0,candidate}-{1'b0,metric_q4};
    wire consistent=difference<=({1'b0,candidate}>>2)+13'd2;
    wire [13:0] average_sum=({2'd0,candidate}<<2)-candidate+metric_q4+14'd2;
    wire [11:0] candidate_next=average_sum[13:2];
    wire [19:0] numerator=({8'd0,metric_q4}<<7)+({8'd0,reference_q4}>>1);
    wire [19:0] threshold=denominator<<bit_index;
    wire take_bit=remainder>=threshold;
    wire [7:0] quotient_next=take_bit ? quotient|(8'd1<<bit_index) : quotient;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            reference_ready<=0;reference_q4<=0;candidate<=0;stable_count<=0;
            busy<=0;remainder<=0;denominator<=1;quotient<=0;bit_index<=0;
            result_valid<=0;result_luma<=128;result_from_ae<=0;
        end else begin
            result_valid<=0;
            if(!busy && sample_pulse) begin
                if(!reference_ready) begin
                    if(!qualified) stable_count<=0;
                    else if(stable_count==0 || !consistent) begin
                        candidate<=metric_q4;stable_count<=1;
                        if(STABLE_FRAMES==1) begin reference_ready<=1;reference_q4<=metric_q4;end
                    end else begin
                        candidate<=candidate_next;
                        if(stable_count>=STABLE_FRAMES-1) begin
                            reference_ready<=1;reference_q4<=candidate_next;
                        end else stable_count<=stable_count+1'b1;
                    end
                end
                result_from_ae<=telemetry_valid;
                if(telemetry_valid && reference_ready) begin
                    if(numerator>=({8'd0,reference_q4}<<8)) begin
                        result_luma<=255;result_valid<=1;
                    end else begin
                        denominator<={8'd0,reference_q4};remainder<=numerator;
                        quotient<=0;bit_index<=7;busy<=1;
                    end
                end else begin
                    result_luma<=telemetry_valid ? raw_light : image_mean;
                    // On the establishing sample, its own reference is neutral.
                    if(!reference_ready && qualified &&
                       ((STABLE_FRAMES==1) || (stable_count>=STABLE_FRAMES-1 && consistent)))
                        result_luma<=128;
                    result_valid<=1;
                end
            end else if(busy) begin
                if(take_bit) remainder<=remainder-threshold;
                quotient<=quotient_next;
                if(bit_index==0) begin
                    result_luma<=quotient_next;result_valid<=1;busy<=0;
                end else bit_index<=bit_index-1'b1;
            end
        end
    end
    // synthesis translate_off
    initial if(STABLE_FRAMES<1 || STABLE_FRAMES>65535) $fatal(1,"Invalid reference settling frame count");
    // synthesis translate_on
endmodule
