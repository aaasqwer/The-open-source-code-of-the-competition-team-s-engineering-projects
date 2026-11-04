`timescale 1ns/1ps
// Read-only exposure telemetry after configuration. Original implementation.
// LIGHT is a relative, exposure-normalized sensor-average index, NOT lux.
module ov5640_light_meter #(
    parameter EXPECTED_FORMAT=8'h61,
    parameter EXPECTED_ISP=8'h03,
    parameter ALLOW_LOCKED_EXPOSURE=0,
    // -1 retains the legacy policy; 0/3 require actual auto/locked readback.
    parameter integer EXPECTED_AEC_AGC=-1,
    parameter CLK_HZ=25000000,
    parameter SCCB_HZ=100000,
    parameter START_DELAY_CYCLES=CLK_HZ/500,
    parameter STALE_CYCLES=CLK_HZ/2,
    parameter REFERENCE_LINES=336,
    parameter REFERENCE_GAIN_Q4=16,
    parameter AE_TARGET=44
)(
    input clk, rstn, enable, camera_vs,
    output scl, inout sda,
    output reg valid,
    output reg [7:0] light_index, sensor_average, error_code,
    output reg [15:0] exposure_lines,
    output reg [11:0] light_metric_q4,
    output reg [9:0] gain_q4
);
    localparam WAIT_FRAME=0, DELAY=1, START_READ=2, WAIT_READ=3,
               CHECK=4, PREP_DIV=5, DIVIDE=6;
    localparam [35:0] REFERENCE_NUMERATOR=36'd128*REFERENCE_LINES*REFERENCE_GAIN_Q4*16;
    reg en_meta,en_ready,vs_meta,vs_sync,vs_d;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin en_meta<=0;en_ready<=0;vs_meta<=0;vs_sync<=0;vs_d<=0;end
        else begin en_meta<=enable;en_ready<=en_meta;vs_meta<=camera_vs;vs_sync<=vs_meta;vs_d<=vs_sync;end
    end
    wire frame_edge=vs_sync && !vs_d;
    reg [2:0] state;
    reg [31:0] delay_count,stale_count;
    reg crossed_frame;
    reg [4:0] index;
    reg [7:0] sample [0:16];
    reg read_start;
    reg [15:0] read_address;
    wire read_done,read_error,read_busy;
    wire [7:0] read_data;
    always @* begin
        case(index)
            0,11:read_address=16'h3500;
            1,12:read_address=16'h3501;
            2,13:read_address=16'h3502;
            3,14:read_address=16'h350a;
            4,15:read_address=16'h350b;
            5:read_address=16'h56a1;
            6:read_address=16'h4300;
            7:read_address=16'h501f;
            8:read_address=16'h5001;
            9:read_address=16'h300a;
            10:read_address=16'h300b;
            default:read_address=16'h3503;
        endcase
    end
    sccb_read_byte #(.CLK_HZ(CLK_HZ),.SCCB_HZ(SCCB_HZ)) reader(
        .clk(clk),.rstn(rstn),.enable(en_ready),.start(read_start),
        .address(read_address),.busy(read_busy),.done(read_done),
        .error(read_error),.data(read_data),.scl(scl),.sda(sda));

    wire [15:0] exposure_now={sample[0][3:0],sample[1],sample[2][7:4]};
    wire [9:0] gain_now={sample[3][1:0],sample[4]};
    wire coherent=sample[0][3:0]==sample[11][3:0] && sample[1]==sample[12] &&
                  sample[2][7:4]==sample[13][7:4] &&
                  sample[3][1:0]==sample[14][1:0] && sample[4]==sample[15];
    reg [25:0] exposure_product;
    reg [35:0] numerator,quotient;
    reg [31:0] divisor;
    reg [32:0] remainder;
    reg [5:0] divide_bit;
    // AE_TARGET is a constant: normal PDS synthesis folds this multiplication.
    wire [31:0] denominator=exposure_product*AE_TARGET;
    wire [32:0] trial={remainder[31:0],quotient[35]};
    wire take_bit=trial>={1'b0,divisor};
    wire [35:0] quotient_next={quotient[34:0],take_bit};
    wire [35:0] rounded_whole=(quotient_next+36'd8)>>4;
    // Error bits: 01 NACK, 02 chip ID, 04 format/manual AE, 08 unstable
    // snapshot/crossed frame, 10 invalid exposure/gain, 20 stale, 80 startup.
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            state<=WAIT_FRAME;read_start<=0;index<=0;delay_count<=0;
            stale_count<=0;crossed_frame<=0;valid<=0;error_code<=8'h80;
            light_index<=128;sensor_average<=0;exposure_lines<=0;gain_q4<=0;
            light_metric_q4<=0;
            exposure_product<=0;numerator<=0;divisor<=1;quotient<=0;
            remainder<=0;divide_bit<=0;
        end else if(!en_ready) begin
            state<=WAIT_FRAME;read_start<=0;valid<=0;error_code<=8'h80;
            stale_count<=0;crossed_frame<=0;
        end else begin
            read_start<=0;
            if(stale_count<STALE_CYCLES) stale_count<=stale_count+1'b1;
            if(stale_count==STALE_CYCLES-1) begin valid<=0;error_code<=8'h20;end
            if(frame_edge && state!=WAIT_FRAME && state!=DELAY) crossed_frame<=1;
            case(state)
                WAIT_FRAME:if(frame_edge) begin delay_count<=0;state<=DELAY;end
                DELAY:begin
                    if(frame_edge) delay_count<=0;
                    else if(delay_count>=START_DELAY_CYCLES-1) begin
                        index<=0;crossed_frame<=0;state<=START_READ;
                    end else delay_count<=delay_count+1'b1;
                end
                START_READ:if(!read_busy) begin read_start<=1;state<=WAIT_READ;end
                WAIT_READ:if(read_done) begin
                    if(read_error) begin valid<=0;error_code<=8'h01;state<=WAIT_FRAME;end
                    else begin
                        sample[index]<=read_data;
                        if(index==16) state<=CHECK;
                        else begin index<=index+1'b1;state<=START_READ;end
                    end
                end
                CHECK:begin
                    state<=WAIT_FRAME;
                    if({sample[9],sample[10]}!=16'h5640) begin valid<=0;error_code<=8'h02;end
                    else if(sample[6]!=EXPECTED_FORMAT || sample[7]!=8'h01 || sample[8]!=EXPECTED_ISP ||
                            ((EXPECTED_AEC_AGC>=0) ? sample[16][1:0]!=EXPECTED_AEC_AGC :
                             (sample[16][1:0]!=0 && !(ALLOW_LOCKED_EXPOSURE && sample[16][1:0]==3)))) begin
                        valid<=0;error_code<=8'h04;
                    end else if(!coherent || crossed_frame || frame_edge) begin valid<=0;error_code<=8'h08;end
                    else if(exposure_now==0 || gain_now<16 || sample[2][3:0]!=0) begin valid<=0;error_code<=8'h10;end
                    else begin
                        exposure_product<=exposure_now*gain_now;
                        numerator<=sample[5]*REFERENCE_NUMERATOR;
                        state<=PREP_DIV;
                    end
                end
                PREP_DIV:begin
                    divisor<=denominator;
                    // Keep four fractional bits before normalizing to a scene
                    // reference. Integer display remains round-to-nearest.
                    quotient<=numerator;
                    remainder<=0;divide_bit<=35;state<=DIVIDE;
                end
                DIVIDE:begin
                    remainder<=take_bit ? trial-{1'b0,divisor} : trial;
                    quotient<=quotient_next;
                    if(divide_bit==0) begin
                        state<=WAIT_FRAME;
                        if(crossed_frame || frame_edge) begin valid<=0;error_code<=8'h08;end
                        else begin
                            light_metric_q4<=|quotient_next[35:12] ? 12'hfff : quotient_next[11:0];
                            light_index<=|rounded_whole[35:8] ? 8'd255 : rounded_whole[7:0];
                            exposure_lines<=exposure_now;gain_q4<=gain_now;sensor_average<=sample[5];
                            valid<=1;error_code<=0;stale_count<=0;
                        end
                    end else divide_bit<=divide_bit-1'b1;
                end
                default:state<=WAIT_FRAME;
            endcase
        end
    end
    // synthesis translate_off
    initial begin
        if(REFERENCE_LINES<1 || REFERENCE_GAIN_Q4<16 ||
           REFERENCE_LINES*REFERENCE_GAIN_Q4>65535 || AE_TARGET<1 || AE_TARGET>63 ||
           START_DELAY_CYCLES<1 || STALE_CYCLES<1)
            $fatal(1,"Light meter parameters exceed bounded exposure arithmetic");
    end
    // synthesis translate_on
endmodule

// Single-byte 16-bit-register SCCB/I2C read. START, 78, addr_hi, addr_lo,
// repeated START, 79, one data byte, master NACK, STOP. Open-drain SDA.
// No clock-stretch support (same board clock-output interface as configuration).
module sccb_read_byte #(parameter CLK_HZ=25000000, parameter SCCB_HZ=100000)(
    input clk,rstn,enable,start,input [15:0] address,
    output reg busy,done,error,output reg [7:0] data,
    output reg scl,inout sda
);
    localparam QUARTER=(CLK_HZ+SCCB_HZ*4-1)/(SCCB_HZ*4);
    localparam IDLE=0,START_A=1,START_B=2,SEND=3,ACK=4,
               RESTART=5,RECEIVE=6,MASTER_NACK=7,STOP=8;
    reg [3:0] state;
    reg [1:0] phase,byte_index;
    reg [2:0] bit_index;
    reg [31:0] divider;
    reg [15:0] latched_address;
    reg [7:0] shift;
    reg release_sda;
    assign sda=release_sda ? 1'bz : 1'b0;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            state<=IDLE;scl<=1;release_sda<=1;busy<=0;done<=0;error<=0;
            data<=0;divider<=0;phase<=0;byte_index<=0;bit_index<=7;
            latched_address<=0;shift<=0;
        end else if(!enable) begin
            state<=IDLE;scl<=1;release_sda<=1;busy<=0;done<=0;divider<=0;
        end else begin
            done<=0;
            if(state==IDLE) begin
                scl<=1;release_sda<=1;divider<=0;
                if(start) begin
                    latched_address<=address;busy<=1;error<=0;data<=0;
                    state<=START_A;byte_index<=0;phase<=0;bit_index<=7;shift<=8'h78;
                end
            end else if(divider==QUARTER-1) begin
                divider<=0;
                case(state)
                    START_A:begin scl<=1;release_sda<=0;state<=START_B;end
                    START_B:begin scl<=0;state<=SEND;phase<=0;end
                    SEND:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=shift[bit_index];end
                            1:scl<=1;
                            3:begin
                                scl<=0;
                                if(bit_index==0) state<=ACK;
                                else bit_index<=bit_index-1'b1;
                            end
                        endcase
                        phase<=phase+1'b1;
                    end
                    ACK:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=1;end
                            1:scl<=1;
                            2:if(sda!==1'b0) error<=1;
                            3:begin
                                scl<=0;bit_index<=7;
                                if(error) state<=STOP;
                                else if(byte_index==2) state<=RESTART;
                                else if(byte_index==3) begin state<=RECEIVE;shift<=0;end
                                else begin
                                    byte_index<=byte_index+1'b1;state<=SEND;
                                    shift<=byte_index==0 ? latched_address[15:8] : latched_address[7:0];
                                end
                            end
                        endcase
                        phase<=phase+1'b1;
                    end
                    RESTART:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=1;end
                            1:scl<=1;
                            2:release_sda<=0;
                            3:begin scl<=0;shift<=8'h79;byte_index<=3;state<=SEND;end
                        endcase
                        phase<=phase+1'b1;
                    end
                    RECEIVE:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=1;end
                            1:scl<=1;
                            2:shift<={shift[6:0],sda};
                            3:begin
                                scl<=0;
                                if(bit_index==0) begin data<=shift;state<=MASTER_NACK;end
                                else bit_index<=bit_index-1'b1;
                            end
                        endcase
                        phase<=phase+1'b1;
                    end
                    MASTER_NACK:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=1;end
                            1:scl<=1;
                            3:begin scl<=0;state<=STOP;end
                        endcase
                        phase<=phase+1'b1;
                    end
                    STOP:begin
                        case(phase)
                            0:begin scl<=0;release_sda<=0;end
                            1:scl<=1;
                            2:release_sda<=1;
                            3:begin busy<=0;done<=1;state<=IDLE;end
                        endcase
                        phase<=phase+1'b1;
                    end
                    default:state<=IDLE;
                endcase
            end else divider<=divider+1'b1;
        end
    end
endmodule

// Held bundle with request/acknowledge; no independent sampling of wide status.
module camera_light_status_cdc #(parameter WIDTH=43)(
    input rstn,source_clk,display_clk,input [WIDTH-1:0] status_in,
    output reg [WIDTH-1:0] status_out
);
    reg [WIDTH-1:0] held,meta,data_sync;
    reg req,ack,ack_meta,ack_sync;
    reg [2:0] req_sync;
    reg [1:0] source_reset,display_reset;
    always @(posedge source_clk or negedge rstn) begin
        if(!rstn) source_reset<=0;else source_reset<={source_reset[0],1'b1};
    end
    always @(posedge display_clk or negedge rstn) begin
        if(!rstn) display_reset<=0;else display_reset<={display_reset[0],1'b1};
    end
    always @(posedge source_clk or negedge rstn) begin
        if(!rstn) begin held<=0;req<=0;ack_meta<=0;ack_sync<=0;end
        else if(!source_reset[1]) begin held<=0;req<=0;ack_meta<=0;ack_sync<=0;end
        else begin
            ack_meta<=ack;ack_sync<=ack_meta;
            if(req==ack_sync && held!=status_in) begin held<=status_in;req<=~req;end
        end
    end
    always @(posedge display_clk or negedge rstn) begin
        if(!rstn) begin meta<=0;data_sync<=0;status_out<=0;req_sync<=0;ack<=0;end
        else if(!display_reset[1]) begin meta<=0;data_sync<=0;status_out<=0;req_sync<=0;ack<=0;end
        else begin
            meta<=held;data_sync<=meta;req_sync<={req_sync[1:0],req};
            if(req_sync[2]!=ack) begin status_out<=data_sync;ack<=req_sync[2];end
        end
    end
endmodule
