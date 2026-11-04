`timescale 1ns / 1ps

// Commands received on the board USB-UART parameter/control channel.
// Frame: A5 5A CMD LEN PAYLOAD[LEN] CHECKSUM
// CHECKSUM is the modulo-256 sum of all preceding frame bytes.
module host_command_parser #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer FRAME_TIMEOUT_MS = 500
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg image_request,image_ack_update,
    output reg [31:0] image_sequence,
    output reg [63:0] image_ack,
    output reg control_update,
    output reg [4:0] control_flags,
    output reg         stream_enable,
    output reg  [15:0] stream_period_ms,
    output reg         snapshot_request,
    output reg  [31:0] command_count,
    output reg  [31:0] command_error_count,
    output reg performance_request,
    output reg imu_query_request,preview_update,preview_enable,
    output reg[31:0]preview_session,
    output reg lens_write,lens_commit,lens_clear,
    output reg [3:0] lens_index,output reg [31:0] lens_value,lens_id,lens_crc,
    output reg geo_write,geo_commit,geo_clear,geo_slot,ui_update,output reg [3:0] geo_index,output reg [31:0] geo_value,geo_crc,output reg [7:0] ui_flags
);

    localparam [2:0] ST_HEAD_0 = 3'd0;
    localparam [2:0] ST_HEAD_1 = 3'd1;
    localparam [2:0] ST_CMD    = 3'd2;
    localparam [2:0] ST_LEN    = 3'd3;
    localparam [2:0] ST_DATA   = 3'd4;
    localparam [2:0] ST_SUM    = 3'd5;

    reg [2:0] state;
    reg [7:0] command;
    reg [7:0] payload_len;
    reg [7:0] payload_index;
    reg [7:0] checksum;
    reg [7:0] payload [0:7];
    reg [31:0] idle_count;
    localparam integer TIMEOUT_CLKS = (CLK_FREQ_HZ / 1000) * FRAME_TIMEOUT_MS;
    wire command_shape_valid =
        ((command == 8'h01 || command == 8'h12 || command==8'h21 || command==8'h60) && payload_len == 0) ||
        (command == 8'h10 && payload_len == 1) ||
        (command == 8'h11 && payload_len == 2) ||
        (command == 8'h30 && payload_len == 4) ||
        (command == 8'h31 && payload_len == 8) ||
        (command == 8'h32 && payload_len == 5 && payload[0]<=1) ||
        (command == 8'h20 && payload_len == 1 && payload[0][1:0]<=1 && payload[0][7:4]==0) ||
        (command == 8'h40 && payload_len == 5 && payload[0]<12) ||
        (command == 8'h41 && payload_len == 8) || (command == 8'h42 && payload_len == 4) ||
        (command==8'h50 && payload_len==6 && payload[0]<=1 && payload[1]<9) ||
        (command==8'h51 && payload_len==5 && payload[0]<=1) ||
        (command==8'h52 && payload_len==1 && payload[0]<=1) || (command==8'h53 && payload_len==1 && payload[0][7]==0 && payload[0][2:1]<3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            performance_request<=0;geo_write<=0;geo_commit<=0;geo_clear<=0;geo_slot<=0;geo_index<=0;geo_value<=0;geo_crc<=0;ui_update<=0;ui_flags<=0;
            lens_write<=0;lens_commit<=0;lens_clear<=0;lens_index<=0;lens_value<=0;lens_id<=0;lens_crc<=0;
            preview_update<=0;preview_enable<=0;preview_session<=0;imu_query_request<=0;control_update<=0;control_flags<=8;image_request<=0;image_ack_update<=0;image_sequence<=0;image_ack<=0;
            state               <= ST_HEAD_0;
            command             <= 8'd0;
            payload_len         <= 8'd0;
            payload_index       <= 8'd0;
            checksum            <= 8'd0;
            idle_count          <= 32'd0;
            // The USB-UART host explicitly enables periodic telemetry.
            stream_enable       <= 1'b0;
            stream_period_ms    <= 16'd20;
            snapshot_request    <= 1'b0;
            command_count       <= 32'd0;
            command_error_count <= 32'd0;
        end else begin
            performance_request<=0;snapshot_request <= 1'b0; control_update<=0;image_request<=0;image_ack_update<=0;imu_query_request<=0;preview_update<=0;
            lens_write<=0;lens_commit<=0;lens_clear<=0;geo_write<=0;geo_commit<=0;geo_clear<=0;ui_update<=0;

            if (rx_valid || state == ST_HEAD_0)
                idle_count <= 32'd0;
            else if (idle_count >= TIMEOUT_CLKS - 1) begin
                idle_count <= 32'd0;
                state <= ST_HEAD_0;
                command_error_count <= command_error_count + 1'b1;
            end else
                idle_count <= idle_count + 1'b1;

            if (rx_valid) begin
                case (state)
                    ST_HEAD_0: begin
                        if (rx_data == 8'hA5) begin
                            checksum <= 8'hA5;
                            state    <= ST_HEAD_1;
                        end
                    end

                    ST_HEAD_1: begin
                        if (rx_data == 8'h5A) begin
                            checksum <= checksum + rx_data;
                            state    <= ST_CMD;
                        end else if (rx_data == 8'hA5) begin
                            checksum <= 8'hA5;
                        end else begin
                            state <= ST_HEAD_0;
                        end
                    end

                    ST_CMD: begin
                        command  <= rx_data;
                        checksum <= checksum + rx_data;
                        state    <= ST_LEN;
                    end

                    ST_LEN: begin
                        payload_len   <= rx_data;
                        payload_index <= 8'd0;
                        checksum      <= checksum + rx_data;
                        if (rx_data > 8'd8) begin
                            state <= ST_HEAD_0;
                            command_error_count <= command_error_count + 1'b1;
                        end else if (rx_data == 8'd0) begin
                            state <= ST_SUM;
                        end else begin
                            state <= ST_DATA;
                        end
                    end

                    ST_DATA: begin
                        payload[payload_index] <= rx_data;
                        checksum               <= checksum + rx_data;
                        if (payload_index == payload_len - 1'b1) begin
                            payload_index <= 8'd0;
                            state         <= ST_SUM;
                        end else begin
                            payload_index <= payload_index + 1'b1;
                        end
                    end

                    ST_SUM: begin
                        state <= ST_HEAD_0;
                        if (rx_data == checksum && command_shape_valid) begin
                            command_count <= command_count + 1'b1;
                            case (command)
                                8'h50:begin geo_write<=1;geo_slot<=payload[0][0];geo_index<=payload[1][3:0];geo_value<={payload[5],payload[4],payload[3],payload[2]};end
                                8'h51:begin geo_commit<=1;geo_slot<=payload[0][0];geo_crc<={payload[4],payload[3],payload[2],payload[1]};end
                                8'h52:begin geo_clear<=1;geo_slot<=payload[0][0];end
                                8'h53:begin ui_update<=1;ui_flags<=payload[0];end
                                8'h40:begin lens_write<=1;lens_index<=payload[0][3:0];lens_value<={payload[4],payload[3],payload[2],payload[1]};end
                                8'h41:begin lens_commit<=1;lens_id<={payload[3],payload[2],payload[1],payload[0]};lens_crc<={payload[7],payload[6],payload[5],payload[4]};end
                                8'h42:begin lens_clear<=1;lens_id<={payload[3],payload[2],payload[1],payload[0]};end
                                8'h01: begin
                                    if (payload_len == 0)
                                        snapshot_request <= 1'b1;
                                end

                                8'h30: begin image_request<=1;image_sequence<={payload[3],payload[2],payload[1],payload[0]};end
                                8'h31: begin image_ack_update<=1;image_ack<={payload[7],payload[6],payload[5],payload[4],payload[3],payload[2],payload[1],payload[0]};end
                                8'h20: begin control_flags<=payload[0][4:0];control_update<=1;end
                                8'h21: imu_query_request<=1;
                                8'h60: performance_request<=1;
                                8'h32: begin preview_update<=1;preview_enable<=payload[0][0];preview_session<={payload[4],payload[3],payload[2],payload[1]};end

                                8'h10: begin
                                    if (payload_len == 1)
                                        stream_enable <= payload[0][0];
                                end

                                8'h11: begin
                                    if (payload_len == 2)
                                        stream_period_ms <= ({payload[1], payload[0]} < 16'd10) ? 16'd10 :
                                                            ({payload[1], payload[0]} > 16'd5000) ? 16'd5000 :
                                                            {payload[1], payload[0]};
                                end

                                8'h12: begin
                                    if (payload_len == 0)
                                        snapshot_request <= 1'b1;
                                end

                                default: begin
                                    command_error_count <= command_error_count + 1'b1;
                                end
                            endcase
                        end else begin
                            command_error_count <= command_error_count + 1'b1;
                            if (rx_data == 8'hA5) begin
                                checksum <= 8'hA5;
                                state <= ST_HEAD_1;
                            end
                        end
                    end

                    default: state <= ST_HEAD_0;
                endcase
            end
        end
    end

endmodule
