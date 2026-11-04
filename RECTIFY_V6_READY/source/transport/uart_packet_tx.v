`timescale 1ns / 1ps

module uart_packet_tx(
    input             clk,
    input             rst_n,
    input             start,
    input      [7:0]  packet_type,
    input      [31:0] sequence,
    input      [15:0] payload_length,
    output reg [15:0] payload_addr,
    input      [7:0]  payload_data,

    output reg        uart_start,
    output reg [7:0]  uart_data,
    input             uart_busy,

    output reg        busy,
    output reg        done,
    output reg        payload_sent,
    output reg [7:0]  payload_sent_data,
    output reg [15:0] payload_sent_addr
);
    localparam ST_IDLE = 3'd0, ST_HEADER = 3'd1, ST_HCRC = 3'd2,
               ST_PAYLOAD = 3'd3, ST_PCRC = 3'd4;
    reg [2:0] state;
    reg [4:0] byte_index;
    reg [7:0] type_latched;
    reg [31:0] seq_latched;
    reg [15:0] len_latched;
    reg [31:0] header_crc;
    reg [31:0] header_crc_final;
    reg [31:0] payload_crc;
    reg [31:0] payload_crc_final;
    reg wait_uart;
    reg wait_seen_busy;
    reg last_byte_pending;

    function [31:0] crc32_next;
        input [31:0] crc_in;
        input [7:0] data_in;
        reg [31:0] c;
        integer n;
        begin
            c = crc_in ^ data_in;
            for (n = 0; n < 8; n = n + 1)
                c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
            crc32_next = c;
        end
    endfunction

    function [7:0] header_byte;
        input [4:0] idx;
        begin
            case (idx)
                0: header_byte = 8'h50; // P
                1: header_byte = 8'h47; // G
                2: header_byte = 8'h50; // P
                3: header_byte = 8'h4B; // K
                4: header_byte = 8'd1;
                5: header_byte = type_latched;
                6,7: header_byte = 8'd0;
                8: header_byte = seq_latched[7:0];
                9: header_byte = seq_latched[15:8];
                10: header_byte = seq_latched[23:16];
                11: header_byte = seq_latched[31:24];
                12: header_byte = len_latched[7:0];
                13: header_byte = len_latched[15:8];
                14,15: header_byte = 8'd0;
                default: header_byte = 8'd0;
            endcase
        end
    endfunction

    reg [7:0] launch_byte;
    reg launch_last;
    reg launch_payload;
    always @(*) begin
        launch_byte = 8'd0;
        launch_last = 1'b0;
        launch_payload = 1'b0;
        case (state)
            ST_HEADER: launch_byte = header_byte(byte_index);
            ST_HCRC: launch_byte = (header_crc_final >> (byte_index * 8));
            ST_PAYLOAD: begin
                launch_byte = payload_data;
                launch_payload = 1'b1;
            end
            ST_PCRC: begin
                launch_byte = (payload_crc_final >> (byte_index * 8));
                if (byte_index == 5'd3)
                    launch_last = 1'b1;
            end
            default: launch_byte = 8'd0;
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            byte_index <= 5'd0;
            payload_addr <= 16'd0;
            uart_start <= 1'b0;
            uart_data <= 8'd0;
            busy <= 1'b0;
            done <= 1'b0;
            payload_sent <= 1'b0;
            payload_sent_data <= 8'd0;
            payload_sent_addr <= 16'd0;
            wait_uart <= 1'b0;
            wait_seen_busy <= 1'b0;
            last_byte_pending <= 1'b0;
            header_crc <= 32'hFFFFFFFF;
            payload_crc <= 32'hFFFFFFFF;
            header_crc_final <= 32'd0;
            payload_crc_final <= 32'd0;
            type_latched <= 8'd0;
            seq_latched <= 32'd0;
            len_latched <= 16'd0;
        end else begin
            uart_start <= 1'b0;
            done <= 1'b0;
            payload_sent <= 1'b0;

            if (!busy && start) begin
                busy <= 1'b1;
                state <= ST_HEADER;
                byte_index <= 5'd0;
                payload_addr <= 16'd0;
                type_latched <= packet_type;
                seq_latched <= sequence;
                len_latched <= payload_length;
                header_crc <= 32'hFFFFFFFF;
                payload_crc <= 32'hFFFFFFFF;
                wait_uart <= 1'b0;
                wait_seen_busy <= 1'b0;
                last_byte_pending <= 1'b0;
            end else if (busy) begin
                if (wait_uart) begin
                    if (uart_busy)
                        wait_seen_busy <= 1'b1;
                    else if (wait_seen_busy) begin
                        wait_uart <= 1'b0;
                        wait_seen_busy <= 1'b0;
                        if (last_byte_pending) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= ST_IDLE;
                            last_byte_pending <= 1'b0;
                        end
                    end
                end else if (!uart_busy) begin
                    uart_data <= launch_byte;
                    uart_start <= 1'b1;
                    wait_uart <= 1'b1;
                    wait_seen_busy <= 1'b0;
                    if (launch_payload) begin
                        payload_sent <= 1'b1;
                        payload_sent_data <= launch_byte;
                        payload_sent_addr <= payload_addr;
                    end
                    if (launch_last)
                        last_byte_pending <= 1'b1;

                    case (state)
                        ST_HEADER: begin
                            header_crc <= crc32_next(header_crc, launch_byte);
                            if (byte_index == 5'd15) begin
                                header_crc_final <= ~crc32_next(header_crc, launch_byte);
                                byte_index <= 5'd0;
                                state <= ST_HCRC;
                            end else
                                byte_index <= byte_index + 1'b1;
                        end
                        ST_HCRC: begin
                            if (byte_index == 5'd3) begin
                                byte_index <= 5'd0;
                                payload_addr <= 16'd0;
                                if (len_latched == 16'd0) begin
                                    payload_crc_final <= 32'd0;
                                    state <= ST_PCRC;
                                end else
                                    state <= ST_PAYLOAD;
                            end else
                                byte_index <= byte_index + 1'b1;
                        end
                        ST_PAYLOAD: begin
                            payload_crc <= crc32_next(payload_crc, launch_byte);
                            if (payload_addr == len_latched - 1'b1) begin
                                payload_crc_final <= ~crc32_next(payload_crc, launch_byte);
                                byte_index <= 5'd0;
                                state <= ST_PCRC;
                            end else
                                payload_addr <= payload_addr + 1'b1;
                        end
                        ST_PCRC: begin
                            if (byte_index != 5'd3)
                                byte_index <= byte_index + 1'b1;
                        end
                        default: state <= ST_IDLE;
                    endcase
                end
            end
        end
    end
endmodule
