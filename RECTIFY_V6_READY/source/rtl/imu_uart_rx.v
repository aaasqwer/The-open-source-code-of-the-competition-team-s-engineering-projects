`timescale 1ns / 1ps
// 8-N-1 UART receiver for the ATK-MS901M link.
// The input is synchronized to clk before it is sampled.  A small integer
// baud divider is sufficient here: at 27 MHz and 115200 baud the error is
// below 0.2 percent.
module imu_uart_rx #(
    parameter integer CLK_HZ = 27000000,
    parameter integer BAUD   = 115200,
    parameter integer USE_EXTERNAL_DIVIDER=0
)(
    input             clk,
    input             rst_n,
    input             rx,
    input [15:0] baud_divisor,
    output reg [7:0]  data,
    output reg        valid,
    output reg        framing_error,
    output            busy
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD/2)) / BAUD;
    wire [15:0] bit_ticks=USE_EXTERNAL_DIVIDER?baud_divisor:CLKS_PER_BIT;
    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_START = 2'd1;
    localparam [1:0] S_DATA  = 2'd2;
    localparam [1:0] S_STOP  = 2'd3;

    reg       rx_meta;
    reg       rx_sync;
    reg [1:0] state;
    reg [15:0] clk_count;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;

    assign busy = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta        <= 1'b1;
            rx_sync        <= 1'b1;
            state          <= S_IDLE;
            clk_count      <= 16'd0;
            bit_count      <= 4'd0;
            shift_reg      <= 8'd0;
            data           <= 8'd0;
            valid          <= 1'b0;
            framing_error  <= 1'b0;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            valid <= 1'b0;
            framing_error <= 1'b0;

            case (state)
                S_IDLE: begin
                    clk_count <= 16'd0;
                    bit_count <= 4'd0;
                    if (!rx_sync) begin
                        state <= S_START;
                        clk_count <= (bit_ticks >> 1) - 1;
                    end
                end

                S_START: begin
                    if (clk_count == 0) begin
                        if (!rx_sync) begin
                            state <= S_DATA;
                            bit_count <= 4'd0;
                            clk_count <= bit_ticks - 1;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        clk_count <= clk_count - 1'b1;
                    end
                end

                S_DATA: begin
                    if (clk_count == 0) begin
                        shift_reg[bit_count] <= rx_sync;
                        clk_count <= bit_ticks - 1;
                        if (bit_count == 4'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_count <= bit_count + 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count - 1'b1;
                    end
                end

                S_STOP: begin
                    if (clk_count == 0) begin
                        data <= shift_reg;
                        valid <= rx_sync;
                        framing_error <= !rx_sync;
                        state <= S_IDLE;
                    end else begin
                        clk_count <= clk_count - 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
