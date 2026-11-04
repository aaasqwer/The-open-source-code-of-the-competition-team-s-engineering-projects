`timescale 1ns / 1ps

// Small, vendor-independent 8-N-1 UART transmitter.
// Present tx_data together with tx_valid; data is accepted when tx_ready is 1.
module uart_tx_8n1 #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE   = 115200,
    parameter integer USE_EXTERNAL_DIVIDER=0
)(
    input  wire       clk,
    input  wire       rst_n,
    input wire [15:0] baud_divisor,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output reg        uart_tx
);

    localparam integer CLKS_PER_BIT = (CLK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;

    wire [15:0] bit_ticks=USE_EXTERNAL_DIVIDER?baud_divisor:CLKS_PER_BIT;

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA  = 2'd2;
    localparam [1:0] ST_STOP  = 2'd3;

    reg [1:0]  state;
    reg [31:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  shift_reg;

    assign tx_ready = (state == ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            clk_count <= 32'd0;
            bit_index <= 3'd0;
            shift_reg <= 8'd0;
            uart_tx   <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    uart_tx   <= 1'b1;
                    clk_count <= 32'd0;
                    bit_index <= 3'd0;
                    if (tx_valid) begin
                        shift_reg <= tx_data;
                        uart_tx   <= 1'b0;
                        state     <= ST_START;
                    end
                end

                ST_START: begin
                    uart_tx <= 1'b0;
                    if (clk_count == bit_ticks - 1) begin
                        clk_count <= 32'd0;
                        uart_tx   <= shift_reg[0];
                        state     <= ST_DATA;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                ST_DATA: begin
                    uart_tx <= shift_reg[bit_index];
                    if (clk_count == bit_ticks - 1) begin
                        clk_count <= 32'd0;
                        if (bit_index == 3'd7) begin
                            bit_index <= 3'd0;
                            uart_tx   <= 1'b1;
                            state     <= ST_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            uart_tx   <= shift_reg[bit_index + 1'b1];
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                ST_STOP: begin
                    uart_tx <= 1'b1;
                    if (clk_count == bit_ticks - 1) begin
                        clk_count <= 32'd0;
                        state     <= ST_IDLE;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
