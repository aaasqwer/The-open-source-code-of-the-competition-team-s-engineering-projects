`timescale 1ns/1ps
// This board uses MS7210 TX only. The old MS7200 instance had no clock and
// must never acquire the bus after TX initialization. Keep public ports.
module ms72xx_ctl #(
 parameter CLK_HZ=10000000,IIC_HZ=400000,T_WR_MS=1,
 parameter WAIT_CYCLES=3199898,POLL_CYCLES=10000000
)(input clk,rst_n,output init_over_rx,init_over,output iic_scl,inout iic_sda);
 reg [2:0] reset_pipe;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) reset_pipe<=0;else reset_pipe<={reset_pipe[1:0],1'b1};
 wire rstn=reset_pipe[2];
 wire [7:0] device_id,data_in,data_out;
 wire [15:0] addr;
 wire trigger,write_mode,busy,byte_over,error,sda_out,sda_out_en;
 assign init_over_rx=1'b1; // Legacy convention: receiver initialization bypassed.
 ms7210_ctl #(.WAIT_CYCLES(WAIT_CYCLES),.POLL_CYCLES(POLL_CYCLES)) transmitter(
  .clk(clk),.rstn(rstn),.init_over(init_over),.device_id(device_id),
  .iic_trig(trigger),.w_r(write_mode),.addr(addr),.data_in(data_in),
  .busy(busy),.data_out(data_out),.byte_over(byte_over),.transfer_error(error));
 iic_dri #(.CLK_FRE(CLK_HZ),.IIC_FREQ(IIC_HZ),.T_WR(T_WR_MS),
           .ADDR_BYTE(2),.LEN_WIDTH(3),.DATA_BYTE(1)) bus_driver(
  .clk(clk),.rstn(rstn),.pluse(trigger),.device_id(device_id),.w_r(write_mode),
  .byte_len(4'd1),.addr(addr),.data_in(data_in),.busy(busy),.byte_over(byte_over),
  .data_out(data_out),.scl(iic_scl),.sda_in(iic_sda),.sda_out(sda_out),
  .sda_out_en(sda_out_en),.transfer_error(error));
 assign iic_sda=sda_out_en?1'b0:1'bz;
endmodule
