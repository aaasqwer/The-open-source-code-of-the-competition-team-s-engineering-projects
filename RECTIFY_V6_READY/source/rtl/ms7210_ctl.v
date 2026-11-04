`timescale 1ns/1ps
// MS7210 transmitter configuration. Preserve vendor register order, little-
// endian register addresses, echo probe and the pause after register 18.
// Advance only after a successful whole transaction, never on byte_over.
module ms7210_ctl #(parameter WAIT_CYCLES=3199898, POLL_CYCLES=10000000)(
 input clk,rstn,busy,transfer_error,input [7:0] data_out,input byte_over,
 output reg init_over,output [7:0] device_id,output reg iic_trig,w_r,
 output reg [15:0] addr,output reg [7:0] data_in
);
 assign device_id=8'hB2;
function [23:0] cmd_data;
input [5:0] index;
    begin
        case(index)
            6'd0     : cmd_data = {16'h1281,8'h04};
            6'd1     : cmd_data = {16'h0016,8'h04};//
            6'd2     : cmd_data = {16'h0009,8'h01};//
            6'd3     : cmd_data = {16'h0007,8'h09};//
            6'd4     : cmd_data = {16'h0008,8'hF0};//
            6'd5     : cmd_data = {16'h000A,8'hF0};//
            6'd6     : cmd_data = {16'h0006,8'h11};//
            6'd7     : cmd_data = {16'h0531,8'h84};//
            6'd8     : cmd_data = {16'h0900,8'h20};//
            6'd9     : cmd_data = {16'h0901,8'h47};//
            6'd10    : cmd_data = {16'h0904,8'h09};
            6'd11    : cmd_data = {16'h0923,8'h07};//
            6'd12    : cmd_data = {16'h0924,8'h44};//
            6'd13    : cmd_data = {16'h0925,8'h44};//
            6'd14    : cmd_data = {16'h090F,8'h80};//
            6'd15    : cmd_data = {16'h091F,8'h07};//
            6'd16    : cmd_data = {16'h0920,8'h1E};//  INT EN
            6'd17    : cmd_data = {16'h0018,8'h20};//
            6'd18    : cmd_data = {16'h05c0,8'hFE};//
            6'd19    : cmd_data = {16'h000B,8'h00};//  seting
            6'd20    : cmd_data = {16'h0507,8'h06};
            6'd21    : cmd_data = {16'h0906,8'h04};//
            6'd22    : cmd_data = {16'h0920,8'h5E};//
            6'd23    : cmd_data = {16'h0926,8'hDD};//
            6'd24    : cmd_data = {16'h0927,8'h0D};//
            6'd25    : cmd_data = {16'h0928,8'h88};//
            6'd26    : cmd_data = {16'h0929,8'h08};//
            6'd27    : cmd_data = {16'h0910,8'h01};//
            6'd28    : cmd_data = {16'h000B,8'h11};//
            6'd29    : cmd_data = {16'h050E,8'h00};//
            6'd30    : cmd_data = {16'h050A,8'h82};
            6'd31    : cmd_data = {16'h0509,8'h02};//
            6'd32    : cmd_data = {16'h050B,8'h0D};//
            6'd33    : cmd_data = {16'h050D,8'h0B};//
            6'd34    : cmd_data = {16'h050D,8'h12};//
            6'd35    : cmd_data = {16'h050D,8'h28};//
            6'd36    : cmd_data = {16'h050D,8'h08};//
            6'd37    : cmd_data = {16'h050D,8'h22};//
            6'd38    : cmd_data = {16'h050D,8'h00};//
            6'd39    : cmd_data = {16'h050D,8'h00};//
            6'd40    : cmd_data = {16'h050D,8'h00};
            6'd41    : cmd_data = {16'h050D,8'h00};//
            6'd42    : cmd_data = {16'h050D,8'h00};//
            6'd43    : cmd_data = {16'h050D,8'h00};//
            6'd44    : cmd_data = {16'h050D,8'h00};//
            6'd45    : cmd_data = {16'h050D,8'h00};//
            6'd46    : cmd_data = {16'h050D,8'h00};//
            6'd47    : cmd_data = {16'h050E,8'h40};//
            6'd48    : cmd_data = {16'h0507,8'h00};//
       endcase 
    end
endfunction
 localparam ISSUE=0,WAIT_BUSY=1,WAIT_DONE=2,SETTLE=3,POLL_DELAY=4;
 localparam PROBE_WRITE=0,PROBE_READ=1,CONFIGURE=2,STATUS_READ=3;
 reg [2:0] state;
 reg [1:0] operation;
 reg [5:0] cmd_index;
 reg [31:0] delay_count;
 wire [23:0] command=cmd_data(cmd_index);
 always @(posedge clk or negedge rstn) begin
  if(!rstn) begin
   state<=ISSUE;operation<=PROBE_WRITE;cmd_index<=0;delay_count<=0;
   init_over<=0;iic_trig<=0;w_r<=1;addr<=16'h0003;data_in<=8'h5a;
  end else begin
   iic_trig<=0;
   case(state)
    ISSUE:if(!busy) begin
     case(operation)
      PROBE_WRITE:begin w_r<=1;addr<=16'h0003;data_in<=8'h5a;end
      PROBE_READ:begin w_r<=0;addr<=16'h0003;data_in<=0;end
      CONFIGURE:begin w_r<=1;addr<=command[23:8];data_in<=command[7:0];end
      STATUS_READ:begin w_r<=0;addr<=16'h0502;data_in<=0;end
     endcase
     iic_trig<=1;state<=WAIT_BUSY;
    end
    WAIT_BUSY:if(busy) state<=WAIT_DONE;
    WAIT_DONE:if(!busy) begin
     state<=ISSUE;
     if(transfer_error) begin
      // Retry this exact command. A failed status poll invalidates READY and
      // re-runs the probe/table, permitting reconnection without FPGA reset.
      if(operation==STATUS_READ) begin
       init_over<=0;operation<=PROBE_WRITE;cmd_index<=0;
      end
     end else case(operation)
      PROBE_WRITE:operation<=PROBE_READ;
      PROBE_READ:if(data_out==8'h5a) begin operation<=CONFIGURE;cmd_index<=0;end
                 else operation<=PROBE_WRITE;
      CONFIGURE:begin
       if(cmd_index==48) begin
        init_over<=1;operation<=STATUS_READ;delay_count<=0;state<=POLL_DELAY;
       end else begin
        cmd_index<=cmd_index+1'b1;
        if(cmd_index==18) begin delay_count<=0;state<=SETTLE;end
       end
      end
      STATUS_READ:begin delay_count<=0;state<=POLL_DELAY;end
     endcase
    end
    SETTLE:if(delay_count>=WAIT_CYCLES-1) state<=ISSUE;
           else delay_count<=delay_count+1'b1;
    POLL_DELAY:if(delay_count>=POLL_CYCLES-1) state<=ISSUE;
               else delay_count<=delay_count+1'b1;
    default:begin state<=ISSUE;operation<=PROBE_WRITE;init_over<=0;end
   endcase
  end
 end
 // synthesis translate_off
 initial if(WAIT_CYCLES<1 || POLL_CYCLES<1) $fatal(1,"Invalid HDMI delays");
 // synthesis translate_on
endmodule
