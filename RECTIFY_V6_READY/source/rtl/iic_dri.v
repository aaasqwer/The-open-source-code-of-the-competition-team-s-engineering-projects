`timescale 1ns/1ps
// MS72xx I2C master. Address bytes are LOW FIRST, matching the vendor driver.
// SDA is open drain. SCL remains an output: clock stretching is unsupported.
// T_WR is milliseconds; busy includes STOP and this inter-transaction gap.
module iic_dri #(
 parameter CLK_FRE=50000000,IIC_FREQ=400000,T_WR=5,
 parameter ADDR_BYTE=1,LEN_WIDTH=3,DATA_BYTE=1
)(input clk,rstn,pluse,input [7:0] device_id,input w_r,
 input [LEN_WIDTH:0] byte_len,input [ADDR_BYTE*8-1:0] addr,input [7:0] data_in,
 output reg busy,byte_over,output reg [7:0] data_out,
 output reg scl,input sda_in,output reg sda_out,output sda_out_en,
 output reg transfer_error);
 localparam QUARTER=(CLK_FRE+IIC_FREQ*4-1)/(IIC_FREQ*4);
 localparam GAP_CYCLES=(64'd1*T_WR*CLK_FRE+999)/1000;
 localparam IDLE=0,START_A=1,START_B=2,SEND=3,ACK=4,RESTART=5,
            RECEIVE=6,MASTER_ACK=7,STOP=8,GAP=9;
 reg [3:0] state;
 reg [1:0] phase;
 reg [2:0] bit_index,byte_index;
 reg [31:0] divider,gap_count;
 reg trigger_d,write_mode,data_phase,read_id;
 reg [ADDR_BYTE*8-1:0] held_addr;
 reg [7:0] held_data,held_id,shift;
 reg [LEN_WIDTH:0] total,completed;
 assign sda_out_en=!sda_out;
 always @(posedge clk or negedge rstn) begin
  if(!rstn) begin
   state<=IDLE;phase<=0;bit_index<=7;byte_index<=0;divider<=0;gap_count<=0;
   trigger_d<=0;write_mode<=1;data_phase<=0;read_id<=0;
   held_addr<=0;held_data<=0;held_id<=0;shift<=0;total<=0;completed<=0;
   busy<=0;byte_over<=0;data_out<=0;scl<=1;sda_out<=1;transfer_error<=0;
  end else begin
   trigger_d<=pluse;byte_over<=0;
   if(state==IDLE) begin
    scl<=1;sda_out<=1;divider<=0;
    if(pluse && !trigger_d) begin
     held_addr<=addr;held_data<=data_in;held_id<={device_id[7:1],1'b1};
     write_mode<=w_r;total<=byte_len;completed<=0;byte_index<=0;
     // A low SDA before START is a busy/stuck bus, not an ACK. Do not let a
     // short to ground make every address/data byte appear acknowledged.
     data_phase<=0;read_id<=0;transfer_error<=(byte_len==0 || sda_in!==1'b1);
     shift<={device_id[7:1],1'b0};busy<=1;
     state<=(byte_len==0 || sda_in!==1'b1)?GAP:START_A;
     gap_count<=0;phase<=0;bit_index<=7;
    end
   end else if(state==GAP) begin
    if(gap_count>=GAP_CYCLES-1) begin busy<=0;state<=IDLE;end
    else gap_count<=gap_count+1'b1;
   end else if(divider==QUARTER-1) begin
    divider<=0;
    case(state)
     START_A:begin scl<=1;sda_out<=0;state<=START_B;end
     START_B:begin scl<=0;state<=SEND;phase<=0;end
     SEND:begin
      case(phase)
       0:begin scl<=0;sda_out<=shift[bit_index];end
       1:scl<=1;
       3:begin scl<=0;if(bit_index==0) state<=ACK;else bit_index<=bit_index-1'b1;end
      endcase
      phase<=phase+1'b1;
     end
     ACK:begin
      case(phase)
       0:begin scl<=0;sda_out<=1;end
       1:scl<=1;
       2:if(sda_in!==1'b0) transfer_error<=1;
       3:begin
        scl<=0;bit_index<=7;
        if(transfer_error) state<=STOP;
        else if(byte_index<ADDR_BYTE) begin
         shift<=held_addr[byte_index*8+:8];byte_index<=byte_index+1'b1;state<=SEND;
        end else if(!write_mode) begin
         if(read_id) begin shift<=0;state<=RECEIVE;end
         else state<=RESTART;
        end else if(!data_phase) begin data_phase<=1;shift<=held_data;state<=SEND;end
        else begin
         byte_over<=1;completed<=completed+1'b1;
         if(completed+1'b1==total) state<=STOP;else begin shift<=held_data;state<=SEND;end
        end
       end
      endcase
      phase<=phase+1'b1;
     end
     RESTART:begin
      case(phase)
       0:begin scl<=0;sda_out<=1;end
       1:scl<=1;
       2:sda_out<=0;
       3:begin scl<=0;shift<=held_id;read_id<=1;state<=SEND;end
      endcase
      phase<=phase+1'b1;
     end
     RECEIVE:begin
      case(phase)
       0:begin scl<=0;sda_out<=1;end
       1:scl<=1;
       2:shift<={shift[6:0],sda_in};
       3:begin scl<=0;if(bit_index==0) state<=MASTER_ACK;else bit_index<=bit_index-1'b1;end
      endcase
      phase<=phase+1'b1;
     end
     MASTER_ACK:begin
      case(phase)
       0:begin scl<=0;sda_out<=completed+1'b1==total;end
       1:scl<=1;
       3:begin
        scl<=0;data_out<=shift;byte_over<=1;completed<=completed+1'b1;
        if(completed+1'b1==total) state<=STOP;
        else begin bit_index<=7;shift<=0;state<=RECEIVE;end
       end
      endcase
      phase<=phase+1'b1;
     end
     STOP:begin
      case(phase)
       0:begin scl<=0;sda_out<=0;end
       1:scl<=1;
       2:sda_out<=1;
       3:begin state<=GAP;gap_count<=0;end
      endcase
      phase<=phase+1'b1;
     end
     default:begin state<=IDLE;busy<=0;scl<=1;sda_out<=1;transfer_error<=1;end
    endcase
   end else divider<=divider+1'b1;
  end
 end
 // synthesis translate_off
 initial if(QUARTER<1 || T_WR<1 || GAP_CYCLES>32'hffffffff ||
            ADDR_BYTE<1 || ADDR_BYTE>2 || DATA_BYTE!=1)
  $fatal(1,"Unsupported I2C parameter configuration");
 // synthesis translate_on
endmodule
