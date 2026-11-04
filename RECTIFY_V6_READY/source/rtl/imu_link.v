`timescale 1ns/1ps
// Scan only documented MS901M baud rates. Lock after two checksum-valid
// attitude reports. Configure volatile uploads, then read gyro/accel ranges and
// algorithm selection. Never SAVE, change BAUD, or assume a sensor full scale.
module imu_link #(parameter integer CLK_HZ=27000000,EXTENDED_REPORTS=0,
 parameter integer SLOT_CYCLES=54000000, parameter integer BOOT_CYCLES=40500000)(
 input clk,rst_n,rx, attitude_valid,config_complete,
 output tx, output [7:0] rx_data, output rx_valid,
 output reg rx_seen, output reg uart_error,
 output reg [3:0] baud_index, output reg locked,
 output link_rst_n
);
 reg [31:0] age,good_age,boot;
 reg [1:0] good_count;
 reg [15:0] divisor;
 always @* case(baud_index)
  0:divisor=(CLK_HZ+57600)/115200;
  1:divisor=(CLK_HZ+4800)/9600;
  2:divisor=(CLK_HZ+28800)/57600;
  3:divisor=(CLK_HZ+19200)/38400;
  4:divisor=(CLK_HZ+9600)/19200;
  5:divisor=(CLK_HZ+115200)/230400;
  6:divisor=(CLK_HZ+128000)/256000;
  7:divisor=(CLK_HZ+230400)/460800;
  8:divisor=(CLK_HZ+460800)/921600;
  9:divisor=(CLK_HZ+2400)/4800;
  default:divisor=(CLK_HZ+1200)/2400;
 endcase
 reg restart;
 assign link_rst_n=rst_n && !restart;
 wire byte_valid,frame_error;
 imu_uart_rx #(.CLK_HZ(CLK_HZ),.USE_EXTERNAL_DIVIDER(1)) rx_core(
  .clk(clk),.rst_n(link_rst_n),.rx(rx),.baud_divisor(divisor),
  .data(rx_data),.valid(byte_valid),.framing_error(frame_error),.busy());
 assign rx_valid=byte_valid;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   baud_index<=0;age<=0;good_age<=0;boot<=0;locked<=0;restart<=1;
   good_count<=0;rx_seen<=0;uart_error<=0;
  end else begin
   restart<=0;
   if(boot<BOOT_CYCLES) begin boot<=boot+1;age<=0; end
   else begin
    // A validated report wins over an expiring timer on the same clock.
    // Give the first good report a full slot to obtain the second report.
    if(attitude_valid) begin
     good_age<=0;age<=0;
     if(good_count<2) good_count<=good_count+1'b1;
     if(good_count>=1) locked<=1;
    end else if((!locked && age>=SLOT_CYCLES-1) || (locked && good_age>=SLOT_CYCLES)) begin
     baud_index<=(baud_index==10)?0:baud_index+1'b1;
     age<=0;good_age<=0;good_count<=0;locked<=0;restart<=1;rx_seen<=0;uart_error<=0;
    end else begin
     if(age<SLOT_CYCLES) age<=age+1;
     if(good_age<SLOT_CYCLES) good_age<=good_age+1;
    end
   end
   if(byte_valid) rx_seen<=1;
   if(frame_error) uart_error<=1;
  end
 end
 // Two setting writes and four read-only queries once per candidate.
 // Rewriting RETURNRATE faster than a slow upload period can restart a sensor
 // timer indefinitely. A lost link restarts the scan and configuration pair.
 // UART at
 // 2400/4800 uses a lower upload rate to stay within the byte-rate budget.
 reg [31:0] command_age;
 reg [3:0] n;
 reg sending,commands_done;
 reg [2:0] command_index;
 wire ready;
 reg [7:0] data;
 // Manufacturer RETURNRATE: 4=50Hz,5=20Hz,6=10Hz,7=5Hz,8=2Hz.
 // Five reports total 69 bytes/cycle, 8N1. Keep slow baud rates below 75%.
 wire [7:0] rate=EXTENDED_REPORTS!=0 ? (baud_index==10?8:baud_index==9?7:baud_index==1?6:
                      (baud_index==3||baud_index==4)?5:4) : ((baud_index==10)?8'h08:((baud_index==9)?8'h07:8'h06));
 // Lower the upload rate before enabling all required report types.
 wire [7:0] id=(command_index==0)?8'h0A:((command_index==1)?8'h08:
              ((command_index==2)?8'h83:((command_index==3)?8'h84:((command_index==4)?8'h8B:8'h8C))));
 wire [7:0] value=(command_index==0)?rate:((command_index==1)?(EXTENDED_REPORTS!=0?8'h1f:8'h15):8'h00);
 always @* case(n)
  0:data=8'h55;1:data=8'hAF;2:data=id;3:data=1;4:data=value;
  default:data=8'h05+id+value;
 endcase
 uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.USE_EXTERNAL_DIVIDER(1)) tx_core(
  .clk(clk),.rst_n(link_rst_n),.baud_divisor(divisor),
  .tx_data(data),.tx_valid(sending),.tx_ready(ready),.uart_tx(tx));
 always @(posedge clk or negedge link_rst_n) begin
  if(!link_rst_n) begin command_age<=0;n<=0;sending<=0;command_index<=0;commands_done<=0;end
  else if(boot>=BOOT_CYCLES && !commands_done) begin
   if(sending) begin
    if(ready) begin
     if(n==5) begin
      n<=0;sending<=0;command_age<=0;
      if(command_index==5) commands_done<=1;
      else command_index<=command_index+1'b1;
     end
     else n<=n+1'b1;
    end
   end else if(command_age==CLK_HZ/10-1) begin
    // One 100 ms gap before each command; no periodic writes after startup.
    sending<=1;command_age<=0;
   end else command_age<=command_age+1'b1;
  end else if(commands_done) begin
   // Retry only the read queries if a reply was lost. Never repeat settings
   // on a healthy link, and never interrupt a command already being sent.
   if(locked && !config_complete) begin
    if(command_age>=CLK_HZ-1) begin command_age<=0;command_index<=2;commands_done<=0;end
    else command_age<=command_age+1'b1;
   end else command_age<=0;
  end
 end
endmodule
