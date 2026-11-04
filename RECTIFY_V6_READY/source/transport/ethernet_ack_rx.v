`timescale 1ns/1ps
// Fixed IPv4/IHL5 UDP PGAK, destination port 8081. Also accept the board
// subnet's directed broadcast so multi-NIC PCs need no ARP responder.
// Accept only a complete minimum-size Ethernet frame with good PHY status,
// Ethernet FCS, IPv4 checksum/length and application CRC. Session/chunk still
// must match the current stop-and-wait transmitter; UART ACK remains usable.
module ethernet_ack_rx(input clk,rst_n,rx_valid,rx_error,input [7:0] rx_data,
 output reg ack_update,output reg [63:0] ack);
 reg active,bad;reg [3:0] preamble;reg [11:0] count;
 reg [31:0] fcs,app_crc,app_expected,dest_ip;reg [47:0] dest_mac;
 reg [63:0] candidate;reg [7:0] ip_high;reg [20:0] ip_sum;
 wire [16:0] fold={1'b0,ip_sum[15:0]}+ip_sum[20:16];
 wire [16:0] folded={1'b0,fold[15:0]}+fold[16];
 function [31:0] crc_next;input [31:0] c;input [7:0] b;reg [31:0] v;integer k;
 begin v=c^b;for(k=0;k<8;k=k+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;crc_next=v;end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;bad<=0;preamble<=0;count<=0;fcs<=0;app_crc<=0;app_expected<=0;
   dest_mac<=0;dest_ip<=0;candidate<=0;ip_high<=0;ip_sum<=0;ack<=0;ack_update<=0;end
  else begin
   ack_update<=0;
   if(!rx_valid)begin
    if(active&&!bad&&!rx_error&&count==64&&fcs==32'hdebb20e3&&app_expected==~app_crc&&folded==17'h0ffff&&
     (dest_mac==48'hffffffffffff||dest_mac==48'h026760100001)&&
     (dest_ip==32'hffffffff||dest_ip==32'hc0a8010b||dest_ip==32'hc0a801ff))begin ack<=candidate;ack_update<=1;end
    active<=0;preamble<=0;
   end else if(!active)begin
    if(rx_data==8'h55)begin if(preamble<7)preamble<=preamble+1'b1;end
    else if(rx_data==8'hd5&&preamble==7)begin
     active<=1;bad<=rx_error;count<=0;fcs<=32'hffffffff;app_crc<=32'hffffffff;
     ip_sum<=0;dest_mac<=0;dest_ip<=0;candidate<=0;app_expected<=0;
    end else preamble<=0;
   end else begin
    if(count<4095)count<=count+1'b1;
    fcs<=crc_next(fcs,rx_data);if(rx_error)bad<=1;
    if(count<6)dest_mac<={dest_mac[39:0],rx_data};
    if(count>=30&&count<=33)dest_ip<={dest_ip[23:0],rx_data};
    if(count>=14&&count<=33)begin
     if(!count[0])ip_high<=rx_data;else ip_sum<=ip_sum+{ip_high,rx_data};
    end
    if(count>=42&&count<=53)app_crc<=crc_next(app_crc,rx_data);
    if(count>=46&&count<=53)candidate[(count-46)*8+:8]<=rx_data;
    if(count>=54&&count<=57)app_expected[(count-54)*8+:8]<=rx_data;
    case(count)
     12:if(rx_data!=8'h08)bad<=1;
     13,16,21,38:if(rx_data!=0)bad<=1;
     14:if(rx_data!=8'h45)bad<=1;
     17:if(rx_data!=44)bad<=1;
     20:if((rx_data&8'h3f)!=0)bad<=1;
     23:if(rx_data!=17)bad<=1;
     36:if(rx_data!=8'h1f)bad<=1;
     37:if(rx_data!=8'h91)bad<=1;
     39:if(rx_data!=24)bad<=1;
     42:if(rx_data!="P")bad<=1;
     43:if(rx_data!="G")bad<=1;
     44:if(rx_data!="A")bad<=1;
     45:if(rx_data!="K")bad<=1;
    endcase
   end
  end
 end
endmodule
