`timescale 1ns/1ps
module ethernet_image_bridge #(parameter PHOTO_CHUNK_BYTES=1024,PHOTO_COALESCE_TICKS=0,PHOTO_WINDOW=1)(
 input sys_clk,sys_rst_n,input rgmii_rxc,input rgmii_rx_ctl,input [3:0] rgmii_rxd,
 output rgmii_txc,rgmii_tx_ctl,output [3:0] rgmii_txd,output reg phy_rstn,
 output eth_clk,eth_rst_n,ready,output fault,abort,
 input capture,input [31:0] capture_sequence,input ack_update,input [63:0] ack,
 input image_start,input [7:0] image_data,input image_packet_active,abort_capture,output image_busy,image_pending,input preview_tx_en,input[7:0]preview_tx_data,
 output reg [31:0] photo_packets);
 reg [21:0] reset_count;
 always @(posedge sys_clk or negedge sys_rst_n)
 if(!sys_rst_n)begin reset_count<=0;phy_rstn<=0;end
 else if(reset_count<2700000)begin reset_count<=reset_count+1'b1;phy_rstn<=0;end
 else phy_rstn<=1;
 wire sample_clk,locked,rx_valid,rx_error;
 wire [7:0] rx_data,tx_data;wire tx_en;
 eth_ref_clock rx_phase(.clkin1(rgmii_rxc),.clkout0(sample_clk),.lock(locked));
 wire async_ready=sys_rst_n&&phy_rstn&&locked;
 reg [2:0] reset_sync;
 always @(posedge eth_clk or negedge async_ready)
 if(!async_ready)reset_sync<=0;else reset_sync<={reset_sync[1:0],1'b1};
 assign eth_rst_n=reset_sync[2];
 assign ready=async_ready&&eth_rst_n;
 rgmii_interface io(.rst(!eth_rst_n),.rgmii_clk(eth_clk),.rgmii_clk_90p(1'b0),
 .mac_tx_data_valid(tx_en|preview_tx_en),.mac_tx_data(preview_tx_en?preview_tx_data:tx_data),.mac_rx_error(rx_error),.mac_rx_data_valid(rx_valid),.mac_rx_data(rx_data),
 .rgmii_rxc(sample_clk),.rgmii_rx_ctl(rgmii_rx_ctl),.rgmii_rxd(rgmii_rxd),
 .rgmii_txc(rgmii_txc),.rgmii_tx_ctl(rgmii_tx_ctl),.rgmii_txd(rgmii_txd));
 wire net_ack_update;wire [63:0] net_ack;
 ethernet_ack_rx acknowledgements(.clk(eth_clk),.rst_n(eth_rst_n),.rx_valid(rx_valid),.rx_error(rx_error),.rx_data(rx_data),
  .ack_update(net_ack_update),.ack(net_ack));
 // Same reliable PGUD V1 protocol, with fast Ethernet or legacy UART ACK.
 ethernet_image_tx #(.REQUIRE_ACK(1),.CHUNK_BYTES(PHOTO_CHUNK_BYTES),.COALESCE_TICKS(PHOTO_COALESCE_TICKS),.WINDOW(PHOTO_WINDOW)) stream(.clk(eth_clk),.rst_n(eth_rst_n&&!abort_capture),.capture(capture),.capture_sequence(capture_sequence),
 .ack_update(ack_update||net_ack_update),.ack(net_ack_update?net_ack:ack),.image_start(image_start),.image_data(image_data),.image_packet_active(image_packet_active),
 .image_busy(image_busy),.image_pending(stream_pending),.abort(abort),.fault(fault),.tx_en(tx_en),.tx_data(tx_data));
 reg tx_last,buffered;
 // Byte backpressure and outstanding photo data are different while several
 // PGPK rows share one PGUD datagram. Fence preview/new capture through ACK.
 wire stream_pending;
 assign image_pending=PHOTO_WINDOW==4?stream_pending:(buffered||image_busy);
 always @(posedge eth_clk or negedge eth_rst_n)
  if(!eth_rst_n)begin tx_last<=0;photo_packets<=0;buffered<=0;end
  else begin
   tx_last<=tx_en;
   if(capture||abort_capture||abort)buffered<=0;
   else if(tx_en&&!tx_last)buffered<=0;
   else if(image_start&&!image_busy)buffered<=1;
   if(capture)photo_packets<=0;else if(tx_en&&!tx_last)photo_packets<=photo_packets+1'b1;
  end
endmodule
