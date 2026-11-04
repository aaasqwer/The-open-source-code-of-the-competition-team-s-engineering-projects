`timescale 1ns/1ps
// Real bridge admission logic and named window-pending connection. PHY/UART
// leaves are isolated; exact packet/retry bytes are covered separately.
module tb_photo_fence;
 reg clk=0,rst=0;always #5 clk=~clk;
 return_pc_bridge dut(.clk(clk),.rst_n(rst),.pix_clk(clk),.pix_rst_n(rst),.video_vs(1'b0),.video_de(1'b0));
 initial begin
  force dut.preview_busy=0;force dut.transfer_active=0;force dut.packet_active=0;force dut.image_busy=0;
  force dut.new_capture=0;force dut.drain_cycles=12500;
  force dut.capture_pending=1;force dut.preview_enabled=0;
  repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  dut.image_ethernet.image_pending=1;#1;
  if(dut.image_pending!==1||dut.capture_event!==0)$fatal(1,"capture admitted before final window ACK");
  dut.image_ethernet.image_pending=0;#1;if(dut.capture_event!==1)$fatal(1,"capture not admitted after drain");
  force dut.capture_pending=0;force dut.preview_enabled=1;
  dut.image_ethernet.image_pending=1;#1;if(dut.preview_run!==0)$fatal(1,"preview collided with pending/retransmitted photo");
  dut.image_ethernet.image_pending=0;#1;if(dut.preview_run!==1)$fatal(1,"preview did not resume after final ACK");
  $display("PASS real bridge waits for outstanding photo window before preview/new capture, even when byte backpressure is low");$finish;
 end
endmodule
module ethernet_image_bridge #(parameter PHOTO_CHUNK_BYTES=1024,PHOTO_COALESCE_TICKS=0,PHOTO_WINDOW=1)(
 input sys_clk,sys_rst_n,rgmii_rxc,rgmii_rx_ctl,input[3:0]rgmii_rxd,
 output rgmii_txc,rgmii_tx_ctl,output[3:0]rgmii_txd,output phy_rstn,eth_clk,eth_rst_n,ready,fault,abort,
 input capture,input[31:0]capture_sequence,input ack_update,input[63:0]ack,input image_start,input[7:0]image_data,
 input image_packet_active,abort_capture,output image_busy,output reg image_pending=0,input preview_tx_en,input[7:0]preview_tx_data,output[31:0]photo_packets);
 assign eth_clk=sys_clk;assign eth_rst_n=sys_rst_n;assign ready=sys_rst_n;assign image_busy=0;
endmodule
