`timescale 1ns/1ps
// Isolate the real bridge control/parser/mailbox. Physical UART, Ethernet PHY,
// and unrelated image engines are black-boxed; their data paths are tested
// separately by the full transport fixtures. No claim of physical link test.
module tb_boot_controls;
 reg clk=0,pix_clk=0,netclk=0,rst=0,vs=0;always #5 clk=~clk;always #7 pix_clk=~pix_clk;always #4 netclk=~netclk;
 reg[7:0]rx_data=0;reg rx_valid=0,photo_hold=0;wire gamma_bypass,hud_enable;
 return_pc_bridge #(.RESOURCE_TELEMETRY(0)) dut(.clk(clk),.rst_n(rst),.pix_clk(pix_clk),.pix_rst_n(rst),.video_vs(vs),.video_de(1'b0),
  .scan_freeze_active(1'b0),.frame_underflow(1'b0),.data_page_active(1'b0),.gamma_bypass(gamma_bypass),.hud_enable(hud_enable));
 task byte_in;input[7:0]b;begin @(negedge clk);rx_data=b;rx_valid=1;@(negedge clk);rx_valid=0;end endtask
 task control;input[7:0]value;begin byte_in(8'ha5);byte_in(8'h5a);byte_in(8'h20);byte_in(8'h01);byte_in(value);byte_in(8'h20+value);repeat(100)@(negedge clk);end endtask
 task sync;begin @(negedge pix_clk);vs=1;repeat(4)@(negedge pix_clk);vs=0;repeat(6)@(negedge pix_clk);end endtask
 initial begin
  force dut.rx_data=rx_data;force dut.rx_valid=rx_valid;force dut.eth_clk=netclk;force dut.eth_rst_n=rst;force dut.photo_freeze=photo_hold;
  repeat(8)@(negedge clk);rst=1;repeat(100)@(negedge clk);
  if(gamma_bypass!==0||dut.requested!==4'b1001||dut.network_mode!==1)$fatal(1,"Gamma not enabled by default");
  control(13);if(gamma_bypass!==0)$fatal(1,"Control applied before frame boundary");sync();if(gamma_bypass!==1)$fatal(1,"BKB bypass control lost");
  photo_hold=1;repeat(10)@(negedge pix_clk);control(9);sync();if(gamma_bypass!==1)$fatal(1,"Photo lease did not hold controls");
  photo_hold=0;repeat(10)@(negedge pix_clk);sync();if(gamma_bypass!==0)$fatal(1,"Auto Gamma could not resume");
  if(dut.command_errors!==0)$fatal(1,"Unchanged BKB control commands rejected");
  $display("PASS actual bridge/parser/CDC: Gamma auto-on at boot, original UART commands toggle bypass only at frame boundaries, photo hold respected");$finish;
 end
 initial begin #200000;$fatal(1,"boot control watchdog");end
endmodule
