`timescale 1ns/1ps
// Two read clients, two write clients. Display reads have command priority;
// camera writes have burst priority. Grants never change while backpressured.
module video_ddr_fabric #(parameter A=28)(input clk,rst_n,input scan_urgent,
 input [A-1:0] c_awaddr,input [3:0] c_awlen,input c_awvalid,output c_awready,
 input [255:0] c_wdata,output c_wready,
 input [A-1:0] p_awaddr,input [3:0] p_awlen,input p_awvalid,output p_awready,
 input [255:0] p_wdata,input [31:0] p_wstrb,output p_wready,
 output [A-1:0] awaddr,output [3:0] awlen,output awvalid,input awready,
 output [255:0] wdata,output [31:0] wstrb,input wready,
 input [A-1:0] s_araddr,input [3:0] s_arlen,s_arid,input s_arvalid,output s_arready,
 input [A-1:0] p_araddr,input [3:0] p_arlen,p_arid,input p_arvalid,output p_arready,
 output [A-1:0] araddr,output [3:0] arlen,arid,output arvalid,input arready,
 input [255:0] rdata,input [3:0] rid,input rvalid,rlast,
 output s_rvalid,p_rvalid
);
 reg [1:0] ws;reg write_camera;reg [3:0] beats_left;
 wire choose_camera=(ws==0)?c_awvalid:write_camera;
 wire allow_write=(ws==1) || !scan_urgent;
 assign awaddr=choose_camera?c_awaddr:p_awaddr;assign awlen=choose_camera?c_awlen:p_awlen;
 assign awvalid=(ws!=2) && allow_write && (choose_camera?c_awvalid:p_awvalid);
 assign c_awready=(ws!=2) && allow_write && choose_camera && awready;
 assign p_awready=(ws!=2) && allow_write && !choose_camera && awready;
 assign wdata=write_camera?c_wdata:p_wdata;assign wstrb=write_camera?32'hffffffff:p_wstrb;
 assign c_wready=ws==2 && write_camera && wready;assign p_wready=ws==2 && !write_camera && wready;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin ws<=0;write_camera<=0;beats_left<=0;end
  else case(ws)
   0,1:if(awvalid) begin
    write_camera<=choose_camera;
    if(awready) begin beats_left<=awlen;ws<=2;end else ws<=1;
   end
   2:if(wready) begin if(beats_left==0) ws<=0;else beats_left<=beats_left-1'b1;end
   default:ws<=0;
  endcase
 reg ar_locked,read_scan;
 wire select_scan=ar_locked?read_scan:s_arvalid;
 assign araddr=select_scan?s_araddr:p_araddr;assign arlen=select_scan?s_arlen:p_arlen;
 assign arid=select_scan?{1'b1,s_arid[2:0]}:{1'b0,p_arid[2:0]};
 assign arvalid=select_scan?s_arvalid:p_arvalid;
 assign s_arready=select_scan && arready;assign p_arready=!select_scan && arready;
 assign s_rvalid=rvalid && rid[3];assign p_rvalid=rvalid && !rid[3];
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin ar_locked<=0;read_scan<=0;end
  else if(arvalid) begin
   if(arready) ar_locked<=0;else begin ar_locked<=1;read_scan<=select_scan;end
  end
endmodule
