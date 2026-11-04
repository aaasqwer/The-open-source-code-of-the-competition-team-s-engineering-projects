`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Meyesemi
// Engineer: Nill
// 
// Create Date: 15/03/23 14:17:29
// Design Name: 
// Module Name: fram_buf
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
module fram_buf #(
    parameter                     MEM_ROW_WIDTH        = 15    ,
    parameter                     MEM_COLUMN_WIDTH     = 10    ,
    parameter                     MEM_BANK_WIDTH       = 3     ,
    parameter                     CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter                     MEM_DQ_WIDTH         = 32    ,
    parameter integer             H_NUM                = 1920,
    parameter integer             V_NUM                = 1080,
    parameter                     PIX_WIDTH            = 16,
    parameter                     ROTATION_DIRECTION   = 1,
    parameter NETWORK_PROCESSED=0,GAMMA_POLICY=3
)(
    input [31:0] performance_time_gray,output [31:0] performance_source_id,performance_render_id,performance_source_start,
    output [63:0] performance_render_origin,performance_display_origin,
    input [612:0] geometry_request,output [63:0] display_geometry,
    input freeze_display,output freeze_active,
    input net_clk,net_rst_n,net_hold,net_line_request,net_processed,
    input net_gamma_bypass,input [4:0] net_gamma_step,
    input [10:0] net_row,net_read_x,output [15:0] net_pixel,net_pixel_next,net_pixel_previous,net_pixel_previous_next,output net_pair_valid,output [63:0] net_frame_geometry,
    output net_ready,net_line_done,net_frame_model,output [31:0] net_frame_id,
    input [416:0] lens_request,output [418:0] lens_status,
    input                         vin_clk,
    input                         wr_fsync,
    input                         wr_en,
    input  [PIX_WIDTH- 1'b1 : 0]  wr_data,
    output reg                    init_done=0,
    
    input                         ddr_clk,
    input                         ddr_rstn,
    
    input                         vout_clk,
    input                         rd_fsync,
    input                         rd_en,
    input signed [15:0]           rotation_angle,
    input                         wr_line_end,
    output                        video_underflow,
    output                        video_last_underflow,
    output                        capture_overflow,
    output                        render_busy,
    output signed [15:0]          applied_angle,
    output                        camera_frame_valid,
    output                        render_frame_valid,
    output                        scan_frame_ready,
    output [31:0]                 video_debug,
    output [63:0]                 rotation_trace,
    output                        vout_valid,
    output                        vout_de,
    output [PIX_WIDTH- 1'b1 : 0]  vout_data,
    
    output [CTRL_ADDR_WIDTH-1:0]  axi_awaddr     ,
    output [3:0]                  axi_awid       ,
    output [3:0]                  axi_awlen      ,
    output [2:0]                  axi_awsize     ,
    output [1:0]                  axi_awburst    ,
    input                         axi_awready    ,
    output                        axi_awvalid    ,
                                                  
    output [MEM_DQ_WIDTH*8-1:0]   axi_wdata      ,
    output [MEM_DQ_WIDTH -1 :0]   axi_wstrb      ,
    input                         axi_wlast      ,
    output                        axi_wvalid     ,
    input                         axi_wready     ,
    input  [3 : 0]                axi_bid        ,                                      
                                                  
    output [CTRL_ADDR_WIDTH-1:0]  axi_araddr     ,
    output [3:0]                  axi_arid       ,
    output [3:0]                  axi_arlen      ,
    output [2:0]                  axi_arsize     ,
    output [1:0]                  axi_arburst    ,
    output                        axi_arvalid    ,
    input                         axi_arready    ,
                                                  
    output                        axi_rready     ,
    input  [MEM_DQ_WIDTH*8-1:0]   axi_rdata      ,
    input                         axi_rvalid     ,
    input                         axi_rlast      ,
    input  [3:0]                  axi_rid            
);

 wire [1:0] latest_source,held_source;
 wire source_valid,output_valid;
 wire scan_urgent;
 wire [2:0] latest_output,display_output;wire [31:0] latest_geometry,latest_control_sequence;wire [31:0] renderer_wstrb;
 wire signed [15:0] completed_angle;
 wire [34:0] latest_lens,display_lens;
 wire [383:0] latest_lens_params,display_lens_params;
 assign lens_status={display_lens,display_lens_params};
 wire signed [15:0] requested_angle=(ROTATION_DIRECTION<0)?-rotation_angle:rotation_angle;
 wire signed [15:0] capture_angle_cam,source_frame_angle;
 assign rotation_trace={requested_angle,source_frame_angle,completed_angle,applied_angle};
 // Freeze the compensation pose with its camera frame. A new IMU report must
 // not rotate old captured pixels using a different time reference.
 imu_angle_cdc capture_pose_cdc(.src_clk(ddr_clk),.dst_clk(vin_clk),.rst_n(ddr_rstn),
  .src_angle(requested_angle),.dst_angle(capture_angle_cam));
 assign camera_frame_valid=source_valid;
 assign render_frame_valid=output_valid;
 wire [CTRL_ADDR_WIDTH-1:0] c_awaddr,p_awaddr,p_araddr,s_araddr,n_araddr,m_araddr;
 wire [3:0] n_arlen,n_arid,m_arlen,m_arid;
 wire n_arvalid,n_arready,n_rvalid,m_arvalid,m_arready,m_rvalid;
 wire network_lease,network_processed;wire [2:0] network_bank;wire [31:0] source_serial,render_serial;
 wire [5:0] network_gamma;
 assign performance_source_id=source_serial;assign performance_render_id=render_serial;
 cdc_mailbox #(.WIDTH(6)) gamma_for_network(.src_clk(vout_clk),.src_rst_n(ddr_rstn),.dst_clk(ddr_clk),.dst_rst_n(ddr_rstn),
  .src_data({net_gamma_bypass,net_gamma_step}),.dst_data(network_gamma));
 wire reserve_net_id;
 wire [3:0] c_awlen,p_awlen,p_arlen,p_arid,s_arlen,s_arid;
 wire c_awvalid,c_awready,c_wready,p_awvalid,p_awready,p_wready;
 wire p_arvalid,p_arready,s_arvalid,s_arready,p_rvalid,s_rvalid;
 wire [MEM_DQ_WIDTH*8-1:0] c_wdata,p_wdata;
 camera_frame_writer #(.W(H_NUM),.H(V_NUM),.ADDR_W(CTRL_ADDR_WIDTH),.COLUMN_BITS(MEM_COLUMN_WIDTH),.NETWORK_LEASE(1),.POSE_SAMPLE_MID(1)) capture(
  .time_gray(performance_time_gray),.latest_start_tick(performance_source_start),
  .cam_clk(vin_clk),.rst_n(ddr_rstn),.vs(wr_fsync),.pixel_valid(wr_en),.line_end(wr_line_end),.pixel(wr_data),.frame_angle(capture_angle_cam),
  .ddr_clk(ddr_clk),.display_frame(held_source),.latest_frame(latest_source),.frame_valid(source_valid),.latest_frame_angle(source_frame_angle),
  .network_frame_hold(network_lease&&!network_processed),.network_frame(network_bank[1:0]),.frame_serial(source_serial),
  .overflow(capture_overflow),.awaddr(c_awaddr),.awid(),.awlen(c_awlen),.awvalid(c_awvalid),
  .awready(c_awready),.wdata(c_wdata),.wstrb(),.wready(c_wready));
 rect_frame_engine #(.W(H_NUM),.H(V_NUM),.A(CTRL_ADDR_WIDTH),.GAMMA_ENABLE(1),.GAMMA_POLICY(GAMMA_POLICY)) renderer(
  .gamma_request(network_gamma),
  .source_start_tick(performance_source_start),.latest_origin(performance_render_origin),
  .request_mode(geometry_request[577:576]),.request_imu(geometry_request[578]),.request_plane(geometry_request[579]),.plane_valid(geometry_request[580]),
  .control_sequence(geometry_request[612:581]),.normal_matrix(geometry_request[287:0]),.plane_matrix(geometry_request[575:288]),
  .latest_geometry(latest_geometry),.latest_control_sequence(latest_control_sequence),.source_serial(source_serial),
  .clk(ddr_clk),.rst_n(ddr_rstn),.source_valid(source_valid),.source_frame(latest_source),.angle(source_frame_angle),
  .network_frame_hold(network_lease&&network_processed),.network_frame(network_bank),.render_serial(render_serial),
  .display_bank(display_output),.held_source(held_source),.latest_bank(latest_output),.latest_angle(completed_angle),
  .lens_request(lens_request),.freeze_config(freeze_display),.latest_lens(latest_lens),.latest_lens_params(latest_lens_params),
  .frame_valid(output_valid),.busy(render_busy),.debug_position(video_debug),
  .araddr(p_araddr),.arid(p_arid),.arlen(p_arlen),.arvalid(p_arvalid),.arready(p_arready),
  .rdata(axi_rdata),.rid(axi_rid),.rvalid(p_rvalid),.rlast(axi_rlast),
  .awaddr(p_awaddr),.awid(),.awlen(p_awlen),.awvalid(p_awvalid),.awready(p_awready),
  .wdata(p_wdata),.wstrb(renderer_wstrb),.wready(p_wready));
 video_frame_scan #(.W(H_NUM),.H(V_NUM),.ADDR_W(CTRL_ADDR_WIDTH),.FIFO_BITS(11),.PREFILL_WORDS(1536)) scan(
  .latest_origin(performance_render_origin),.display_origin(performance_display_origin),
  .latest_geometry({latest_control_sequence,latest_geometry}),.display_geometry(display_geometry),
  .clk(ddr_clk),.rst_n(ddr_rstn),.pix_clk(vout_clk),.vs(rd_fsync),.de(rd_en),
  .latest_bank(latest_output),.frame_valid(output_valid),.latest_angle(completed_angle),
  .latest_lens(latest_lens),.latest_lens_params(latest_lens_params),.display_lens(display_lens),.display_lens_params(display_lens_params),
  .display_bank(display_output),.applied_angle(applied_angle),.freeze_display(freeze_display),.freeze_active(freeze_active),
  .araddr(s_araddr),.arid(s_arid),.arlen(s_arlen),.arvalid(s_arvalid),.arready(s_arready),
  .rdata(axi_rdata),.rid(axi_rid),.rvalid(s_rvalid),.rlast(axi_rlast),
  .out_de(vout_de),.out_data_valid(vout_valid),.out_pixel(vout_data),.underflow(video_underflow),.display_ready(scan_frame_ready),
  .last_frame_underflow(video_last_underflow),.read_urgent(scan_urgent));
 network_pair_reader #(.W(H_NUM),.H(V_NUM),.A(CTRL_ADDR_WIDTH),.BANK_BITS(3),.SUPPORT_GAMMA(NETWORK_PROCESSED),.GAMMA_POLICY(GAMMA_POLICY),.GAMMA_IN_FRAME(1)) raw_reader(
  .clk(ddr_clk),.rst_n(ddr_rstn),.net_clk(net_clk),.net_rst_n(net_rst_n),
  .hold_frame(net_hold),.request_toggle(net_line_request),.request_row(net_row),.read_x(net_read_x),.read_pixel(net_pixel),.read_pixel_next(net_pixel_next),.read_pixel_previous(net_pixel_previous),.read_pixel_previous_next(net_pixel_previous_next),.read_pair_valid(net_pair_valid),.frame_geometry(net_frame_geometry),.alternate_geometry({latest_control_sequence,latest_geometry}),
  .ready(net_ready),.done_toggle(net_line_done),.frame_id(net_frame_id),.gamma_bypass_frame(net_frame_model),
  .source_valid(source_valid),.latest_bank({1'b0,latest_source}),.latest_id(source_serial),.lease_active(network_lease),.leased_bank(network_bank),.lease_processed(network_processed),
  .select_alternate(net_processed),.alternate_valid(output_valid),.alternate_bank(latest_output),.alternate_id(render_serial),
  .gamma_bypass(network_gamma[5]),.gamma_step(network_gamma[4:0]),
  .araddr(n_araddr),.arid(n_arid),.arlen(n_arlen),.arvalid(n_arvalid),.arready(n_arready),.rdata(axi_rdata),.rvalid(n_rvalid),.rlast(axi_rlast));
 network_read_arbiter #(.A(CTRL_ADDR_WIDTH)) net_arbiter(.clk(ddr_clk),.rst_n(ddr_rstn),
  .p_addr(p_araddr),.p_id(p_arid),.p_len(p_arlen),.p_valid(p_arvalid),.p_ready(p_arready),
  .n_addr(n_araddr),.n_id(n_arid),.n_len(n_arlen),.n_valid(n_arvalid),.n_ready(n_arready),
  .reserve_net_id(reserve_net_id),.rid(axi_rid),
  .addr(m_araddr),.id(m_arid),.len(m_arlen),.valid(m_arvalid),.ready(m_arready),
  .rvalid(m_rvalid),.rlast(axi_rlast),.p_rvalid(p_rvalid),.n_rvalid(n_rvalid));
 video_ddr_fabric #(.A(CTRL_ADDR_WIDTH)) fabric(
  .clk(ddr_clk),.rst_n(ddr_rstn),.scan_urgent(scan_urgent),
  .c_awaddr(c_awaddr),.c_awlen(c_awlen),.c_awvalid(c_awvalid),.c_awready(c_awready),.c_wdata(c_wdata),.c_wready(c_wready),
  .p_awaddr(p_awaddr),.p_awlen(p_awlen),.p_awvalid(p_awvalid),.p_awready(p_awready),.p_wdata(p_wdata),.p_wstrb(renderer_wstrb),.p_wready(p_wready),
  .awaddr(axi_awaddr),.awlen(axi_awlen),.awvalid(axi_awvalid),.awready(axi_awready),
  .wdata(axi_wdata),.wstrb(axi_wstrb),.wready(axi_wready),
  .s_araddr(s_araddr),.s_arlen(s_arlen),.s_arid(s_arid),.s_arvalid(s_arvalid),.s_arready(s_arready),
  .p_araddr(m_araddr),.p_arlen(m_arlen),.p_arid(m_arid),.p_arvalid(m_arvalid),.p_arready(m_arready),
  .araddr(axi_araddr),.arlen(axi_arlen),.arid(axi_arid),.arvalid(axi_arvalid),.arready(axi_arready),
  .rdata(axi_rdata),.rid(axi_rid),.rvalid(axi_rvalid),.rlast(axi_rlast),.s_rvalid(s_rvalid),.p_rvalid(m_rvalid));
 assign axi_awid=0;assign axi_awsize=3'b110;assign axi_awburst=2'b01;
 assign axi_arsize=3'b110;assign axi_arburst=2'b01;assign axi_rready=1;assign axi_wvalid=axi_wready;
 always @(posedge ddr_clk or negedge ddr_rstn)
  if(!ddr_rstn) init_done<=0;else init_done<=output_valid;
endmodule
