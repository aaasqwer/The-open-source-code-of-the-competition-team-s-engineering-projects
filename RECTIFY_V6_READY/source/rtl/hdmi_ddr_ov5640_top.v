`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// 
// Create Date: 2023-03-17  
// Design Name:  
// Module Name: 
// Project Name: 
// Target Devices: Pango
// Tool Versions: 
// Description: 
//      
// Dependencies: 
// 
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
//cmos1、cmos2二选一，作为视频源输入
//`define CMOS_1      //cmos1作为视频输入；
`ifndef CMOS_1
`define CMOS_2      //cmos2作为视频输入；
`endif

module hdmi_ddr_ov5640_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8      ,
    // Legacy Euler mode only: 0=Roll, 1=Pitch, 2=Yaw, 3=auto-axis.
    parameter integer IMU_ROT_AXIS = 0,
    // V26 follows the CURRENT 2026-10-27 STABLE13 reference: Roll/raw,
    // first valid reference, +1 direction and its scan/fabric service policy.
    parameter integer IMU_IMAGE_DIRECTION = 1,
    // -1 preserves legacy Euler-axis tracking; 0/1/2 project gravity about
    // the camera's X/Y/Z optical axis. Use only after physical mapping checks.
    parameter integer IMU_CAMERA_AXIS = -1,
    parameter integer IMU_CAMERA_FORWARD_SIGN = 1,
    // Saved rigid-installation calibration from the same-download three-pose set.
    parameter integer MOUNT_CALIBRATION_ENABLE = 1,
    parameter integer MOUNT_PLANE_X_X = -60102,
    parameter integer MOUNT_PLANE_X_Y = -20060,
    parameter integer MOUNT_PLANE_X_Z = 16742,
    parameter integer MOUNT_PLANE_Y_X = -11857,
    parameter integer MOUNT_PLANE_Y_Y = 58359,
    parameter integer MOUNT_PLANE_Y_Z = 27361,

    // 0=raw, 1=STABLE13, 2=adaptive quiet/motion smoothing.
    parameter integer IMU_SMOOTHING_ENABLE = 0,
    // 0=OPT12 sensor setup, 1=HDMI_GAMMA_1 sensor setup (including color/ISP).
    parameter integer CAMERA_PROFILE = 0,
    parameter integer CAMERA_LOCK_EXPOSURE = 0,
    parameter SENSOR_TEST_PATTERN=0,
    parameter integer SENSOR_XCLK_HZ=24000000,
    parameter integer HDMI_LED_MODE=0,
    parameter SHOW_REFERENCE=0,
    parameter GAMMA_POLICY = 3,
    parameter BRIGHTNESS_GAMMA_BYPASS = 1'b0
)(
	input                                sys_clk              ,//27MHz board reference
    input key0_n,key2_n,key3_n,
    input key4_n, // KEY4 B20: hold selected-plane rectification; release ordinary view.
    input key1_n, // KEY1, C23: release after a debounced press toggles camera text.
    input uart_rx, output uart_tx,
    input rgmii_rxc,rgmii_rx_ctl,input [3:0] rgmii_rxd,
    output rgmii_txc,rgmii_tx_ctl,output [3:0] rgmii_txd,
    input                                clk_p ,
    input                                clk_n ,
//ATK-MS901M UART (FMC-LPC-IO J3 pin 23/24 LA22_N/P; PG2L100H-676 supported)
    input                                imu_uart_rx         ,
    output                               imu_uart_tx         ,
//OV5640
    output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
    input                                cmos1_vsync          ,//cmos1 vsync
    input                                cmos1_href           ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data           ,//cmos1 data
    output                               cmos1_reset          ,//cmos1 reset
    //coms2
    inout                                cmos2_scl            ,//cmos2 i2c 
    inout                                cmos2_sda            ,//cmos2 i2c 
    input                                cmos2_vsync          ,//cmos2 vsync
    input                                cmos2_href           ,//cmos2 hsync refrence,data valid
    input                                cmos2_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos2_data           ,//cmos2 data
    output                               cmos2_reset          ,//cmos2 reset
//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成
//HDMI_OUT
    output                               pix_clk                   ,//pixclk                           
    output                               vs_out                    ,
    output                               hs_out                    ,
    output                               de_out                    ,
    output        [7:0]                  r_out                     ,
    output        [7:0]                  g_out                     ,
    output        [7:0]                  b_out
);
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    // DDR3 core_clk is 125 MHz in the PG2L100H DDR3 IP.
    parameter TH_1S = 27'd125000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;//cmos i2c clock
    wire                        cmos_sda            ;//cmos i2c data
    wire                        cmos_vsync          ;//cmos vsync
    wire                        cmos_href           ;//cmos hsync refrence,data valid
    wire                        cmos_pclk           ;//cmos pxiel clock
    wire   [7:0]                cmos_data           ;//cmos data
    wire                        cmos_reset          ;//cmos reset
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_pclk_16bit    ;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        video_clk;
    wire                        cfg_clk;
    wire                        clk_25M;
    wire                        locked;
    wire                        init_over_rx;
    wire                        core_clk;
    wire                        pll_lock;
    wire                        phy_pll_lock;
    wire                        gpll_lock;
    wire                        rst_gpll_lock;
    wire                        ddrphy_cpd_lock;
    wire                        vs_o;
    wire                        hs_o;
    wire                        de_o;
    wire                        init_done;
    wire[15:0]                  o_rgb565            ;
    wire                        pclk_in_test        ;    
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565   /*synthesis PAP_MARK_DEBUG="1"*/         ;
    wire                        de_re               ;
    wire                        imu_rx_data_valid    ;
    wire [7:0]                  imu_rx_data          ;
    wire                        imu_attitude_valid  ;
    wire signed [15:0]          imu_roll_raw        ;
    wire signed [15:0]          imu_pitch_raw       ;
    wire signed [15:0]          imu_yaw_raw         ;
    wire                        imu_motion_valid    ;
    wire signed [15:0]          imu_ax_raw          ;
    wire signed [15:0]          imu_ay_raw          ;
    wire signed [15:0]          imu_az_raw          ;
    wire signed [15:0]          imu_gx_raw          ;
    wire signed [15:0]          imu_gy_raw          ;
    wire signed [15:0]          imu_gz_raw          ;
    wire                        imu_altitude_valid  ;
    wire signed [31:0]          imu_pressure_raw    ;
    wire signed [31:0]          imu_altitude_cm     ;
    wire signed [15:0]          imu_temperature_raw ;
    wire                        imu_frame_error     ;
    wire [1:0]                  imu_rotation_mode   ;
    wire                        imu_alive           ;
    wire                        imu_altitude_seen   ;
    wire [11:0]                 sync_x_act;
    wire [11:0]                 sync_y_act;
    wire [15:0]                 hud_rgb565;
    wire [23:0] hud_rgb888,gamma_rgb888;
    wire [63:0] rotation_trace_ddr,rotation_trace_pix;
    wire frame_pixel_valid,cmos1_monitor_line,cmos2_monitor_line;
    wire gamma_de,gamma_vs,gamma_hs;
    wire [11:0] gamma_x,gamma_y;
    wire [54:0] light_status1,light_status2,selected_light_status;
    wire camera_light_valid;
    wire [7:0] camera_light_index,camera_light_error;
    wire [15:0] camera_exposure_lines;
    wire [9:0] camera_gain_q4;
    wire [11:0] camera_light_metric_q4;
`ifdef CMOS_1
    localparam METER_1=1,METER_2=0;
    wire camera_configured=cmos_init_done[0];
    wire camera_line=cmos1_monitor_line;
    assign selected_light_status=light_status1;
`else
    localparam METER_1=0,METER_2=1;
    wire camera_configured=cmos_init_done[1];
    wire camera_line=cmos2_monitor_line;
    assign selected_light_status=light_status2;
`endif
    camera_light_status_cdc #(.WIDTH(55)) light_status_crossing(
      .rstn(rstn_out),.source_clk(clk_25M),.display_clk(video_clk),
      .status_in(selected_light_status),
      .status_out({camera_light_metric_q4,camera_light_valid,camera_light_index,
                   camera_light_error,camera_exposure_lines,camera_gain_q4}));
    wire [7:0] camera_fps_bcd;
    wire [15:0] camera_width_bcd,camera_height_bcd;
    wire camera_fps_valid,camera_geometry_valid,camera_geometry_bad;
    video_input_monitor input_monitor(
      .rstn(rstn_out && camera_configured),.camera_clk(pclk_in_test),
      .camera_vs(vs_in_test),.camera_line(camera_line),.camera_valid(de_in_test),
      .display_clk(video_clk),.display_vs(vs_o),.fps_bcd(camera_fps_bcd),
      .fps_valid(camera_fps_valid),.width_bcd(camera_width_bcd),.height_bcd(camera_height_bcd),
      .geometry_valid(camera_geometry_valid),.geometry_bad(camera_geometry_bad));
    reg [11:0] sync_x_d1,sync_x_d2,sync_y_d1,sync_y_d2;
    reg sync_vs_d1,sync_vs_d2,sync_hs_d1,sync_hs_d2,sync_de_d1,sync_de_d2;
    reg [1:0] video_reset_pipe;
    wire video_timing_rstn=video_reset_pipe[1];
    // DDR readiness controls image availability, not the HDMI timing source.
    always @(posedge video_clk or negedge rstn_out)
      if(!rstn_out) video_reset_pipe<=0;
      else video_reset_pipe<={video_reset_pipe[0],1'b1};
    wire hud_de,hud_vs,hud_hs;
    wire [1:0] imu_gyro_range,imu_acc_range;
    wire imu_gyro_known,imu_acc_known,imu_algorithm_known,imu_algorithm_mag;
    wire imu_mounting_known,imu_mounting_vertical;
    wire [639:0] telemetry_sys,telemetry_pix;
    wire [47:0] pose_angles_pix;
    wire pose_page_active;
    wire pc_freeze_request,pc_frozen_core,pc_bypass,pc_hud;
    wire [416:0] lens_request_sys,lens_request_core;
    wire [418:0] lens_status_core,lens_status_sys;
    wire [34:0] lens_state_pix;wire [418:0] lens_status_pix;
    cdc_mailbox #(.WIDTH(419)) full_lens_pix(.src_clk(core_clk),.src_rst_n(rstn_out),.dst_clk(video_clk),.dst_rst_n(rstn_out),.src_data(lens_status_core),.dst_data(lens_status_pix));
    reg [1:0] scan_ready_pix;
    always @(posedge video_clk or negedge video_timing_rstn)
     if(!video_timing_rstn)scan_ready_pix<=0;else scan_ready_pix<={scan_ready_pix[0],scan_frame_ready};
    cdc_mailbox #(.WIDTH(35)) lens_hud_cdc(.src_clk(core_clk),.src_rst_n(rstn_out),.dst_clk(video_clk),.dst_rst_n(rstn_out),.src_data(lens_status_core[418:384]),.dst_data(lens_state_pix));
    cdc_mailbox #(.WIDTH(417)) lens_req_cdc(.src_clk(sys_clk),.src_rst_n(rstn_out),.dst_clk(core_clk),.dst_rst_n(rstn_out),.src_data(lens_request_sys),.dst_data(lens_request_core));
    cdc_mailbox #(.WIDTH(419)) lens_state_cdc(.src_clk(core_clk),.src_rst_n(rstn_out),.dst_clk(sys_clk),.dst_rst_n(rstn_out),.src_data(lens_status_core),.dst_data(lens_status_sys));
    reg [1:0] pc_freeze_core,pc_frozen_pix;
    wire [4:0] pc_tone_step;
    wire net_clk,net_rst_n,net_hold,net_line_request,net_ready,net_line_done,net_processed,net_frame_model;
    wire [6:0] resource_addr;wire [7:0] resource_byte;
    wire [10:0] net_row,net_read_x;wire [15:0] net_pixel,net_pixel_next,net_pixel_previous,net_pixel_previous_next,clean_gamma_rgb;wire net_pair_valid;wire [63:0] net_frame_geometry;
    wire [31:0] net_frame_id;
    always @(posedge core_clk or negedge rstn_out)
      if(!rstn_out)pc_freeze_core<=0;else pc_freeze_core<={pc_freeze_core[0],pc_freeze_request};
    always @(posedge video_clk or negedge video_timing_rstn)
      if(!video_timing_rstn)pc_frozen_pix<=0;else pc_frozen_pix<={pc_frozen_pix[0],pc_frozen_core};
    wire camera_text_visible,key_text_visible;
    wire [1:0] ui_mode;wire [7:0] ui_flags,remote_ui_flags;wire remote_ui_update,ui_imu,ui_plane,ui_page,ui_caption,ui_numbers,ui_eval,ui_edit,ui_horizontal;
    wire [2:0] ui_position;wire [10:0] ui_cx,ui_cy;wire [31:0] ui_sequence,geometry_version;
    wire [287:0] normal_matrix_sys,plane_matrix_sys;wire plane_valid_sys;
    wire [612:0] geometry_sys={ui_sequence+geometry_version,plane_valid_sys,ui_plane,ui_imu,ui_mode,plane_matrix_sys,normal_matrix_sys};
    wire [612:0] geometry_core;wire [63:0] display_geometry_core,display_geometry_sys,display_geometry_pix;
    cdc_mailbox #(.WIDTH(613)) geometry_cdc(.src_clk(sys_clk),.src_rst_n(rstn_out),.dst_clk(core_clk),.dst_rst_n(rstn_out),.src_data(geometry_sys),.dst_data(geometry_core));
    imu_snapshot_cdc #(.W(64)) display_geo_sys(.src_clk(core_clk),.dst_clk(sys_clk),.rst_n(rstn_out),.src_data(display_geometry_core),.dst_data(display_geometry_sys));
    imu_snapshot_cdc #(.W(64)) display_geo_pix(.src_clk(core_clk),.dst_clk(video_clk),.rst_n(rstn_out),.src_data(display_geometry_core),.dst_data(display_geometry_pix));
    rect_board_ui user_interface(.clk(sys_clk),.rst_n(rstn_out),.buttons_n({key4_n,key3_n,key2_n,key1_n,key0_n}),.remote_update(remote_ui_update),.remote_flags(remote_ui_flags),
     .mode(ui_mode),.ui_flags(ui_flags),.imu_enabled(ui_imu),.plane_held(ui_plane),.imu_page(ui_page),.caption_visible(ui_caption),.numbers_visible(ui_numbers),
     .evaluation_enabled(ui_eval),.editing(ui_edit),.horizontal(ui_horizontal),.position(ui_position),.center_x(ui_cx),.center_y(ui_cy),.sequence_id(ui_sequence));
    wire [32:0] ui_osd_sys={plane_valid_sys,ui_plane,ui_page,ui_numbers,ui_caption,ui_horizontal,ui_edit,ui_eval,ui_position,ui_cx,ui_cy};
    wire [32:0] ui_osd_crossed;reg [32:0] ui_osd_pix;reg ui_last_vs;
    cdc_mailbox #(.WIDTH(33)) ui_osd_cdc(.src_clk(sys_clk),.src_rst_n(rstn_out),.dst_clk(video_clk),.dst_rst_n(rstn_out),.src_data(ui_osd_sys),.dst_data(ui_osd_crossed));
    always @(posedge video_clk or negedge video_timing_rstn)
     if(!video_timing_rstn)begin ui_osd_pix<=0;ui_last_vs<=0;end
     else begin ui_last_vs<=gamma_vs;if(gamma_vs&&!ui_last_vs)ui_osd_pix<=ui_osd_crossed;end
    wire pv_pix,ph_pix,page_pix,numbers_pix,caption_pix,horizontal_pix,editing_pix,evaluation_pix;
    wire [2:0] position_pix;wire [10:0] center_x_pix,center_y_pix;
    assign {pv_pix,ph_pix,page_pix,numbers_pix,caption_pix,horizontal_pix,editing_pix,evaluation_pix,position_pix,center_x_pix,center_y_pix}=ui_osd_pix;
    assign key_text_visible=caption_pix;
    assign camera_text_visible=pc_hud&&key_text_visible&&!pose_page_active;
    return_pc_bridge #(.PROCESSED_PREVIEW(1),.RESOURCE_TELEMETRY(0),.EXTENDED_IMU(1)) u_pc(.performance_probe(performance_host),.clk(sys_clk),.rst_n(rstn_out),.uart_rx(uart_rx),.uart_tx(uart_tx),
      .extra_ready(imu_attitude_ready),.quaternion_valid(imu_quaternion_valid),.magnetic_valid(imu_magnetic_valid),.camera_text_visible(camera_text_visible),
      .q0(imu_q0_raw),.q1(imu_q1_raw),.q2(imu_q2_raw),.q3(imu_q3_raw),.mx(imu_mx_raw),.my(imu_my_raw),.mz(imu_mz_raw),.mag_temperature(imu_mag_temperature_raw),
      .lens_request(lens_request_sys),.lens_status(lens_status_sys),
      .board_ui_flags(ui_flags),.display_geometry(display_geometry_sys),.remote_ui_update(remote_ui_update),.remote_ui_flags(remote_ui_flags),
      .normal_matrix(normal_matrix_sys),.plane_matrix(plane_matrix_sys),.plane_valid(plane_valid_sys),.geometry_version(geometry_version),
      .pix_clk(video_clk),.pix_rst_n(video_timing_rstn),.video_vs(gamma_vs),.video_de(gamma_de),.pixel(clean_gamma_rgb),
      .net_clk(net_clk),.net_rst_n(net_rst_n),.net_hold(net_hold),.net_line_request(net_line_request),.net_processed(net_processed),
      .net_frame_model(net_frame_model),.resource_addr(resource_addr),.resource_byte(resource_byte),
      .net_row(net_row),.net_read_x(net_read_x),.net_pixel(net_pixel),.net_pixel_next(net_pixel_next),.net_pixel_previous(net_pixel_previous),.net_pixel_previous_next(net_pixel_previous_next),.net_pair_valid(net_pair_valid),.net_frame_geometry(net_frame_geometry),.net_ready(net_ready),.net_line_done(net_line_done),.net_frame_id(net_frame_id),
      .rgmii_rxc(rgmii_rxc),.rgmii_rx_ctl(rgmii_rx_ctl),.rgmii_rxd(rgmii_rxd),.rgmii_txc(rgmii_txc),.rgmii_tx_ctl(rgmii_tx_ctl),.rgmii_txd(rgmii_txd),
      .telemetry(telemetry_sys),.gyro_range(imu_gyro_range),.acc_range(imu_acc_range),
      .roll(imu_roll_raw),.pitch(imu_pitch_raw),.yaw(imu_yaw_raw),.ax(imu_ax_raw),.ay(imu_ay_raw),.az(imu_az_raw),.gx(imu_gx_raw),.gy(imu_gy_raw),.gz(imu_gz_raw),
      .temperature(imu_temperature_raw),.pressure(imu_pressure_raw),.altitude(imu_altitude_cm),
      .attitude_valid(imu_attitude_valid),.motion_valid(imu_motion_valid),.altitude_valid(imu_altitude_valid),.imu_error(imu_frame_error||imu_uart_error),
      .camera_ready(camera_configured),.ddr_ready(ddr_init_done),.render_ready(render_frame_valid),
      .camera_fps_bcd(camera_fps_bcd),.camera_fps_valid(camera_fps_valid),.gamma_step(pc_tone_step),
      .scan_freeze_active(pc_frozen_pix[1]),.frame_underflow(video_underflow || video_last_underflow),.data_page_active(pose_page_active),
      .freeze_request(pc_freeze_request),.gamma_bypass(pc_bypass),.hud_enable(pc_hud));
    reg [31:0] diagnostic_meta,diagnostic_sync;
//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
/////////////////////////////////////////////////////////////////////////////////////
//PLL

    pll u_pll (
        .clkin1   (  sys_clk    ),//27MHz board clock
        .clkout0  (  video_clk  ),
        .clkout0n (  pix_clk    ),//74.25MHz 1080P30
        .clkout1  (  cfg_clk    ),//10MHz
        .clkout2  (  clk_25M    ),//24.75MHz SCCB source, legacy port name
        .lock (  locked     )
    );

    wire imu_link_reset,imu_rx_seen,imu_uart_error,imu_link_locked;
    wire [3:0] imu_baud_index;
    wire signed [15:0] imu_relative_angle;
    wire signed [15:0] imu_stable_angle;
    reg imu_angle_sample_valid;
    wire [1:0] imu_selected_axis;
    wire cmos1_line_end,cmos2_line_end;
    wire [1:0] camera_config_error;
    wire [1:0] camera_exposure_locked;
    wire selected_exposure_locked;
    wire selected_line_end,selected_camera_error;
    wire video_underflow,video_last_underflow,capture_overflow,render_busy;
    wire camera_frame_valid,render_frame_valid,scan_frame_ready;
    wire [31:0] video_debug_ddr,video_debug_sys;
    wire signed [15:0] imu_applied_ddr,imu_applied_sys;
    wire imu_quaternion_valid,imu_magnetic_valid;
    wire signed [15:0] imu_q0_raw,imu_q1_raw,imu_q2_raw,imu_q3_raw,imu_mx_raw,imu_my_raw,imu_mz_raw,imu_mag_temperature_raw;
    imu_link #(.EXTENDED_REPORTS(1)) u_imu_link(.clk(sys_clk),.rst_n(rstn_out),.rx(imu_uart_rx),.tx(imu_uart_tx),
       .attitude_valid(imu_attitude_valid),.config_complete(imu_gyro_known && imu_acc_known && imu_algorithm_known && imu_mounting_known),
       .rx_data(imu_rx_data),.rx_valid(imu_rx_data_valid),
       .rx_seen(imu_rx_seen),.uart_error(imu_uart_error),.baud_index(imu_baud_index),
       .locked(imu_link_locked),.link_rst_n(imu_link_reset));
    // IMU_COMPENSATION_BEGIN
    wire signed [15:0] imu_projection_angle;
    wire imu_projection_valid,imu_projection_good;
    wire [1:0] imu_tracker_axis;
    // A candidate checksum-valid report is not yet a locked UART stream.
    // Optional gravity projection retains lock/epoch protection. Legacy Euler
    // uses the first parser-valid report, exactly as the successful OPT12.
    wire imu_attitude_ready=imu_link_locked && imu_link_reset;
    wire imu_trusted_attitude_valid=imu_attitude_valid && imu_attitude_ready;
    wire signed [15:0] imu_calibrated_angle;
    wire imu_calibrated_valid,imu_calibrated_good;
    imu_gravity_projector_calibrated #(
      .PLANE_X_X(MOUNT_PLANE_X_X),.PLANE_X_Y(MOUNT_PLANE_X_Y),.PLANE_X_Z(MOUNT_PLANE_X_Z),
      .PLANE_Y_X(MOUNT_PLANE_Y_X),.PLANE_Y_Y(MOUNT_PLANE_Y_Y),.PLANE_Y_Z(MOUNT_PLANE_Y_Z)) u_mount_projection(
      .clk(sys_clk),.rst_n(rstn_out),.stream_ready(imu_attitude_ready),
      .sample_valid(imu_trusted_attitude_valid),.roll_raw(imu_roll_raw),.pitch_raw(imu_pitch_raw),
      .angle_out(imu_calibrated_angle),.angle_valid(imu_calibrated_valid),.good(imu_calibrated_good),.busy());
    // A calibrated angle already has a saved absolute installation reference.
    // Do not subtract a new startup Roll from this angle on every download.
    wire signed [15:0] imu_image_source_angle=MOUNT_CALIBRATION_ENABLE?imu_calibrated_angle:imu_relative_angle;
    wire imu_image_sample_valid=MOUNT_CALIBRATION_ENABLE?imu_calibrated_valid:imu_comp_sample_valid;
    wire imu_image_projection_good=MOUNT_CALIBRATION_ENABLE?imu_calibrated_good:imu_projection_good;

    imu_gravity_projector #(.FORWARD_AXIS(IMU_CAMERA_AXIS<0?0:IMU_CAMERA_AXIS),
       .FORWARD_SIGN(IMU_CAMERA_FORWARD_SIGN)) u_gravity_projector(
       .clk(sys_clk),.rst_n(rstn_out),.stream_ready(imu_attitude_ready),.sample_valid(imu_trusted_attitude_valid),
       .roll_raw(imu_roll_raw),.pitch_raw(imu_pitch_raw),.angle_out(imu_projection_angle),
       .angle_valid(imu_projection_valid),.good(imu_projection_good),.busy());
    wire imu_comp_sample_valid=(IMU_CAMERA_AXIS<0)?imu_attitude_valid:(imu_attitude_ready && imu_projection_valid);
    wire signed [15:0] imu_comp_angle=(IMU_CAMERA_AXIS<0)?imu_roll_raw:imu_projection_angle;
    assign imu_selected_axis=MOUNT_CALIBRATION_ENABLE?2:((IMU_CAMERA_AXIS<0)?imu_tracker_axis:IMU_CAMERA_AXIS);
    imu_angle_tracker #(.AXIS(IMU_CAMERA_AXIS<0?IMU_ROT_AXIS:0)) u_angle_tracker(
       .clk(sys_clk),.rst_n(rstn_out),.attitude_valid(imu_comp_sample_valid),
       .roll_raw(imu_comp_angle),.pitch_raw(imu_pitch_raw),.yaw_raw(imu_yaw_raw),
       .angle_raw(imu_relative_angle),.selected_axis(imu_tracker_axis));

    // Tracker output is registered: delay its valid strobe one clock so the
    // stabilizer samples this report, rather than the previous report's angle.
    always @(posedge sys_clk or negedge rstn_out)
      if(!rstn_out) imu_angle_sample_valid<=0;
      else imu_angle_sample_valid<=imu_image_sample_valid;
    imu_angle_stabilizer #(.ADAPTIVE_ENABLE(IMU_SMOOTHING_ENABLE==2)) u_angle_stabilizer(.clk(sys_clk),.rst_n(rstn_out),
       .sample_valid(imu_angle_sample_valid),.angle_in(imu_image_source_angle),.angle_out(imu_stable_angle));
    // IMU_COMPENSATION_END

    imu_ms901m_parser u_imu_parser (
        .clk             (sys_clk),
        .rst_n           (imu_link_reset),
        .rx_data         (imu_rx_data),
        .rx_valid        (imu_rx_data_valid),
        .quaternion_valid(imu_quaternion_valid),.magnetic_valid(imu_magnetic_valid),
        .q0_raw(imu_q0_raw),.q1_raw(imu_q1_raw),.q2_raw(imu_q2_raw),.q3_raw(imu_q3_raw),
        .mx_raw(imu_mx_raw),.my_raw(imu_my_raw),.mz_raw(imu_mz_raw),.mag_temperature_raw(imu_mag_temperature_raw),
        .attitude_valid  (imu_attitude_valid),
        .roll_raw        (imu_roll_raw),
        .pitch_raw       (imu_pitch_raw),
        .yaw_raw         (imu_yaw_raw),
        .motion_valid     (imu_motion_valid),
        .ax_raw           (imu_ax_raw),
        .ay_raw           (imu_ay_raw),
        .az_raw           (imu_az_raw),
        .gx_raw           (imu_gx_raw),
        .gy_raw           (imu_gy_raw),
        .gz_raw           (imu_gz_raw),
        .altitude_valid   (imu_altitude_valid),
        .pressure_raw     (imu_pressure_raw),
        .altitude_cm      (imu_altitude_cm),
        .temperature_raw  (imu_temperature_raw),
        .frame_id         (),
        .frame_error      (imu_frame_error),
        .gyro_range(imu_gyro_range),.acc_range(imu_acc_range),
        .gyro_known(imu_gyro_known),.acc_known(imu_acc_known),
        .algorithm_known(imu_algorithm_known),.algorithm_mag(imu_algorithm_mag),
        .mounting_known(imu_mounting_known),.mounting_vertical(imu_mounting_vertical)
    );

    assign imu_alive=imu_link_locked;
    assign imu_altitude_seen=1'b0;
    assign imu_rotation_mode=imu_image_source_angle[15:14];
    wire signed [15:0] imu_render_angle;
    // IMU_OPT12_PATH_BEGIN: raw mode 0; filtered modes 1 (legacy) and 2 (adaptive).
    // Keep optional smoothing in its own elaboration branch. The image sign
    // is applied exactly once, inside fram_buf (current STABLE13 direction: +1).
    generate if(IMU_SMOOTHING_ENABLE==0) begin:opt12_raw_path
      imu_angle_cdc u_angle_cdc(.src_clk(sys_clk),.dst_clk(core_clk),.rst_n(rstn_out),
         .src_angle(imu_image_source_angle),.dst_angle(imu_render_angle));
    end else begin:stable13_path
      imu_angle_cdc u_angle_cdc(.src_clk(sys_clk),.dst_clk(core_clk),.rst_n(rstn_out),
         .src_angle(imu_stable_angle),.dst_angle(imu_render_angle));
    end endgenerate
    // IMU_OPT12_PATH_END

    imu_angle_cdc u_applied_cdc(.src_clk(core_clk),.dst_clk(sys_clk),.rst_n(rstn_out),
       .src_angle(imu_applied_ddr),.dst_angle(imu_applied_sys));
    imu_snapshot_cdc #(.W(32)) u_video_debug_cdc(.src_clk(core_clk),.dst_clk(sys_clk),.rst_n(rstn_out),
       .src_data(video_debug_ddr),.dst_data(video_debug_sys));
    reg video_underflow_seen,ddr_ready_meta,ddr_ready_sync,ddr_was_ready,ddr_reset_seen;
    always @(posedge video_clk or negedge video_timing_rstn)
      if(!video_timing_rstn) begin
       video_underflow_seen<=0;ddr_ready_meta<=0;ddr_ready_sync<=0;ddr_was_ready<=0;ddr_reset_seen<=0;
      end else begin
       ddr_ready_meta<=ddr_init_done;ddr_ready_sync<=ddr_ready_meta;
       if(ddr_ready_sync) ddr_was_ready<=1;
       if(ddr_was_ready && !ddr_ready_sync) ddr_reset_seen<=1;
       if(video_last_underflow) video_underflow_seen<=1;
      end
    // IMU_RX_DIAGNOSTICS_BEGIN
    // These flags survive candidate-baud restarts, unlike rx_seen. They never
    // authorize attitude data or alter the parser, reference, or image angle.
    reg imu_rx_meta_diag,imu_rx_sync_diag,imu_rx_prev_diag,imu_rx_armed;
    reg [2:0] imu_diag_ready;
    reg imu_edge_ever,imu_byte_ever,imu_scan_wrapped;
    reg [3:0] imu_previous_baud;
    always @(posedge sys_clk or negedge rstn_out) begin
      if(!rstn_out) begin
        imu_rx_meta_diag<=1;imu_rx_sync_diag<=1;imu_rx_prev_diag<=1;imu_rx_armed<=0;
        imu_diag_ready<=0;
        imu_edge_ever<=0;imu_byte_ever<=0;imu_scan_wrapped<=0;imu_previous_baud<=0;
      end else begin
        imu_rx_meta_diag<=imu_uart_rx;imu_rx_sync_diag<=imu_rx_meta_diag;imu_rx_prev_diag<=imu_rx_sync_diag;
        imu_diag_ready<={imu_diag_ready[1:0],1'b1};
        if(imu_diag_ready[2] && imu_rx_sync_diag)imu_rx_armed<=1;
        if(imu_rx_armed && imu_rx_prev_diag && !imu_rx_sync_diag)imu_edge_ever<=1;
        if(imu_rx_data_valid)imu_byte_ever<=1;
        if(imu_previous_baud==10 && imu_baud_index==0)imu_scan_wrapped<=1;
        imu_previous_baud<=imu_baud_index;
      end
    end
    // IMU_RX_DIAGNOSTICS_END
    wire [31:0] diagnostic_live={imu_scan_wrapped,imu_byte_ever,imu_edge_ever,imu_mounting_known,imu_mounting_vertical,imu_image_projection_good,selected_exposure_locked,ddr_reset_seen,scan_frame_ready,render_frame_valid,camera_frame_valid,
       render_busy,init_over_tx,ddr_init_done,init_done,
       imu_selected_axis,imu_baud_index,2'd0,imu_acc_known,imu_gyro_known,
       video_underflow_seen,capture_overflow,selected_camera_error,imu_frame_error,
       imu_uart_error,imu_rx_seen,imu_link_locked};
    always @(posedge sys_clk or negedge rstn_out)
      if(!rstn_out) begin diagnostic_meta<=0;diagnostic_sync<=0;end
      else begin diagnostic_meta<=diagnostic_live;diagnostic_sync<=diagnostic_meta;end
    imu_telemetry u_telemetry(.clk(sys_clk),.rst_n(rstn_out),.stream_rst_n(imu_link_reset),.link_locked(imu_link_locked),
       .attitude_valid(imu_attitude_valid),.motion_valid(imu_motion_valid),.altitude_valid(imu_altitude_valid),
       .roll_raw(imu_roll_raw),.pitch_raw(imu_pitch_raw),.yaw_raw(imu_yaw_raw),
       .gx_raw(imu_gx_raw),.gy_raw(imu_gy_raw),.gz_raw(imu_gz_raw),
       .ax_raw(imu_ax_raw),.ay_raw(imu_ay_raw),.az_raw(imu_az_raw),
       .pressure_pa(imu_pressure_raw),.altitude_cm(imu_altitude_cm),.temperature_centi(imu_temperature_raw),
       .rotation_raw(imu_applied_sys),.gyro_range(imu_gyro_range),.acc_range(imu_acc_range),
       .gyro_known(imu_gyro_known),.acc_known(imu_acc_known),
       .algorithm_known(imu_algorithm_known),.algorithm_mag(imu_algorithm_mag),
       .diagnostic_status(diagnostic_sync),.video_debug(video_debug_sys),.snapshot(telemetry_sys));
    // Raw pose and scaled numeric values cross as ONE coherent sensor snapshot.
    imu_snapshot_cdc #(.W(688)) u_telemetry_cdc(.src_clk(sys_clk),.dst_clk(video_clk),.rst_n(rstn_out),
       .src_data({imu_yaw_raw,imu_pitch_raw,imu_roll_raw,telemetry_sys}),.dst_data({pose_angles_pix,telemetry_pix}));

//配置7210
wire init_over_tx;
    ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ), //input       clk,
        .rst_n           (  rstn_out       ), //input       rstn,
        .init_over    (  init_over_tx   ), //output      init_over,                                
        .init_over_rx    (  init_over_rx   ), //output      init_over,
//        .iic_tx_scl      (  iic_tx_scl     ), //output      iic_scl,
//        .iic_tx_sda      (  iic_tx_sda     ), //inout       iic_sda
        .iic_scl         (  iic_tx_scl        ), //output      iic_scl,
        .iic_sda         (  iic_tx_sda        )  //inout       iic_sda
    );
//   assign    hdmi_int_led    =    init_over_tx; 
    
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 16'h2710);

//配置CMOS///////////////////////////////////////////////////////////////////////////////////
//OV5640 register configure enable    
    power_on_delay	power_on_delay_inst(
    	.clk_50M                 (sys_clk        ),//legacy port name; actual 27MHz
    	.reset_n                 (locked         ),//input	
    	.camera1_rstn            (cmos1_reset    ),//output
    	.camera2_rstn            (cmos2_reset    ),//output	
    	.camera_pwnd             (               ),//output
    	.initial_en              (initial_en     ) //output		
    );
//CMOS1 Camera 
    reg_config #(.SENSOR_PROFILE(CAMERA_PROFILE),.SENSOR_TEST_PATTERN(SENSOR_TEST_PATTERN),
       .SENSOR_XCLK_HZ(SENSOR_XCLK_HZ),.LOCK_EXPOSURE(CAMERA_LOCK_EXPOSURE),.ENABLE_LIGHT_METER(METER_1)) coms1_reg_config(
        .camera_vs(cmos1_vsync),.light_valid(light_status1[42]),
        .light_index(light_status1[41:34]),.light_metric_q4(light_status1[54:43]),
        .light_error(light_status1[33:26]),.exposure_lines(light_status1[25:10]),
        .gain_q4(light_status1[9:0]),
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos1_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos1_scl          ),//output
    	.i2c_sdat                (cmos1_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[0]  ),
        .reg_conf_error          (camera_config_error[0]),//output config_finished
        .exposure_locked         (camera_exposure_locked[0]),
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );

//CMOS2 Camera 
    reg_config #(.SENSOR_PROFILE(CAMERA_PROFILE),.SENSOR_TEST_PATTERN(SENSOR_TEST_PATTERN),
       .SENSOR_XCLK_HZ(SENSOR_XCLK_HZ),.LOCK_EXPOSURE(CAMERA_LOCK_EXPOSURE),.ENABLE_LIGHT_METER(METER_2)) coms2_reg_config(
        .camera_vs(cmos2_vsync),.light_valid(light_status2[42]),
        .light_index(light_status2[41:34]),.light_metric_q4(light_status2[54:43]),
        .light_error(light_status2[33:26]),.exposure_lines(light_status2[25:10]),
        .gain_q4(light_status2[9:0]),
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos2_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos2_scl          ),//output
    	.i2c_sdat                (cmos2_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[1]  ),
        .reg_conf_error          (camera_config_error[1]),//output config_finished
        .exposure_locked         (camera_exposure_locked[1]),
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );
//CMOS 8bit转16bit///////////////////////////////////////////////////////////////////////////////////
//CMOS1
    always@(posedge cmos1_pclk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),//input
    	.rst_n          (cmos_init_done[0]),//input
    	.pdata_i        (cmos1_d_d0       ),//input[7:0]
    	.de_i           (cmos1_href_d0    ),//input
    	.vs_i           (cmos1_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos1_pclk_16bit ),//output
    	.pdata_o        (cmos1_d_16bit    ),//output[15:0]
    	.line_end       (cmos1_line_end),.line_active(cmos1_monitor_line),
        .de_o           (cmos1_href_16bit ) //output
    );
//CMOS2
    always@(posedge cmos2_pclk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),//input
    	.rst_n          (cmos_init_done[1]),//input
    	.pdata_i        (cmos2_d_d0       ),//input[7:0]
    	.de_i           (cmos2_href_d0    ),//input
    	.vs_i           (cmos2_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos2_pclk_16bit ),//output
    	.pdata_o        (cmos2_d_16bit    ),//output[15:0]
    	.line_end       (cmos2_line_end),.line_active(cmos2_monitor_line),
        .de_o           (cmos2_href_16bit ) //output
    );
//输入视频源选择//////////////////////////////////////////////////////////////////////////////////////////
`ifdef CMOS_1
assign     pclk_in_test    =    cmos1_pclk_16bit    ;
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;
assign     selected_line_end = cmos1_line_end;
assign     selected_camera_error = camera_config_error[0];
assign     selected_exposure_locked = camera_exposure_locked[0];
assign     i_rgb565        =    (CAMERA_PROFILE==1)?cmos1_d_16bit:{cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};//{r,g,b}
`elsif CMOS_2
assign     pclk_in_test    =    cmos2_pclk_16bit    ;
assign     vs_in_test      =    cmos2_vsync_d0      ;
assign     de_in_test      =    cmos2_href_16bit    ;
assign     selected_line_end = cmos2_line_end;
assign     selected_camera_error = camera_config_error[1];
assign     selected_exposure_locked = camera_exposure_locked[1];
assign     i_rgb565        =    (CAMERA_PROFILE==1)?cmos2_d_16bit:{cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};//{r,g,b}
`endif
 assign    hdmi_int_led    = (HDMI_LED_MODE==1)?init_over_tx:|i_rgb565;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Measurement signals are observational only; never enter image/control paths.
    wire [31:0] performance_time_gray,performance_source_id,performance_render_id,performance_source_start;
    wire [63:0] performance_render_origin,performance_display_origin;
    wire [511:0] performance_host;
    cdc_mailbox #(.WIDTH(512)) performance_readback(.src_clk(core_clk),.src_rst_n(ddr_init_done),.dst_clk(sys_clk),.dst_rst_n(rstn_out),.src_data(performance_probe),.dst_data(performance_host));
    wire [511:0] performance_probe /* synthesis PAP_MARK_DEBUG="1" syn_keep=1 */;
    rect_performance_monitor performance(.clk(core_clk),.rst_n(ddr_init_done),.cam_clk(pclk_in_test),.cam_vs(vs_in_test),
      .pix_clk(video_clk),.pix_vs(resource_vs),.pix_de(resource_de),.pix_underflow(video_underflow),
      .source_id(performance_source_id),.render_id(performance_render_id),.source_start_tick(performance_source_start),
      .render_origin(performance_render_origin),.display_origin(performance_display_origin),.render_busy(render_busy),
      .capture_overflow(capture_overflow),.time_gray(performance_time_gray),.probe(performance_probe));
//修改ddr读写模块v1
    fram_buf #(.ROTATION_DIRECTION(IMU_IMAGE_DIRECTION),.NETWORK_PROCESSED(1),.GAMMA_POLICY(GAMMA_POLICY)) fram_buf(
        .performance_time_gray(performance_time_gray),.performance_source_id(performance_source_id),.performance_render_id(performance_render_id),
        .performance_source_start(performance_source_start),.performance_render_origin(performance_render_origin),.performance_display_origin(performance_display_origin),
        .geometry_request(geometry_core),.display_geometry(display_geometry_core),.lens_request(lens_request_core),.lens_status(lens_status_core),
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  ddr_init_done        ),//input                         ddr_rstn,
        //data_in                                  
        .vin_clk        (  pclk_in_test         ),//input                         vin_clk,
        .wr_fsync       (  vs_in_test           ),//input                         wr_fsync,
        .wr_en          (  de_in_test           ),//input                         wr_en,
        .wr_data        (  i_rgb565             ),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  video_clk            ),//input                         vout_clk,
        .rd_fsync       (  vs_o               ),//input                         rd_fsync,
        .rd_en          (  de_re                ),//input                         rd_en,
        .rotation_angle (  imu_render_angle ),
        .freeze_display(pc_freeze_core[1]),.freeze_active(pc_frozen_core),
        .net_processed(net_processed),.net_frame_model(net_frame_model),.net_gamma_bypass(pc_bypass),.net_gamma_step(pc_tone_step),
        .net_clk(net_clk),.net_rst_n(net_rst_n),.net_hold(net_hold),.net_line_request(net_line_request),
        .net_row(net_row),.net_read_x(net_read_x),.net_pixel(net_pixel),.net_pixel_next(net_pixel_next),.net_pixel_previous(net_pixel_previous),.net_pixel_previous_next(net_pixel_previous_next),.net_pair_valid(net_pair_valid),.net_frame_geometry(net_frame_geometry),.net_ready(net_ready),.net_line_done(net_line_done),.net_frame_id(net_frame_id),
        .wr_line_end    (  selected_line_end    ),
        .video_underflow(video_underflow),
        .video_last_underflow(video_last_underflow),
        .capture_overflow(capture_overflow),
        .render_busy(render_busy),.applied_angle(imu_applied_ddr),
        .camera_frame_valid(camera_frame_valid),.render_frame_valid(render_frame_valid),
        .scan_frame_ready(scan_frame_ready),.video_debug(video_debug_ddr),.rotation_trace(rotation_trace_ddr),
        .vout_valid     (  frame_pixel_valid    ),
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .init_done      (  init_done            ),//output reg                    init_done,
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (                       ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ) // input[3:0]         
    );

    // Observation only: four poses already owned by the DDR image pipeline.
    // R=requested, C=last complete capture, O=last render, A=scan-selected frame.
    imu_snapshot_cdc #(.W(64)) u_rotation_trace_cdc(
      .src_clk(core_clk),.dst_clk(video_clk),.rst_n(rstn_out),
      .src_data(rotation_trace_ddr),.dst_data(rotation_trace_pix));

     hdmi_sdr_output hdmi_output(.clk(video_clk),.rst_n(video_timing_rstn),.rgb(resource_rgb),
      .vs(resource_vs),.hs(resource_hs),.de(resource_de),.out_rgb({r_out,g_out,b_out}),.out_vs(vs_out),.out_hs(hs_out),.out_de(de_out));
     // Framebuffer has two pixel clocks of latency; delay all sync/coordinates
     // by the same amount. Chinese text/font RAM adds two matched RGB/sync clocks.
     always @(posedge video_clk) begin
      if(!video_timing_rstn) begin
       sync_x_d1<=0;sync_x_d2<=0;sync_y_d1<=0;sync_y_d2<=0;
       sync_vs_d1<=0;sync_vs_d2<=0;sync_hs_d1<=0;sync_hs_d2<=0;sync_de_d1<=0;sync_de_d2<=0;
      end else begin
       sync_x_d1<=sync_x_act;sync_x_d2<=sync_x_d1;
       sync_y_d1<=sync_y_act;sync_y_d2<=sync_y_d1;
       sync_vs_d1<=vs_o;sync_vs_d2<=sync_vs_d1;
       sync_hs_d1<=hs_o;sync_hs_d2<=sync_hs_d1;sync_de_d1<=de_re;sync_de_d2<=sync_de_d1;
      end
     end
     // V4: BKB scene adaptation meters raw camera pixels. Only its controller
     // survives synthesis here; RGB output logic is unused. Raw calibration
     // capture remains unchanged. Gamma is applied once in rect_frame_engine.
     wire [7:0] status_mean,status_contrast,source_mean,source_contrast;
     wire [4:0] source_tone;
     wire [48:0] light_camera;
     cdc_mailbox #(.WIDTH(49)) meter_sensor_cdc(.src_clk(video_clk),.src_rst_n(rstn_out),
      .dst_clk(pclk_in_test),.dst_rst_n(rstn_out),
      .src_data({pc_frozen_pix[1],pc_bypass,camera_light_valid,camera_light_index,camera_light_error,camera_gain_q4,camera_light_metric_q4,8'd0}),.dst_data(light_camera));
     adaptive_preprocess_osd #(.GAMMA_POLICY(GAMMA_POLICY),.GAMMA_BYPASS(BRIGHTNESS_GAMMA_BYPASS),
       .REQUIRE_VALID_PIXELS(1)) u_camera_adaptation(
       .clk(pclk_in_test),.rstn(rstn_out),.hold_frame(light_camera[48]),.runtime_bypass(light_camera[47]),.osd_enable(1'b0),
       .vs_in(vs_in_test),.hs_in(1'b0),.de_in(de_in_test),.pixel_valid(1'b1),.x_in(12'd0),.y_in(12'd0),.rgb_in(i_rgb565),
       .rotation_trace(64'd0),.lens_state_live(35'd0),.camera_fps_bcd_live(8'd0),.camera_fps_valid_live(1'b0),
       .camera_width_bcd_live(16'd0),.camera_height_bcd_live(16'd0),.camera_geometry_valid_live(1'b0),.camera_geometry_bad_live(1'b0),
       .camera_light_valid_live(light_camera[46]),.camera_light_index_live(light_camera[45:38]),.camera_light_error_live(light_camera[37:30]),
       .camera_exposure_lines_live(16'd0),.camera_gain_q4_live(light_camera[29:20]),.camera_light_metric_q4_live(light_camera[19:8]),
       .vs_out(),.hs_out(),.de_out(),.x_out(),.y_out(),.rgb_out(),.rgb888_out(),.clean_rgb(),
       .mean_luma(source_mean),.gain_q8(),.contrast(source_contrast),.gamma_mode(),.gamma_step(),.tone_step(source_tone));
     cdc_mailbox #(.WIDTH(21)) meter_video_cdc(.src_clk(pclk_in_test),.src_rst_n(rstn_out),
       .dst_clk(video_clk),.dst_rst_n(video_timing_rstn),.src_data({source_mean,source_contrast,source_tone}),
       .dst_data({status_mean,status_contrast,pc_tone_step}));
     // No display-only brightness curve: these are the same rendered RGB565
     // pixels read by current-mode photographs and continuous PC capture.
     assign gamma_vs=sync_vs_d2;assign gamma_hs=sync_hs_d2;assign gamma_de=sync_de_d2;
     assign gamma_x=sync_x_d2;assign gamma_y=sync_y_d2;
     assign clean_gamma_rgb=o_rgb565;
     assign gamma_rgb888={o_rgb565[15:11],o_rgb565[15:13],o_rgb565[10:5],o_rgb565[10:9],o_rgb565[4:0],o_rgb565[4:2]};
     wire [23:0] status_rgb;wire [11:0] status_x,status_y;wire status_vs,status_hs,status_de;
     wire eval_vs,eval_hs,eval_de;wire [11:0] eval_x,eval_y;wire [23:0] eval_rgb;
     wire display_vs,display_hs,display_de;wire [11:0] display_x,display_y;wire [23:0] display_rgb;
     assign display_vs=gamma_vs;assign display_hs=gamma_hs;assign display_de=gamma_de;
     assign display_x=gamma_x;assign display_y=gamma_y;assign display_rgb=gamma_rgb888;
     rect_evaluation_overlay local_evaluation(.clk(video_clk),.rst_n(video_timing_rstn),.vs(display_vs),.hs(display_hs),.de(display_de),.x(display_x),.y(display_y),.rgb(display_rgb),
      .numbers_visible(numbers_pix),.enabled(evaluation_pix&&!page_pix),.horizontal(horizontal_pix),.editing(editing_pix),.position(position_pix),.center_x(center_x_pix),.center_y(center_y_pix),
      .actual_geometry(display_geometry_pix[31:0]),.calibration_ready(scan_ready_pix[1]),.calibration_fault(lens_state_pix[0]),.parameters(lens_status_pix[351:0]),
      .out_vs(eval_vs),.out_hs(eval_hs),.out_de(eval_de),.out_x(eval_x),.out_y(eval_y),.out_rgb(eval_rgb));
     rect_status_overlay chinese_status(.clk(video_clk),.rst_n(video_timing_rstn),.enable(1'b0),
      .vs(eval_vs),.hs(eval_hs),.de(eval_de),.x(eval_x),.y(eval_y),.rgb(eval_rgb),.actual_geometry(display_geometry_pix[31:0]),
      .plane_held(ph_pix),.plane_valid(pv_pix),.mapping_fault(lens_state_pix[0]),
      .out_vs(status_vs),.out_hs(status_hs),.out_de(status_de),.out_x(status_x),.out_y(status_y),.out_rgb(status_rgb));
     wire param_vs,param_hs,param_de;wire [11:0] param_x,param_y;wire [23:0] param_rgb;
     // Reference yellow parameters only. Gamma, camera configuration, geometry
     // and pixel values outside the opaque glyph pixels are untouched.
     status_cn_overlay reference_parameters(.clk(video_clk),.rst_n(video_timing_rstn),.hold_frame(pc_frozen_pix[1]),.enable(numbers_pix&&caption_pix&&!evaluation_pix&&!page_pix),
      .vs(status_vs),.hs(status_hs),.de(status_de),.x(status_x),.y(status_y),.rgb(status_rgb),
      .mean(status_mean),.contrast(status_contrast),.gamma_step(pc_tone_step),.bypass(pc_bypass),.lens(lens_state_pix),.angles(rotation_trace_pix),
      .fps(camera_fps_bcd),.fps_valid(camera_fps_valid),.width_bcd(camera_width_bcd),.height_bcd(camera_height_bcd),
      .geometry_valid(camera_geometry_valid),.geometry_bad(camera_geometry_bad),.exposure(camera_exposure_lines),.gain(camera_gain_q4),
      .light(camera_light_index),.light_error(camera_light_error),.light_valid(camera_light_valid),
      .out_vs(param_vs),.out_hs(param_hs),.out_de(param_de),.out_x(param_x),.out_y(param_y),.out_rgb(param_rgb));
     wire [23:0] page_rgb;
     wire [23:0] resource_rgb;wire resource_vs,resource_hs,resource_de;
     wire [11:0] hud_x,hud_y;wire [8:0] resource_text_address;wire [7:0] resource_text_char;
     // Resources are supplied by the fresh build report in the host package,
     // never by stale constants copied from a previous bitstream.
     assign resource_byte=8'd0;assign resource_text_char=8'd0;
     assign resource_rgb=hud_rgb888;assign resource_vs=hud_vs;assign resource_hs=hud_hs;assign resource_de=hud_de;
     wire [11:0] page_x,page_y;
     wire page_vs,page_hs,page_de;
     wire [639:0] page_telemetry;
     pose_page_overlay #(.LAYOUT_V8(1)) pose_panel(.clk(video_clk),.rst_n(video_timing_rstn),.key4_n(!page_pix),.hold_frame(pc_frozen_pix[1]),
      .vs(param_vs),.hs(param_hs),.de(param_de),.x(param_x),.y(param_y),.rgb(param_rgb),
      .angles(pose_angles_pix),.telemetry(telemetry_pix),.page_active(pose_page_active),.page_telemetry(page_telemetry),
      .out_x(page_x),.out_y(page_y),.out_vs(page_vs),.out_hs(page_hs),.out_de(page_de),.out_rgb(page_rgb));
     imu_hud_overlay #(.IMAGE_DIRECTION(IMU_IMAGE_DIRECTION),.RGB888_INPUT(1),.REFLOW_PAGE(1),
       .BUILD_MAJOR(8'h32),.BUILD_DIGIT(8'h38),.SMOOTHING_MODE(IMU_SMOOTHING_ENABLE),.PROJECTION_AXIS(MOUNT_CALIBRATION_ENABLE?3:IMU_CAMERA_AXIS)) u_imu_hud_overlay(
       .pix_clk(video_clk),.rst_n(video_timing_rstn),.hold_frame(pc_frozen_pix[1]),.osd_enable((numbers_pix&&!evaluation_pix)||pose_page_active),.data_page(pose_page_active),
       .video_de(page_de),.video_vs(page_vs),.video_hs(page_hs),
       .pix_x(page_x),.pix_y(page_y),.rgb_in(16'd0),.rgb888_in(page_rgb),.telemetry(pose_page_active?page_telemetry:telemetry_pix),
       .rgb_out(hud_rgb565),.rgb888_out(hud_rgb888),.de_out(hud_de),.vs_out(hud_vs),.hs_out(hud_hs),.out_x(hud_x),.out_y(hud_y));

 /////////////////////////////////////////////////////////////////////////////////////
 //产生visa时序 
     sync_vg sync_vg(                            
        .clk            (  video_clk            ),//input                   clk,                                 
        .rstn           (  video_timing_rstn        ),//input                   rstn,
        .vs_out         (  vs_o                 ),//output reg              vs_out,                                                                                                                                      
        .hs_out         (  hs_o                 ),//output reg              hs_out,            
        .de_out         (  de_re                     ),//output reg              de_out, 
        .de_re          (                  ),
        .x_act          (  sync_x_act          ),
        .y_act          (  sync_y_act          )
    );  
////////////////////////////////////////////////////////////////////////////////////////////
//ddr    
/*        ddr3_100h ddr3_100h (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output
             .ddrphy_clkin              (core_clk           ),// output
             .pll_lock                  (pll_lock           ),// output

             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug
             .debug_data                (                   ),// output [135:0]
             .debug_slice_state         (                   ),// output [51:0]
             .debug_calib_ctrl          (                   ),// output [21:0]
             .ck_dly_set_bin            (                   ),// output [7:0]
             .force_ck_dly_en           (1'b0               ),// input
             .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
             .dll_step                  (                   ),// output [7:0]
             .dll_lock                  (                   ),// output
             .init_read_clk_ctrl        (2'b0               ),// input [1:0]
             .init_slip_step            (4'b0               ),// input [3:0]
             .force_read_clk_ctrl       (1'b0               ),// input
             .ddrphy_gate_update_en     (1'b0               ),// input
             .update_com_val_err_flag   (                   ),// output [3:0]
             .rd_fake_stop              (1'b0               ) // input
       );*/
wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);
 ddr3_test u_ddr3_test_h(
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),          // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
  .debug_cpd_state()                     // output [3:0]
       );
//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
endmodule
