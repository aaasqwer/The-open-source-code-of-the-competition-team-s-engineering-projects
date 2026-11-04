`timescale 1ns/1ps
// Host protocol and transport only. The existing IMU parser owns the sensor UART.
module return_pc_bridge # (parameter CLK_HZ=27000000,PROCESSED_PREVIEW=0,RESOURCE_TELEMETRY=0,EXTENDED_IMU=0)(
 input [511:0] performance_probe,
 input clk,rst_n,uart_rx,output uart_tx,
 input [7:0] board_ui_flags,input [63:0] display_geometry,output remote_ui_update,output [7:0] remote_ui_flags,
 output [287:0] normal_matrix,plane_matrix,output plane_valid,output [31:0] geometry_version,
 input pix_clk,pix_rst_n,video_vs,video_de,input [15:0] pixel,
 output net_clk,net_rst_n,net_hold,net_line_request,net_processed,output [10:0] net_row,net_read_x,
 input [15:0] net_pixel,net_pixel_next,net_pixel_previous,net_pixel_previous_next,input net_pair_valid,input [63:0] net_frame_geometry,input net_ready,net_line_done,net_frame_model,input [31:0] net_frame_id,
 output [6:0] resource_addr,input [7:0] resource_byte,
 input rgmii_rxc,rgmii_rx_ctl,input [3:0] rgmii_rxd,
 output rgmii_txc,rgmii_tx_ctl,output [3:0] rgmii_txd,
 input [639:0] telemetry,input [1:0] gyro_range,acc_range,
 input [15:0] roll,pitch,yaw,ax,ay,az,gx,gy,gz,temperature,
 input [31:0] pressure,altitude,input attitude_valid,motion_valid,altitude_valid,imu_error,
 input extra_ready,quaternion_valid,magnetic_valid,camera_text_visible,
 input [15:0] q0,q1,q2,q3,mx,my,mz,mag_temperature,
 input camera_ready,ddr_ready,render_ready,input [7:0] camera_fps_bcd,input camera_fps_valid,
 input [4:0] gamma_step,input scan_freeze_active,frame_underflow,data_page_active,
 input [418:0] lens_status,output [416:0] lens_request,
 output freeze_request,output reg gamma_bypass,output reg hud_enable
);
 wire [7:0] rx_data,tx_data;wire rx_valid,rx_error,tx_valid,tx_ready;
 team_uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.BAUD_RATE(921600)) rx(
  .clk(clk),.rst_n(rst_n),.uart_rx(uart_rx),.rx_data(rx_data),.rx_valid(rx_valid),.frame_error(rx_error));
 team_uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_HZ),.BAUD_RATE(921600)) tx(
  .clk(clk),.rst_n(rst_n),.uart_tx(uart_tx),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready));
 wire performance_request;
 wire image_request,ack_update,control_update,stream_enable,snapshot_request,preview_update,preview_enable;
 wire [31:0] image_sequence,command_count,command_errors,preview_session;
 wire [63:0] image_ack;wire [4:0] control_flags;wire [15:0] period_ms;
 wire lens_write,lens_commit,lens_clear,lens_busy;wire [3:0] lens_index;
 wire [31:0] lens_value,lens_id,lens_crc,lens_errors;
 wire geo_write,geo_commit,geo_clear,geo_slot,geo_busy;wire [3:0] geo_index;wire [31:0] geo_value,geo_crc,geo_errors;wire [543:0] geometry_payload;
 host_command_parser #(.CLK_FREQ_HZ(CLK_HZ)) parser(.clk(clk),.rst_n(rst_n),.rx_data(rx_data),.rx_valid(rx_valid),
  .image_request(image_request),.image_ack_update(ack_update),.image_sequence(image_sequence),.image_ack(image_ack),
  .control_update(control_update),.control_flags(control_flags),.stream_enable(stream_enable),.stream_period_ms(period_ms),
  .performance_request(performance_request),.snapshot_request(snapshot_request),.command_count(command_count),.command_error_count(command_errors),.imu_query_request(),
  .preview_update(preview_update),.preview_enable(preview_enable),.preview_session(preview_session),
  .lens_write(lens_write),.lens_commit(lens_commit),.lens_clear(lens_clear),.lens_index(lens_index),.lens_value(lens_value),.lens_id(lens_id),.lens_crc(lens_crc),.geo_write(geo_write),.geo_commit(geo_commit),.geo_clear(geo_clear),.geo_slot(geo_slot),.geo_index(geo_index),.geo_value(geo_value),.geo_crc(geo_crc),.ui_update(remote_ui_update),.ui_flags(remote_ui_flags));
 rect_geometry_bank geometry(.clk(clk),.rst_n(rst_n),.write_word(geo_write),.commit(geo_commit),.clear(geo_clear),.slot(geo_slot),.index(geo_index),.value(geo_value),.expected_crc(geo_crc),
  .normal_matrix(normal_matrix),.plane_matrix(plane_matrix),.plane_valid(plane_valid),.version(geometry_version),.errors(geo_errors),.busy(geo_busy));
 assign geometry_payload={376'd3,display_geometry,geometry_version,geo_errors,8'd0,7'd0,geo_busy,7'd0,plane_valid,board_ui_flags,8'd9};
 lens_config_bank calibration(.clk(clk),.rst_n(rst_n),.write_param(lens_write),.param_index(lens_index),.param_value(lens_value),
  .commit(lens_commit),.clear(lens_clear),.request_id(lens_id),.request_crc(lens_crc),.request_bundle(lens_request),.errors(lens_errors),.busy(lens_busy));
 wire eth_clk,eth_rst_n,eth_ready,eth_fault,eth_abort;
 assign net_clk=eth_clk;assign net_rst_n=eth_rst_n;
 reg [1:0] network_mode;
 reg [1:0] mode_e1,mode_e2;
 always @(posedge eth_clk or negedge eth_rst_n)
  if(!eth_rst_n)begin mode_e1<=1;mode_e2<=1;end else begin mode_e1<=network_mode;mode_e2<=mode_e1;end
 reg [1:0] net_ready_sys;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)net_ready_sys<=0;else net_ready_sys<={net_ready_sys[0],eth_ready};
 // Reset BOTH ends of every transport mailbox when its receiving clock/PLL resets.
 // Commands while disconnected are not replayed into a new Ethernet session.
 wire mailbox_rst_n=rst_n && eth_rst_n;
 reg [32:0] capture_src;reg [64:0] ack_src;reg [33:0] preview_src;
 wire [32:0] capture_dst;wire [64:0] ack_dst;wire [33:0] preview_dst;
 always @(posedge clk or negedge mailbox_rst_n)begin
  if(!mailbox_rst_n)begin capture_src<=0;ack_src<=0;preview_src<=0;end
  else begin
   if(image_request)capture_src<={~capture_src[32],image_sequence};
   if(ack_update)ack_src<={~ack_src[64],image_ack};
   if(preview_update)preview_src<={~preview_src[33],preview_enable,preview_session};
  end
 end
 cdc_mailbox #(.WIDTH(33)) cap_cdc(.src_clk(clk),.src_rst_n(mailbox_rst_n),.dst_clk(eth_clk),.dst_rst_n(mailbox_rst_n),.src_data(capture_src),.dst_data(capture_dst));
 cdc_mailbox #(.WIDTH(65)) ack_cdc(.src_clk(clk),.src_rst_n(mailbox_rst_n),.dst_clk(eth_clk),.dst_rst_n(mailbox_rst_n),.src_data(ack_src),.dst_data(ack_dst));
 cdc_mailbox #(.WIDTH(34)) prev_cdc(.src_clk(clk),.src_rst_n(mailbox_rst_n),.dst_clk(eth_clk),.dst_rst_n(mailbox_rst_n),.src_data(preview_src),.dst_data(preview_dst));
 reg capture_seen,ack_seen,preview_seen,preview_enabled,capture_pending;
 reg [31:0] lease,request_sequence;
 reg [13:0] drain_cycles;
 wire preview_busy,preview_tx_en,transfer_active,packet_active,image_start,image_busy,image_pending;
 wire [7:0] preview_tx_data,image_data;
 wire [3:0] photo_state;wire [2:0] photo_reason;wire [10:0] photo_rows;
 wire [31:0] photo_packets;reg [15:0] photo_aborts;
 wire new_capture=capture_dst[32]!=capture_seen;
 // Allow an already admitted pixel line and its CDC notification to drain.
 // 100 us exceeds one complete 1080p30 line (2200/74.25 MHz).
 // The last PGPK bytes may already be assembled while their final MAC frame
 // still awaits ACK. Do not resume preview or admit another capture then.
 wire capture_event=capture_pending&&!new_capture&&drain_cycles==12500&&!preview_busy&&!transfer_active&&!image_pending&&!packet_active;
 wire preview_run=preview_enabled&&!capture_pending&&!new_capture&&!transfer_active&&!image_pending&&!packet_active;
 always @(posedge eth_clk or negedge eth_rst_n)begin
  if(!eth_rst_n)begin capture_seen<=0;ack_seen<=0;preview_seen<=0;preview_enabled<=0;capture_pending<=0;lease<=0;request_sequence<=0;drain_cycles<=0;end
  else begin
   capture_seen<=capture_dst[32];ack_seen<=ack_dst[64];
   if(new_capture)begin capture_pending<=1;request_sequence<=capture_dst[31:0];preview_enabled<=0;drain_cycles<=0;end
   else if(capture_event)capture_pending<=0;
   else if(capture_pending&&drain_cycles<12500)drain_cycles<=drain_cycles+1'b1;
   if(preview_dst[33]!=preview_seen)begin preview_seen<=preview_dst[33];preview_enabled<=preview_dst[32];lease<=0;end
   else if(preview_enabled)begin if(lease==375000000)preview_enabled<=0;else lease<=lease+1'b1;end
  end
 end
 wire line_req,line_done;wire [10:0] line_addr,line_row;wire [15:0] line_pixel;
 // An epoch is changed only when a queued capture actually starts.
 reg capture_epoch;
 always @(posedge eth_clk or negedge eth_rst_n)if(!eth_rst_n)capture_epoch<=0;else if(capture_event)capture_epoch<=~capture_epoch;
 reg [1:0] underflow_eth;
 always @(posedge eth_clk or negedge eth_rst_n)
  if(!eth_rst_n)underflow_eth<=0;
  else underflow_eth<={underflow_eth[0],!net_ready};
 // Before capture, the controller waits for a coherent complete DDR-frame lease.
 // During data transfer, losing that ownership (for example on DDR reset)
 // invalidates the photo even if the underflow accumulator was reset to zero.
 wire photo_abort,photo_freeze;
 // The photo controller has a synchronous reset in the recovered PHY clock.
 // Without a cable/clock it cannot execute that reset. Never expose its state
 // to the always-running HDMI/DDR domains until the PHY reset is released.
 assign freeze_request=eth_rst_n && photo_freeze;
 wire session_rst_n=eth_rst_n&&!eth_abort;
 always @(posedge eth_clk or negedge eth_rst_n)
  if(!eth_rst_n)photo_aborts<=0;else if(photo_abort)photo_aborts<=photo_aborts+1'b1;
 // All photos read a leased complete DDR frame, independent of HDMI line arrival.
 fast_snapshot_ctrl #(.LEASED_DDR(1)) photo(.clk(eth_clk),.rst_n(session_rst_n),.invalid_frame(underflow_eth[1]),.abort_capture(photo_abort),.capture_pulse(capture_event),.capture_sequence(request_sequence),
  .freeze_request(photo_freeze),.freeze_active_vin(net_ready),.leased_frame_id(net_frame_id),.leased_geometry(net_frame_geometry),.video_vs(video_vs),.mode_pix(network_mode),.model_pix(network_mode==0?1'b1:gamma_bypass),
  .line_request_toggle(line_req),.line_done_toggle_pix(net_line_done),.captured_row_pix(photo_rows),.line_rd_addr(line_addr),.line_rd_data(net_pixel),
  .uart_start(image_start),.uart_data(image_data),.uart_busy(image_busy),.packet_active(packet_active),.transfer_active(transfer_active),
  .capture_state(photo_state),.completed_rows(photo_rows),.abort_reason(photo_reason));
 wire preview_hold,preview_request;wire [10:0] preview_row,preview_x;
 wire raw_photo=photo_freeze&&mode_e2==0;
 assign net_hold=photo_freeze||preview_hold;
 assign net_processed=(photo_freeze&&mode_e2!=0)||(PROCESSED_PREVIEW!=0&&preview_hold&&!photo_freeze);
 assign net_line_request=photo_freeze?line_req:preview_request;
 assign net_row=photo_freeze?photo_rows:preview_row;
 assign net_read_x=photo_freeze?line_addr:preview_x;
 rect_preview_stream #(.PROCESSED(PROCESSED_PREVIEW),.LAYOUT_TAG(1)) preview(.clk(eth_clk),.rst_n(eth_rst_n),.enable(preview_run),.session(preview_dst[31:0]),
  .hold_frame(preview_hold),.request_toggle(preview_request),.request_row(preview_row),.read_x(preview_x),
  .ready(net_ready),.done_toggle(net_line_done),.frame_id(net_frame_id),.frame_geometry(net_frame_geometry),.frame_model(net_frame_model),.pair_valid(net_pair_valid),.pixel(net_pixel),.pixel_next(net_pixel_next),.pixel_previous(net_pixel_previous),.pixel_previous_next(net_pixel_previous_next),
  .busy(preview_busy),.tx_en(preview_tx_en),.tx_data(preview_tx_data));
 ethernet_image_bridge #(.PHOTO_CHUNK_BYTES(1400),.PHOTO_COALESCE_TICKS(62500),.PHOTO_WINDOW(4)) image_ethernet(.sys_clk(clk),.sys_rst_n(rst_n),.rgmii_rxc(rgmii_rxc),.rgmii_rx_ctl(rgmii_rx_ctl),.rgmii_rxd(rgmii_rxd),
  .rgmii_txc(rgmii_txc),.rgmii_tx_ctl(rgmii_tx_ctl),.rgmii_txd(rgmii_txd),.phy_rstn(),.eth_clk(eth_clk),.eth_rst_n(eth_rst_n),.ready(eth_ready),.fault(eth_fault),.abort(eth_abort),
  .capture(capture_event),.capture_sequence(request_sequence),.ack_update(ack_dst[64]!=ack_seen),.ack(ack_dst[63:0]),
  .abort_capture(photo_abort),.image_start(image_start),.image_data(image_data),.image_busy(image_busy),.image_pending(image_pending),.image_packet_active(packet_active),.preview_tx_en(preview_tx_en),.preview_tx_data(preview_tx_data),.photo_packets(photo_packets));
 wire [127:0] photo_status_sys;
 wire [7:0] photo_flags={photo_abort,eth_fault,preview_run,capture_pending,photo_state};
 cdc_mailbox #(.WIDTH(128)) photo_diagnostics(.src_clk(eth_clk),.src_rst_n(mailbox_rst_n),.dst_clk(clk),.dst_rst_n(mailbox_rst_n),
  .src_data({request_sequence,photo_packets,5'd0,photo_rows,photo_aborts,photo_flags,5'd0,photo_reason,15'd0,raw_photo&&net_ready}),.dst_data(photo_status_sys));
 reg [31:0] capture_commands;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)capture_commands<=0;else if(image_request)capture_commands<=capture_commands+1'b1;
 // A control request is committed in pixel blanking and read back from there.
 reg [3:0] requested;wire [3:0] requested_pix;reg vs_d;
 reg [1:0] photo_hold_pix;
 always @(posedge pix_clk or negedge pix_rst_n)
  if(!pix_rst_n)photo_hold_pix<=0;else photo_hold_pix<={photo_hold_pix[0],freeze_request};
 always @(posedge clk or negedge rst_n)
  if(!rst_n)requested<=4'b1001;else if(control_update)requested<=control_flags[3:0];
 cdc_mailbox #(.WIDTH(4)) ctrl_cdc(.src_clk(clk),.src_rst_n(rst_n),.dst_clk(pix_clk),.dst_rst_n(rst_n),.src_data(requested),.dst_data(requested_pix));
 always @(posedge pix_clk or negedge pix_rst_n)begin
  if(!pix_rst_n)begin gamma_bypass<=0;hud_enable<=1;network_mode<=1;vs_d<=0;end
  else begin vs_d<=video_vs;if(video_vs&&!vs_d&&!scan_freeze_active&&!photo_hold_pix[1]){hud_enable,gamma_bypass,network_mode}<=requested_pix;end
 end
 reg [31:0] frames,errors,uptime_ms;reg [15:0] ms_div;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin frames<=0;errors<=0;uptime_ms<=0;ms_div<=0;end
  else begin
   if(attitude_valid||motion_valid||altitude_valid||(EXTENDED_IMU!=0&&(quaternion_valid||magnetic_valid)))frames<=frames+1'b1;
   if(imu_error)errors<=errors+1'b1;
   if(ms_div==CLK_HZ/1000-1)begin ms_div<=0;uptime_ms<=uptime_ms+1'b1;end else ms_div<=ms_div+1'b1;
  end
 end
 wire [31:0] validity=telemetry[607:576];
 reg seen_q,seen_mag;reg [31:0] age_q,age_mag;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin seen_q<=0;seen_mag<=0;age_q<=0;age_mag<=0;end
  else if(EXTENDED_IMU==0||!extra_ready)begin seen_q<=0;seen_mag<=0;age_q<=0;age_mag<=0;end
  else begin
   if(quaternion_valid)begin seen_q<=1;age_q<=0;end else if(age_q<2*CLK_HZ)age_q<=age_q+1'b1;
   if(magnetic_valid)begin seen_mag<=1;age_mag<=0;end else if(age_mag<2*CLK_HZ)age_mag<=age_mag+1'b1;
  end
 end
 wire fresh_q=EXTENDED_IMU!=0&&extra_ready&&seen_q&&age_q<2*CLK_HZ;
 wire fresh_mag=EXTENDED_IMU!=0&&extra_ready&&seen_mag&&age_mag<2*CLK_HZ;
 wire [0:0] text_sys;
 cdc_mailbox #(.WIDTH(1)) text_state(.src_clk(pix_clk),.src_rst_n(rst_n),.dst_clk(clk),.dst_rst_n(rst_n),.src_data(EXTENDED_IMU!=0&&camera_text_visible),.dst_data(text_sys));
 // Network readiness and pixel status are coherent and never fabricated as lens calibration.
 wire [18:0] status_pix={data_page_active,frame_underflow,scan_freeze_active,hud_enable,gamma_bypass,gamma_step,camera_fps_valid,camera_fps_bcd};
 wire [18:0] status_sys;
 wire [1:0] mode_sys;
 cdc_mailbox #(.WIDTH(2)) mode_readback(.src_clk(pix_clk),.src_rst_n(rst_n),.dst_clk(clk),.dst_rst_n(rst_n),.src_data(network_mode),.dst_data(mode_sys));
 cdc_mailbox #(.WIDTH(19)) status_cdc(.src_clk(pix_clk),.src_rst_n(rst_n),.dst_clk(clk),.dst_rst_n(rst_n),.src_data(status_pix),.dst_data(status_sys));
 reg [543:0] video_payload,cal_payload;
 integer i;
 always @*begin
  video_payload=0;cal_payload=0;
  // V5 keeps the 68-byte packet and all V4 lens/Gamma fields. Previously
  // unmeasured latency/repeat fields now carry explicit photo diagnostics.
  video_payload[7:0]=10;
  video_payload[55:48]={7'd0,status_sys[18]};
  video_payload[127:96]=capture_commands;
  video_payload[159:128]=photo_status_sys[127:96];
  video_payload[191:160]=photo_status_sys[95:64];
  video_payload[239:224]=photo_status_sys[63:48];
  video_payload[255:240]=photo_status_sys[47:32];
  video_payload[535:528]=photo_status_sys[31:24];
  video_payload[543:536]=photo_status_sys[23:16];
  video_payload[15:8]={3'd0,render_ready,3'd0,camera_ready} | (ddr_ready?8'd1:8'd0);
  video_payload[8]=ddr_ready;video_payload[9]=camera_ready;
  video_payload[23:16]={6'd0,mode_sys};video_payload[31:24]={7'd0,status_sys[14]};
  video_payload[47:40]={3'd0,status_sys[13:9]};
  video_payload[63:56]={4'd0,status_sys[15],status_sys[14],mode_sys};
  video_payload[95:64]=uptime_ms;
  video_payload[207:192]=status_sys[8]?((status_sys[7:4]*10)+status_sys[3:0]):0;
  video_payload[223:208]=pix_rst_n?16'd30:0;
  // A raw-frame lease is sufficient even if the renderer/HDMI scan is not
  // initialized. The diagnostic session and raw lease cross in ONE mailbox.
  video_payload[407:400]={7'd0,mode_sys==0?photo_status_sys[0]:status_sys[16]};
  video_payload[415:408]={7'd0,status_sys[15]};
  // Reserved extension signature R, capability bits and explicit live network status.
  video_payload[431:424]=8'h52;
  video_payload[439:432]={2'd0,1'b1,1'b1,1'b1,1'b1,net_ready_sys[1],1'b1};
  video_payload[447:440]={7'd0,status_sys[17]};
  if(EXTENDED_IMU!=0)video_payload[447:440]={4'd0,1'b1,1'b1,text_sys[0],status_sys[17]};
  video_payload[446]=1'b1; // negotiated four-packet cumulative photo window
  video_payload[455:448]={5'd0,lens_status[384],lens_status[385],lens_status[386]};
  video_payload[487:456]=lens_status[418:387];
  video_payload[519:488]=lens_errors;video_payload[527:520]={7'd0,lens_busy};
  cal_payload[7:0]=2;cal_payload[15:8]=1;cal_payload[23:16]={7'd0,lens_status[386]};
  cal_payload[31:24]={6'd0,lens_status[384],lens_status[385]};cal_payload[63:32]=lens_status[418:387];
  cal_payload[415:64]=lens_status[351:0];
  cal_payload[431:416]=16'd1920;cal_payload[447:432]=16'd1080;cal_payload[479:448]=lens_errors;
 end
 telemetry_packetizer #(.CLK_FREQ_HZ(CLK_HZ),.ENABLE_RESOURCES(RESOURCE_TELEMETRY),.FAST_IMU(EXTENDED_IMU),.ENABLE_GEOMETRY(1)) packets(.clk(clk),.rst_n(rst_n),.link_ready(1'b1),.stream_enable(stream_enable),
  .performance_request(performance_request),.performance_probe(performance_probe),
  .resource_addr(resource_addr),.resource_byte(resource_byte),
  .stream_period_ms(period_ms),.snapshot_request(snapshot_request),.video_payload(video_payload),.calibration_payload(cal_payload),.geometry_payload(geometry_payload),
  .valid_flags({validity[19],fresh_mag,validity[18],fresh_q,validity[17]}),
  .register_valid_flags({1'b0,validity[16],validity[15],1'b0}),.sensor_cal_status(8'd0),.gyro_fsr_code(gyro_range),.accel_fsr_code(acc_range),.return_rate_code(4'd0),
  .imu_uart_error(imu_error),.wifi_uart_error(rx_error),.roll_raw(roll),.pitch_raw(pitch),.yaw_raw(yaw),
  .q0_raw(EXTENDED_IMU!=0?q0:16'd0),.q1_raw(EXTENDED_IMU!=0?q1:16'd0),.q2_raw(EXTENDED_IMU!=0?q2:16'd0),.q3_raw(EXTENDED_IMU!=0?q3:16'd0),.accel_x_raw(ax),.accel_y_raw(ay),.accel_z_raw(az),
  .gyro_x_raw(gx),.gyro_y_raw(gy),.gyro_z_raw(gz),.mag_x_raw(EXTENDED_IMU!=0?mx:16'd0),.mag_y_raw(EXTENDED_IMU!=0?my:16'd0),.mag_z_raw(EXTENDED_IMU!=0?mz:16'd0),.mag_temp_raw(EXTENDED_IMU!=0?mag_temperature:16'd0),
  .pressure_raw(pressure),.altitude_raw(altitude),.barometer_temp_raw(temperature),.imu_frame_count(frames),.imu_checksum_error_count(errors),
  .command_count(command_count),.command_error_count(command_errors),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready));
endmodule
