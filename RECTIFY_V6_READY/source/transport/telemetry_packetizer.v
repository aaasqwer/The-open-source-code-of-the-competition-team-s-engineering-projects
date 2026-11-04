`timescale 1ns/1ps
// AA 55 TYPE 44 PAYLOAD[68] SUM8; types 81 IMU,82 video,83 calibration.
module telemetry_packetizer #(
    parameter integer CLK_FREQ_HZ = 27000000,ENABLE_RESOURCES=0,FAST_IMU=0,ENABLE_GEOMETRY=0
)(
    input performance_request,input [511:0] performance_probe,
    input wire [543:0] video_payload,calibration_payload,geometry_payload,
    input [7:0] resource_byte,output [6:0] resource_addr,
    input  wire        clk,
    input  wire        rst_n,
    input  wire        link_ready,
    input  wire        stream_enable,
    input  wire [15:0] stream_period_ms,
    input  wire        snapshot_request,

    input  wire [4:0]  valid_flags,
    input  wire [3:0]  register_valid_flags,
    input  wire [7:0]  sensor_cal_status,
    input  wire [1:0]  gyro_fsr_code,
    input  wire [1:0]  accel_fsr_code,
    input  wire [3:0]  return_rate_code,
    input  wire        imu_uart_error,
    input  wire        wifi_uart_error,
    input  wire [15:0] roll_raw,
    input  wire [15:0] pitch_raw,
    input  wire [15:0] yaw_raw,
    input  wire [15:0] q0_raw,
    input  wire [15:0] q1_raw,
    input  wire [15:0] q2_raw,
    input  wire [15:0] q3_raw,
    input  wire [15:0] accel_x_raw,
    input  wire [15:0] accel_y_raw,
    input  wire [15:0] accel_z_raw,
    input  wire [15:0] gyro_x_raw,
    input  wire [15:0] gyro_y_raw,
    input  wire [15:0] gyro_z_raw,
    input  wire [15:0] mag_x_raw,
    input  wire [15:0] mag_y_raw,
    input  wire [15:0] mag_z_raw,
    input  wire [15:0] mag_temp_raw,
    input  wire [31:0] pressure_raw,
    input  wire [31:0] altitude_raw,
    input  wire [15:0] barometer_temp_raw,
    input  wire [31:0] imu_frame_count,
    input  wire [31:0] imu_checksum_error_count,
    input  wire [31:0] command_count,
    input  wire [31:0] command_error_count,

    output wire [7:0]  tx_data,
    output wire        tx_valid,
    input  wire        tx_ready
);


 localparam MS_DIVISOR=CLK_FREQ_HZ/1000;
 reg [31:0] divider;
 reg [15:0] elapsed,sequence_counter;
 reg [2:0] phase;
 reg performance_pending;
 reg pending,busy;
 reg [6:0] index;
 reg [7:0] checksum,kind;
 reg [543:0] payload;
 wire tick=divider==MS_DIVISOR-1;
 wire [15:0] period=(stream_period_ms<10)?16'd10:(stream_period_ms>5000?16'd5000:stream_period_ms);
 wire due=tick&&stream_enable&&elapsed>=period-1'b1;
 assign tx_valid=busy;
 assign resource_addr=(busy&&tx_ready)?(index<3?0:index>=71?67:index-3):(index<4?0:index>=72?67:index-4);
 assign tx_data=index==0 ? 8'haa : index==1 ? 8'h55 : index==2 ? kind : index==3 ? 8'h44 : index==72 ? checksum : (kind==8'h84 ? resource_byte : payload[(index-4)*8+:8]);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin performance_pending<=0;divider<=0;elapsed<=0;phase<=0;pending<=0;busy<=0;index<=0;checksum<=0;sequence_counter<=0;payload<=0;kind<=8'h81;end
  else begin
   if(tick)divider<=0;else divider<=divider+1'b1;
   if(!stream_enable||due)elapsed<=0;else if(tick)elapsed<=elapsed+1'b1;
   if(performance_request)performance_pending<=1;
   if(snapshot_request||due)pending<=1;
   // Type86 is emitted only on explicit command60; the original stream cadence
   // and all image transport packets are unchanged when no query is issued.
   if(!busy&&link_ready&&(performance_pending||performance_request))begin
    performance_pending<=0;busy<=1;index<=0;checksum<=0;kind<=8'h86;payload<={32'd1,performance_probe};
   end else if(!busy&&link_ready&&(pending||snapshot_request||due))begin
    pending<=0;busy<=1;index<=0;checksum<=0;sequence_counter<=sequence_counter+1'b1;
    if(phase==0||(FAST_IMU!=0&&phase==2))begin kind<=8'h81;payload<={command_error_count,command_count,imu_checksum_error_count,imu_frame_count,
      barometer_temp_raw,altitude_raw,pressure_raw,mag_temp_raw,mag_z_raw,mag_y_raw,mag_x_raw,
      gyro_z_raw,gyro_y_raw,gyro_x_raw,accel_z_raw,accel_y_raw,accel_x_raw,
      q3_raw,q2_raw,q1_raw,q0_raw,yaw_raw,pitch_raw,roll_raw,sequence_counter,
      4'd0,return_rate_code,6'd0,accel_fsr_code,6'd0,gyro_fsr_code,sensor_cal_status,
      4'd0,register_valid_flags,link_ready,wifi_uart_error,imu_uart_error,valid_flags};phase<=phase+1'b1;end
    else if(phase==1)begin kind<=8'h82;payload<=video_payload;phase<=2;end
    else if(phase==(FAST_IMU!=0?3:2))begin kind<=8'h83;payload<=calibration_payload;phase<=ENABLE_RESOURCES!=0?(FAST_IMU!=0?4:3):ENABLE_GEOMETRY!=0?(FAST_IMU!=0?5:4):0;end
    else if(ENABLE_RESOURCES!=0&&phase==(FAST_IMU!=0?4:3))begin kind<=8'h84;phase<=ENABLE_GEOMETRY!=0?(FAST_IMU!=0?5:4):0;end
    else begin kind<=8'h85;payload<=geometry_payload;phase<=0;end
   end else if(busy&&tx_ready)begin
    if(index<72)checksum<=checksum+tx_data;
    if(index==72)begin busy<=0;index<=0;
     // Insert the static report in the idle gap, not in place of the next IMU deadline.
     if(ENABLE_RESOURCES!=0&&kind==8'h83)pending<=1;
    end else index<=index+1'b1;
   end
  end
 end
endmodule
