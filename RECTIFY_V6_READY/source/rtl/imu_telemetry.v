`timescale 1ns/1ps
// MS901M units from the user manual. Signed rounding is symmetric about zero.
// No assumed gyro/accel full scale: numeric values stay invalid until readback.
module imu_telemetry #(parameter integer CLK_HZ=27000000)(
 input clk,rst_n,stream_rst_n,link_locked,attitude_valid,motion_valid,altitude_valid,
 input signed [15:0] roll_raw,pitch_raw,yaw_raw,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,
 input signed [31:0] pressure_pa,altitude_cm,
 input signed [15:0] temperature_centi,rotation_raw,
 input [1:0] gyro_range,acc_range,
 input gyro_known,acc_known,algorithm_known,algorithm_mag,
 input [31:0] diagnostic_status,
 input [31:0] video_debug,
 output [639:0] snapshot
);
 reg seen_a,seen_m,seen_b,zero_alt_valid;
 reg [31:0] age_a,age_m,age_b;
 reg signed [31:0] zero_alt;
 wire fresh_a=seen_a && age_a<2*CLK_HZ && link_locked && stream_rst_n;
 wire fresh_m=seen_m && age_m<2*CLK_HZ && link_locked && stream_rst_n;
 wire fresh_b=seen_b && age_b<2*CLK_HZ && link_locked && stream_rst_n;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   seen_a<=0;seen_m<=0;seen_b<=0;zero_alt_valid<=0;zero_alt<=0;
   age_a<=0;age_m<=0;age_b<=0;
  end else if(!stream_rst_n) begin
   // The parser clears raw registers on every link/baud restart. Its old
   // freshness must not make those cleared registers look like new reports.
   // Preserve the first height reference until a GLOBAL reset, as before.
   seen_a<=0;seen_m<=0;seen_b<=0;age_a<=0;age_m<=0;age_b<=0;
  end else begin
   if(attitude_valid) begin age_a<=0;seen_a<=1;end else if(age_a<2*CLK_HZ) age_a<=age_a+1'b1;
   if(motion_valid) begin age_m<=0;seen_m<=1;end else if(age_m<2*CLK_HZ) age_m<=age_m+1'b1;
   if(altitude_valid) begin
    age_b<=0;seen_b<=1;
    if(!zero_alt_valid) begin zero_alt<=altitude_cm;zero_alt_valid<=1;end
   end else if(age_b<2*CLK_HZ) age_b<=age_b+1'b1;
  end
 end
 function signed [31:0] angle10;
  input signed [15:0] value;
  reg [15:0] mag;reg [31:0] product,result;
  begin mag=value[15]?-value:value;product=mag*32'd1800;
   result=(product+16384)>>15;angle10=value[15]?-$signed(result):$signed(result);end
 endfunction
 function signed [31:0] rate10;
  input signed [15:0] value;input [1:0] fs;
  reg [15:0] mag;reg [31:0] product,result;
  begin mag=value[15]?-value:value;product=(mag*32'd2500)<<fs;
   result=(product+16384)>>15;rate10=value[15]?-$signed(result):$signed(result);end
 endfunction
 function signed [31:0] accel1000;
  input signed [15:0] value;input [1:0] fs;
  reg [15:0] mag;reg [31:0] product,result;
  begin mag=value[15]?-value:value;product=(mag*32'd2000)<<fs;
   result=(product+16384)>>15;accel1000=value[15]?-$signed(result):$signed(result);end
 endfunction
 wire signed [31:0] yaw10=angle10(yaw_raw);
 wire [31:0] heading10=(yaw10<0)?yaw10+3600:yaw10;
 wire signed [32:0] relative_alt=$signed({altitude_cm[31],altitude_cm})-$signed({zero_alt[31],zero_alt});
 wire relative_ok=relative_alt[32]==relative_alt[31];
 wire [575:0] values={video_debug,(32'd2<<acc_range),(32'd250<<gyro_range),angle10(rotation_raw),
  {{16{temperature_centi[15]}},temperature_centi},pressure_pa,relative_alt[31:0],altitude_cm,
  accel1000(az_raw,acc_range),accel1000(ay_raw,acc_range),accel1000(ax_raw,acc_range),
  rate10(gz_raw,gyro_range),rate10(gy_raw,gyro_range),rate10(gx_raw,gyro_range),
  angle10(yaw_raw),angle10(pitch_raw),angle10(roll_raw),heading10};
 reg [31:0] valid;
 always @* begin
  valid=0;valid[3:0]={4{fresh_a}};valid[6:4]={3{fresh_m && gyro_known}};
  valid[9:7]={3{fresh_m && acc_known}};valid[10]=fresh_b;
  valid[11]=fresh_b && zero_alt_valid && relative_ok;valid[13:12]={2{fresh_b}};
  valid[14]=fresh_a;valid[15]=gyro_known && stream_rst_n;valid[16]=acc_known && stream_rst_n;
  valid[17]=fresh_a;valid[18]=fresh_m;valid[19]=fresh_b;
 end
 wire [31:0] status=(diagnostic_status & 32'hfffff9ff) |
  (algorithm_known?(algorithm_mag?32'h00000200:32'd0):32'h00000400);
 assign snapshot={status,valid,values};
endmodule
