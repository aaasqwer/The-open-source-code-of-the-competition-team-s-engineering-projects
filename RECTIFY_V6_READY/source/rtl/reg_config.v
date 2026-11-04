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
//camera中寄存器的配置程序
 module reg_config #(
          parameter integer CLOCK_DIVIDER=1250,parameter integer LIGHT_FREQ_HZ=50,
          parameter integer ENABLE_LIGHT_METER=0,
          parameter integer SENSOR_PROFILE=0,parameter SENSOR_TEST_PATTERN=0,
          parameter integer SENSOR_XCLK_HZ=24000000,
          parameter integer RETRY_FOREVER=(SENSOR_PROFILE==1),
          parameter integer WAIT_FIRST_FRAME=ENABLE_LIGHT_METER,
          parameter integer LOCK_EXPOSURE=1,parameter integer EXPOSURE_SETTLE_TICKS=19800)(
		  input clk_25M,
		  input camera_rstn,
		  input initial_en,
		  output reg_conf_done,
          output reg reg_conf_error,
          output reg exposure_locked,
		  output i2c_sclk,
		  inout i2c_sdat,
		  output reg clock_20k,
		  output reg [8:0]reg_index,
          input camera_vs,
          output light_valid,
          output [7:0] light_index,light_error,
          output [15:0] exposure_lines,
          output [11:0] light_metric_q4,
          output [9:0] gain_q4
	  );

     reg [15:0]clock_20k_cnt;
     reg [1:0]config_step;	  
     reg [31:0]i2c_data;
     reg [23:0]imu_reg_data;
     wire [23:0] gamma_reg_data;
     wire [23:0] reg_data=(SENSOR_PROFILE==1)?gamma_reg_data:imu_reg_data;
     localparam CONFIG_END=(SENSOR_PROFILE==1)?363:366;
     generate if(SENSOR_PROFILE==1) begin:gamma_profile
       camera_gamma_table #(.SENSOR_TEST_PATTERN(SENSOR_TEST_PATTERN),.SENSOR_XCLK_HZ(SENSOR_XCLK_HZ)) table_rom(
         .reg_index(reg_index),.reg_data(gamma_reg_data));
     end else begin:imu_profile
       assign gamma_reg_data=0;
     end endgenerate
     // A completed configuration write does not prove the camera is producing
     // frames. Start the selected camera's warm-up after an observed VS edge.
     reg camera_vs_meta,camera_vs_sync,camera_vs_d,first_frame_seen;
     reg first_frame_meta,first_frame_ready;
     always @(posedge clk_25M or negedge camera_rstn)
       if(!camera_rstn) begin camera_vs_meta<=0;camera_vs_sync<=0;camera_vs_d<=0;first_frame_seen<=0;end
       else begin
         camera_vs_meta<=camera_vs;camera_vs_sync<=camera_vs_meta;camera_vs_d<=camera_vs_sync;
         if(reg_conf_done_reg && camera_vs_sync && !camera_vs_d) first_frame_seen<=1;
       end
     always @(posedge clock_20k or negedge camera_rstn)
       if(!camera_rstn) begin first_frame_meta<=0;first_frame_ready<=0;end
       else begin first_frame_meta<=first_frame_seen;first_frame_ready<=first_frame_meta;end
     reg start;
     wire ack,tr_end;
     reg [1:0] retries;
     reg [31:0] exposure_settle;
	 reg reg_conf_done_reg;
	  
     // Configuration and the optional exposure-lock write own SCCB first.
     // Readback starts only after their STOP/ACK, and never issues writes.
     wire config_scl, telemetry_scl;
     wire meter_owns_bus=ENABLE_LIGHT_METER && reg_conf_done_reg &&
                         !reg_conf_error && (!LOCK_EXPOSURE || exposure_locked);
     assign i2c_sclk=meter_owns_bus ? telemetry_scl : config_scl;
     generate if(ENABLE_LIGHT_METER) begin:light_meter_enabled
       ov5640_light_meter #(.CLK_HZ(24750000),.EXPECTED_FORMAT((SENSOR_PROFILE==1)?8'h61:8'h60),
           .EXPECTED_ISP((SENSOR_PROFILE==1)?8'h03:8'h83),.EXPECTED_AEC_AGC(LOCK_EXPOSURE?3:0)) meter(
         .clk(clk_25M),.rstn(camera_rstn),.enable(meter_owns_bus),.camera_vs(camera_vs),
         .scl(telemetry_scl),.sda(i2c_sdat),.valid(light_valid),
         .light_index(light_index),.sensor_average(),.error_code(light_error),
         .exposure_lines(exposure_lines),.gain_q4(gain_q4),.light_metric_q4(light_metric_q4));
     end else begin:light_meter_disabled
       assign telemetry_scl=1'b1;
       assign light_valid=0;assign light_index=128;assign light_error=8'h80;
       assign exposure_lines=0;assign gain_q4=0;assign light_metric_q4=0;
     end endgenerate

     i2c_com u1(.clock_i2c(clock_20k),
               .camera_rstn(camera_rstn),
               .ack(ack),
               .i2c_data(i2c_data),
               .start(start),
               .tr_end(tr_end),
               .i2c_sclk(config_scl),
               .i2c_sdat(i2c_sdat));

assign reg_conf_done=reg_conf_done_reg;
// clk_25M/2500 = approximately 10 kHz SCCB clock.
always@(posedge clk_25M or negedge camera_rstn)   
begin
   if(!camera_rstn) begin
        clock_20k<=0;
        clock_20k_cnt<=0;
   end
   else if(!initial_en) begin clock_20k<=0;clock_20k_cnt<=0;end
   else if(clock_20k_cnt<CLOCK_DIVIDER-1)
      clock_20k_cnt<=clock_20k_cnt+1'b1;
   else begin
         clock_20k<=!clock_20k;
         clock_20k_cnt<=0;
   end
end


// A missing camera/NACK is a failure, never a successful configuration.
always @(posedge clock_20k or negedge camera_rstn) begin
 if(!camera_rstn) begin
  config_step<=0;start<=0;reg_index<=0;reg_conf_done_reg<=0;reg_conf_error<=0;retries<=0;i2c_data<=0;
  exposure_locked<=0;exposure_settle<=0;
 end else if(!reg_conf_done_reg && !reg_conf_error) begin
  if(reg_index>=CONFIG_END) begin reg_conf_done_reg<=1;config_step<=0;end
  else if(reg_index>=2 && reg_index<=101) reg_index<=reg_index+1'b1; // reset settling delay; no writes to FFFF
  else case(config_step)
   0:begin i2c_data<={8'h78,reg_data};start<=1;config_step<=1;end
   1:if(tr_end) begin
    start<=0;
    if(!ack) begin retries<=0;config_step<=2;end
    else if(retries==3 && !RETRY_FOREVER) reg_conf_error<=1;
    else begin retries<=retries+1'b1;config_step<=3;end
   end
   2:begin reg_index<=reg_index+1'b1;config_step<=0;end
   3:if(!tr_end) config_step<=0;
  endcase
 end else if(reg_conf_done_reg && LOCK_EXPOSURE && !exposure_locked && !reg_conf_error) begin
  // Let AEC/AGC converge for 19800 / 9900 = 2 s after stream-on.
  // OV5640 0x3503[1:0]=3 holds the current exposure/gain output registers.
  // Do not write guessed shutter/gain values or restart the sensor stream.
  // Keep reg_conf_done asserted: the pixel packer must not be reset here.
  if(WAIT_FIRST_FRAME && !first_frame_ready) exposure_settle<=0;
  else if(exposure_settle<EXPOSURE_SETTLE_TICKS) exposure_settle<=exposure_settle+1'b1;
  else case(config_step)
   0:begin i2c_data<=32'h78350303;start<=1;config_step<=1;end
   1:if(tr_end) begin
    start<=0;
    if(!ack) begin retries<=0;exposure_locked<=1;end
    else if(retries==3 && !RETRY_FOREVER) reg_conf_error<=1;
    else begin retries<=retries+1'b1;config_step<=3;end
   end
   3:if(!tr_end) config_step<=0;
  endcase
 end
end

always@(reg_index)   
 begin
    case(reg_index)
	 0    :imu_reg_data   <=24'h310311 ;//      
	 1    :imu_reg_data   <=24'h300882 ;//      

	 102  :imu_reg_data   <=24'h300842 ;//      
	 103  :imu_reg_data   <=24'h310303 ;//      
	 104  :imu_reg_data   <=24'h3017ff ;//      
	 105  :imu_reg_data   <=24'h3018ff ;//      
	 106  :imu_reg_data   <=24'h303418 ;//      
	 107  :imu_reg_data   <=24'h303713 ;//      
	 108  :imu_reg_data   <=24'h310801 ;//      
	 109  :imu_reg_data   <=24'h363036 ;//      
	 110  :imu_reg_data  <=24'h36310e ;//       
	 111  :imu_reg_data  <=24'h3632e2 ;//       
	 112  :imu_reg_data  <=24'h363312 ;//       
	 113  :imu_reg_data  <=24'h3621e0 ;//       
	 114  :imu_reg_data  <=24'h3704a0 ;//       
	 115  :imu_reg_data  <=24'h37035a ;//       
	 116  :imu_reg_data  <=24'h371578 ;//       
	 117  :imu_reg_data  <=24'h371701 ;//       
	 118  :imu_reg_data  <=24'h370b60 ;//       
	 119  :imu_reg_data  <=24'h37051a ;//       
	 120  :imu_reg_data  <=24'h390502 ;//       
	 121  :imu_reg_data  <=24'h390610 ;//       
	 122  :imu_reg_data  <=24'h39010a ;//       
	 123  :imu_reg_data  <=24'h373112 ;//       
	 124  :imu_reg_data  <=24'h360008 ;//       
	 125  :imu_reg_data  <=24'h360133 ;//       
	 126  :imu_reg_data  <=24'h302d60 ;//       
	 127  :imu_reg_data  <=24'h362052 ;//       
	 128  :imu_reg_data  <=24'h371b20 ;//       
	 129  :imu_reg_data  <=24'h471c50 ;//       
	 130  :imu_reg_data  <=24'h3a1343 ;//       
	 131  :imu_reg_data  <=24'h3a1800 ;//       
	 132  :imu_reg_data  <=24'h3a19f8 ;//       
	 133  :imu_reg_data  <=24'h363513 ;//       
	 134  :imu_reg_data  <=24'h363603 ;//       
	 135  :imu_reg_data  <=24'h363440 ;//       
	 136  :imu_reg_data  <=24'h362201 ;///      
	 137  :imu_reg_data  <=24'h3c0134 ;//       
	 138  :imu_reg_data  <=24'h3c0428 ;//       
	 139  :imu_reg_data  <=24'h3c0598 ;//       
	 140  :imu_reg_data  <=24'h3c0600 ;//       
     141  :imu_reg_data  <=24'h3c0708 ;//       
	 142  :imu_reg_data  <=24'h3c0800 ;//       
	 143  :imu_reg_data  <=24'h3c091c ;//       
	 144  :imu_reg_data  <=24'h3c0a9c ;//       
	 145  :imu_reg_data  <=24'h3c0b40 ;//       
	 146  :imu_reg_data  <=24'h381000 ;//       
	 147  :imu_reg_data  <=24'h381110 ;//       
	 148  :imu_reg_data  <=24'h381200 ;//       
	 149  :imu_reg_data  <=24'h370864 ;//       
	 150  :imu_reg_data  <=24'h400102 ;//       
	 151  :imu_reg_data  <=24'h40051a ;//       
	 152  :imu_reg_data  <=24'h300000 ;//       
	 153  :imu_reg_data  <=24'h3004ff ;//       
	 154  :imu_reg_data  <=24'h300e58 ;//       
	 155  :imu_reg_data  <=24'h302e00 ;//       
	 156  :imu_reg_data  <=24'h430060 ;//       
	 157  :imu_reg_data  <=24'h501f01 ;//       
	 158  :imu_reg_data  <=24'h440e00 ;//       
	 159  :imu_reg_data  <=24'h5000a7 ;//     
	 160  :imu_reg_data  <=24'h3a0f30 ;//       
	 161  :imu_reg_data  <=24'h3a1028 ;//       
	 162  :imu_reg_data  <=24'h3a1b30 ;//       
	 163  :imu_reg_data  <=24'h3a1e26 ;//       
	 164  :imu_reg_data  <=24'h3a1160 ;//       
	 165  :imu_reg_data  <=24'h3a1f14 ;//       
	 166  :imu_reg_data  <=24'h580023 ;//       
	 167  :imu_reg_data  <=24'h580114 ;//       
	 168  :imu_reg_data  <=24'h58020f ;//       
	 169  :imu_reg_data  <=24'h58030f ;//       
	 170  :imu_reg_data  <=24'h580412 ;//       
	 171  :imu_reg_data  <=24'h580526 ;//       
	 172  :imu_reg_data  <=24'h58060c ;//       
	 173  :imu_reg_data  <=24'h580708 ;//       
	 174  :imu_reg_data  <=24'h580805 ;//       
	 175  :imu_reg_data  <=24'h580905 ;//       
	 176  :imu_reg_data  <=24'h580a08 ;//       
	 177  :imu_reg_data  <=24'h580b0d ;//       
	 178  :imu_reg_data  <=24'h580c08 ;//       
	 179  :imu_reg_data  <=24'h580d03 ;//       
	 180  :imu_reg_data  <=24'h580e00 ;//       
	 181  :imu_reg_data  <=24'h580f00 ;//       
	 182  :imu_reg_data  <=24'h581003 ;//       
	 183  :imu_reg_data  <=24'h581109 ;//       
	 184  :imu_reg_data  <=24'h581207 ;//       
	 185  :imu_reg_data  <=24'h581303 ;//       
	 186  :imu_reg_data  <=24'h581400 ;//       
	 187  :imu_reg_data  <=24'h581501 ;//       
	 188  :imu_reg_data  <=24'h581603 ;//       
	 189  :imu_reg_data  <=24'h581708 ;//       
	 190  :imu_reg_data  <=24'h58180d ;//       
	 191  :imu_reg_data  <=24'h581908 ;//       
	 192  :imu_reg_data  <=24'h581a05 ;//       
	 193  :imu_reg_data  <=24'h581b06 ;//       
	 194  :imu_reg_data  <=24'h581c08 ;//       
	 195  :imu_reg_data  <=24'h581d0e ;//       
	 196  :imu_reg_data  <=24'h581e29 ;//       
	 197  :imu_reg_data  <=24'h581f17 ;//       
	 198  :imu_reg_data  <=24'h582011 ;//       
	 199  :imu_reg_data  <=24'h582111 ;//       
	 200  :imu_reg_data <=24'h582215 ;//        
	 201  :imu_reg_data <=24'h582328 ;//        
	 202  :imu_reg_data <=24'h582446 ;//        
	 203  :imu_reg_data <=24'h582526 ;//        
	 204  :imu_reg_data <=24'h582608 ;//        
	 205  :imu_reg_data <=24'h582726 ;//        
	 206  :imu_reg_data <=24'h582864 ;//        
	 207  :imu_reg_data <=24'h582926 ;//        
	 208  :imu_reg_data <=24'h582a24 ;//        
	 209  :imu_reg_data <=24'h582b22 ;//        
	 210  :imu_reg_data <=24'h582c24 ;//        
	 211  :imu_reg_data <=24'h582d24 ;//        
	 212  :imu_reg_data <=24'h582e06 ;//        
	 213  :imu_reg_data <=24'h582f22 ;//        
	 214  :imu_reg_data <=24'h583040 ;//        
	 215  :imu_reg_data <=24'h583142 ;//        
	 216  :imu_reg_data <=24'h583224 ;//        
	 217  :imu_reg_data <=24'h583326 ;//        
	 218  :imu_reg_data <=24'h583424 ;//        
	 219  :imu_reg_data <=24'h583522 ;//        
	 220  :imu_reg_data <=24'h583622 ;//        
	 221  :imu_reg_data <=24'h583726 ;//        
	 222  :imu_reg_data <=24'h583844 ;//        
	 223  :imu_reg_data <=24'h583924 ;//        
	 224  :imu_reg_data <=24'h583a26 ;//        
	 225  :imu_reg_data <=24'h583b28 ;//        
	 226  :imu_reg_data <=24'h583c42 ;//        
	 227  :imu_reg_data <=24'h583dce ;//        
	 228  :imu_reg_data <=24'h5180ff ;//        
	 229  :imu_reg_data <=24'h5181f2 ;//        
	 230  :imu_reg_data <=24'h518200 ;//        
	 231  :imu_reg_data <=24'h518314 ;//        
	 232  :imu_reg_data <=24'h518425 ;//        
	 233  :imu_reg_data <=24'h518524 ;//        
	 234  :imu_reg_data <=24'h518609 ;//        
	 235  :imu_reg_data <=24'h518709 ;//        
	 236  :imu_reg_data <=24'h518809 ;//        
	 237  :imu_reg_data <=24'h518975 ;//        
	 238  :imu_reg_data <=24'h518a54 ;//        
	 239  :imu_reg_data <=24'h518be0 ;//        
	 240  :imu_reg_data <=24'h518cb2 ;//        
	 241  :imu_reg_data <=24'h518d42 ;//        
	 242  :imu_reg_data <=24'h518e3d ;//        
	 243  :imu_reg_data <=24'h518f56 ;//        
	 244  :imu_reg_data <=24'h519046 ;//        
	 245  :imu_reg_data <=24'h5191f8 ;//        
	 246  :imu_reg_data <=24'h519204 ;//        
	 247  :imu_reg_data <=24'h519370 ;//        
	 248  :imu_reg_data <=24'h5194f0 ;//        
	 249  :imu_reg_data <=24'h5195f0 ;//        
	 250  :imu_reg_data <=24'h519603 ;//        
	 251  :imu_reg_data <=24'h519701 ;//        
	 252  :imu_reg_data <=24'h519804 ;//        
	 253  :imu_reg_data <=24'h519912 ;//        
	 254  :imu_reg_data <=24'h519a04 ;//        
	 255  :imu_reg_data <=24'h519b00 ;//        
	 256  :imu_reg_data <=24'h519c06 ;//        
	 257  :imu_reg_data <=24'h519d82 ;//        
	 258  :imu_reg_data <=24'h519e38 ;//        
	 259  :imu_reg_data <=24'h548001 ;//        
	 260  :imu_reg_data <=24'h548108 ;//        
	 261  :imu_reg_data <=24'h548214 ;//        
	 262  :imu_reg_data <=24'h548328 ;//        
	 263  :imu_reg_data <=24'h548451 ;//        
	 264  :imu_reg_data <=24'h548565 ;//        
	 265  :imu_reg_data <=24'h548671 ;//        
	 266  :imu_reg_data <=24'h54877d ;//        
	 267  :imu_reg_data <=24'h548887 ;//        
	 268  :imu_reg_data <=24'h548991 ;//        
	 269  :imu_reg_data <=24'h548a9a ;//        
	 270  :imu_reg_data <=24'h548baa ;//        
	 271  :imu_reg_data <=24'h548cb8 ;//        
	 272  :imu_reg_data <=24'h548dcd ;//        
	 273  :imu_reg_data <=24'h548edd ;//        
	 274  :imu_reg_data <=24'h548fea ;//        
	 275  :imu_reg_data <=24'h54901d ;//        
	 276  :imu_reg_data <=24'h53811e ;//        
	 277  :imu_reg_data <=24'h53825b ;//        
	 278  :imu_reg_data <=24'h538308 ;//        
	 279  :imu_reg_data <=24'h53840a ;//        
	 280  :imu_reg_data <=24'h53857e ;//        
	 281  :imu_reg_data <=24'h538688 ;//        
	 282  :imu_reg_data <=24'h53877c ;//        
	 283  :imu_reg_data <=24'h53886c ;//        
	 284  :imu_reg_data <=24'h538910 ;//        
	 285  :imu_reg_data <=24'h538a01 ;//        
	 286  :imu_reg_data <=24'h538b98 ;//       
	 287  :imu_reg_data <=24'h558006 ;//        
	 288  :imu_reg_data <=24'h558340 ;//        
	 289  :imu_reg_data <=24'h558410 ;//        
	 290  :imu_reg_data <=24'h558910 ;//        
	 291  :imu_reg_data <=24'h558a00 ;//        
	 292  :imu_reg_data <=24'h558bf8 ;//        
	 293  :imu_reg_data <=24'h501d40 ;//        
	 294  :imu_reg_data <=24'h530008 ;//        
	 295  :imu_reg_data <=24'h530130 ;//        
	 296  :imu_reg_data <=24'h530210 ;//        
	 297  :imu_reg_data <=24'h530300 ;//        
	 298  :imu_reg_data <=24'h530408 ;//        
	 299  :imu_reg_data <=24'h530530 ;//        
	 300  :imu_reg_data <=24'h530608 ;//        
	 301  :imu_reg_data <=24'h530716 ;//        
	 302  :imu_reg_data <=24'h530908 ;//        
	 303  :imu_reg_data <=24'h530a30 ;//        
	 304  :imu_reg_data <=24'h530b04 ;//        
	 305  :imu_reg_data <=24'h530c06 ;//        
	 306  :imu_reg_data <=24'h502500 ;//        
	 307  :imu_reg_data <=24'h300842 ;//       
  // OV5640 DVP RGB565 1920x1080 target 15fps; HDMI remains 30Hz; XCLK is module-provided (24MHz reference)
	 308  :imu_reg_data <=24'h300842 ;// software standby while changing mode
	 309  :imu_reg_data <=24'h303520 ;// system clock divider
     310  :imu_reg_data <=24'h303654 ;// PLL multiplier
	 311  :imu_reg_data <=24'h3c0707 ;// light-meter clock
	 312  :imu_reg_data <=24'h382041 ;// vertical orientation / full-resolution path
	 313  :imu_reg_data <=24'h382107 ;// horizontal orientation / full-resolution path
	 314  :imu_reg_data <=24'h381411 ;// horizontal increment
	 315  :imu_reg_data <=24'h381511 ;// vertical increment
	 316  :imu_reg_data <=24'h380001 ;// crop X start high
	 317  :imu_reg_data <=24'h380150 ;// crop X start low
	 318  :imu_reg_data <=24'h380201 ;// crop Y start high
	 319  :imu_reg_data <=24'h3803b2 ;// crop Y start low
	 320  :imu_reg_data <=24'h380408 ;// crop X end high
	 321  :imu_reg_data <=24'h3805ef ;// crop X end low
	 322  :imu_reg_data <=24'h380605 ;// crop Y end high
	 323  :imu_reg_data <=24'h3807f1 ;// crop Y end low
	 324  :imu_reg_data <=24'h380807 ;// output width high: 1920
	 325  :imu_reg_data <=24'h380980 ;// output width low
	 326  :imu_reg_data <=24'h380a04 ;// output height high: 1080
	 327  :imu_reg_data <=24'h380b38 ;// output height low
	 328  :imu_reg_data <=24'h380c09 ;// HTS high: 2500
	 329  :imu_reg_data <=24'h380dc4 ;// HTS low
	 330  :imu_reg_data <=24'h380e04 ;// VTS high: 1120
	 331  :imu_reg_data <=24'h380f60 ;// VTS low
	 332  :imu_reg_data <=24'h381110 ;// ISP horizontal offset
	 333  :imu_reg_data <=24'h381304 ;// ISP vertical offset
	 334  :imu_reg_data <=24'h361804 ;// analog control
	 335  :imu_reg_data <=24'h36122b ;// analog control
	 336  :imu_reg_data <=24'h370864 ;// sensor timing control
	 337  :imu_reg_data <=24'h3a0204 ;// AEC max exposure high
	 338  :imu_reg_data <=24'h3a0360 ;// AEC max exposure low
	 339  :imu_reg_data <=24'h3a0801 ;// 50Hz banding step high
	 340  :imu_reg_data <=24'h3a0950 ;// 50Hz banding step low
	 341  :imu_reg_data <=24'h3a0a01 ;// 60Hz banding step high
	 342  :imu_reg_data <=24'h3a0b18 ;// 60Hz banding step low
	 343  :imu_reg_data <=24'h3a0e03 ;// 50Hz max bands
	 344  :imu_reg_data <=24'h3a0d04 ;// 60Hz max bands
	 345  :imu_reg_data <=24'h3a1404 ;// AEC max exposure high (night)
	 346  :imu_reg_data <=24'h3a1560 ;// AEC max exposure low (night)
	 347  :imu_reg_data <=24'h400102 ;// BLC control
	 348  :imu_reg_data <=24'h400406 ;// BLC line number
	 349  :imu_reg_data <=24'h471302 ;// JPEG/DVP mode select
	 350  :imu_reg_data <=24'h440704 ;// quantization scale
	 351  :imu_reg_data <=24'h460b37 ;// DVP timing
     352  :imu_reg_data <=24'h460c20 ;// DVP PCLK divider
	 353  :imu_reg_data <=24'h382404 ;// PCLK divider
	 354  :imu_reg_data <=24'h40051a ;// BLC control for 1080p
	 355  :imu_reg_data <=24'h500183 ;// ISP enable; keep sensor in standby
	 356  :imu_reg_data <=24'h350300 ;// enable automatic exposure/gain
	 357  :imu_reg_data <=24'h430060; // RGB565
     358  :imu_reg_data <=24'h501f01; // RGB ISP
     359  :imu_reg_data <=24'h370912;
     360  :imu_reg_data <=24'h370c00;
     361  :imu_reg_data <={16'h503d,SENSOR_TEST_PATTERN?8'h80:8'h00}; // normal image, no test pattern
     // OV5640 datasheet 4.8/table 4-8: 3C01[7] manual frequency, 3C00[2] 50 Hz.
     // Keep automatic exposure/gain and the existing exposure limits.
     362  :imu_reg_data <=(LIGHT_FREQ_HZ==50 || LIGHT_FREQ_HZ==60)?24'h3c01b4:24'h3c0134;
     363  :imu_reg_data <=(LIGHT_FREQ_HZ==50)?24'h3c0004:24'h3c0000;
     364  :imu_reg_data <=24'h3a0078; // band filter enabled, night-mode frame extension disabled
     365  :imu_reg_data <=24'h300802; // stream after complete mode/light configuration
     default:imu_reg_data<=24'hffffff;//        
    endcase      
end	 



endmodule
