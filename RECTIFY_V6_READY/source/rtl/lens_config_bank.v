`timescale 1ns/1ps
// Staging and CRC32 commit. No partial calibration reaches another clock domain.
// Parameters: fx,fy,cx,cy (Q16), k1,k2,p1,p2 (Q24), 1/fx,1/fy (Q30),
// inverse zoom (Q20), enable (0/1). Resolution must match the active camera mode.
`include "calibration/profile_config.vh"
module lens_config_bank #(parameter W=1920,H=1080)(
 input clk,rst_n,write_param,input [3:0] param_index,input [31:0] param_value,
 input commit,clear,input [31:0] request_id,request_crc,
 output reg [416:0] request_bundle,output reg [31:0] errors,output busy
);
 reg [31:0] staging[0:11];reg [11:0] written;
 reg checking;reg [5:0] byte_index;reg [31:0] crc,expected_crc,pending_id;
 wire [7:0] current_byte=staging[byte_index[5:2]]>>(byte_index[1:0]*8);
 function [31:0] crcbyte;
  input [31:0] old;input [7:0] value;reg [31:0] c;integer j;
  begin c=old^{24'd0,value};for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end
 endfunction
 wire [31:0] next_crc=crcbyte(crc,current_byte);
 wire [63:0] reciprocal_x=staging[0]*staging[8],reciprocal_y=staging[1]*staging[9];
 wire shape_ok=
  staging[0]>=(W/4)*65536 && staging[0]<=32'd536870912 &&
  staging[1]>=(H/4)*65536 && staging[1]<=32'd536870912 &&
  $signed(staging[2])>=0 && staging[2]<=(W-1)*65536 &&
  $signed(staging[3])>=0 && staging[3]<=(H-1)*65536 &&
  $signed(staging[4])>= -32'sd16777216 && $signed(staging[4])<=32'sd16777216 &&
  $signed(staging[5])>= -32'sd8388608 && $signed(staging[5])<=32'sd8388608 &&
  $signed(staging[6])>= -32'sd2097152 && $signed(staging[6])<=32'sd2097152 &&
  $signed(staging[7])>= -32'sd2097152 && $signed(staging[7])<=32'sd2097152 &&
  reciprocal_x>=64'd70367670435840 && reciprocal_x<=64'd70369817919488 &&
  reciprocal_y>=64'd70367670435840 && reciprocal_y<=64'd70369817919488 &&
  staging[10]>=32'd262144 && staging[10]<=32'd1048576 && staging[11]<=1;
 assign busy=checking;
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   request_bundle<={1'b1,32'd1,`RECTIFY_DEFAULT_PARAMS};errors<=0;checking<=0;written<=0;byte_index<=0;crc<=0;expected_crc<=0;pending_id<=0;
   for(i=0;i<12;i=i+1)staging[i]<=0;
  end else begin
   if(checking)begin
    if(write_param||commit||clear)errors<=errors+1'b1;
    crc<=next_crc;
    if(byte_index==47)begin
     checking<=0;written<=0;
     if((~next_crc)==expected_crc && shape_ok)begin
      request_bundle[416]<=1;request_bundle[415:384]<=pending_id;
      for(i=0;i<12;i=i+1)request_bundle[i*32+:32]<=staging[i];
     end else errors<=errors+1'b1;
    end else byte_index<=byte_index+1'b1;
   end else if(clear)begin
    written<=0;
    if(request_id!=0 && request_id!=request_bundle[415:384])request_bundle<={1'b0,request_id,384'd0};
    else errors<=errors+1'b1;
   end else if(commit)begin
    if(written==12'hfff && request_id!=0 && request_id!=request_bundle[415:384])begin
     checking<=1;byte_index<=0;crc<=32'hffffffff;expected_crc<=request_crc;pending_id<=request_id;
    end else begin written<=0;errors<=errors+1'b1;end
   end else if(write_param)begin
    if(param_index<12)begin staging[param_index]<=param_value;written[param_index]<=1;end
    else errors<=errors+1'b1;
   end
  end
 end
endmodule
