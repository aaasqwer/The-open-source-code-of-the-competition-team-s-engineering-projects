`timescale 1ns/1ps
// Two atomic inverse matrices. Commands: write(slot,index,value), CRC commit,
// clear. No partial matrix is exposed to another clock domain. Plane slot is
// invalid until explicitly selected on the host and committed successfully.
module rect_geometry_bank(
 input clk,rst_n,write_word,commit,clear,slot,input [3:0] index,input [31:0] value,expected_crc,
 output reg [287:0] normal_matrix,plane_matrix,output reg plane_valid,
 output reg [31:0] version,errors,output busy
);
 localparam [287:0] IDENTITY={32'h40000000,32'd0,32'd0,32'd0,32'h00100000,32'd0,32'd0,32'd0,32'h00100000};
 reg [31:0] staging[0:17];reg [17:0] written;reg checking,checking_slot;
 reg [5:0] byte_index;reg [31:0] crc,target_crc;
 wire [4:0] selected_word=(checking_slot?9:0)+byte_index[5:2];
 wire [7:0] selected_byte=staging[selected_word]>>(byte_index[1:0]*8);
 function [31:0] crcbyte;
  input [31:0] old;input [7:0] b;reg [31:0] c;integer j;
  begin c=old^{24'd0,b};for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end
 endfunction
 wire [31:0] next_crc=crcbyte(crc,selected_byte);
 assign busy=checking;
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   normal_matrix<=IDENTITY;plane_matrix<=IDENTITY;plane_valid<=0;version<=0;errors<=0;written<=0;checking<=0;checking_slot<=0;byte_index<=0;crc<=0;target_crc<=0;
   for(k=0;k<18;k=k+1)staging[k]<=0;
  end else begin
   if(checking)begin
    if(write_word||commit||clear)errors<=errors+1'b1;
    crc<=next_crc;
    if(byte_index==35)begin
     checking<=0;
     if(next_crc==~target_crc)begin
      for(k=0;k<9;k=k+1)begin
       if(checking_slot)plane_matrix[k*32+:32]<=staging[9+k];else normal_matrix[k*32+:32]<=staging[k];
      end
      if(checking_slot)plane_valid<=1;
      version<=version+1'b1;
     end else errors<=errors+1'b1;
     if(checking_slot)written[17:9]<=0;else written[8:0]<=0;
    end else byte_index<=byte_index+1'b1;
   end else if(clear)begin
    if(slot)begin plane_matrix<=IDENTITY;plane_valid<=0;written[17:9]<=0;end
    else begin normal_matrix<=IDENTITY;written[8:0]<=0;end
    version<=version+1'b1;
   end else if(commit)begin
    if((slot?written[17:9]:written[8:0])==9'h1ff)begin
     checking<=1;checking_slot<=slot;byte_index<=0;crc<=32'hffffffff;target_crc<=expected_crc;
    end else errors<=errors+1'b1;
   end else if(write_word)begin
    if(index<9)begin staging[(slot?9:0)+index]<=value;written[(slot?9:0)+index]<=1;end
    else errors<=errors+1'b1;
   end
  end
 end
endmodule
