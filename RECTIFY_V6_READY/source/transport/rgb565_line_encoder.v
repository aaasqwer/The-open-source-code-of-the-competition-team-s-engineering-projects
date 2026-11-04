`timescale 1ns/1ps
// PGD1: 16-pixel groups, raw or mask + lossless RGB565 component deltas.
// A whole row falls back to raw if the encoded row is not smaller.
module rgb565_line_encoder #(parameter WIDTH=1920,parameter RAM_STYLE="block_ram",parameter OVERLAP_GROUPS=1)(
 input clk,rst_n,start,input [10:0] row,
 output busy,done,compressed,output [12:0] length,
 output [31:0] row_crc,output [10:0] line_addr,input [15:0] line_data,
 input [11:0] encoded_addr,output [7:0] encoded_data);
 generate if(OVERLAP_GROUPS!=0 && WIDTH>640)begin:fast
  // The same proven lossless engine serves photos and preview. Photos need
  // one encoded-row slot; the external DDR reader retains the raw fallback.
  preview_line_encoder_overlap #(.WIDTH(WIDTH),.PACK_RAW(0),.SLOTS(1)) engine(
   .clk(clk),.rst_n(rst_n),.start(start),.row(row),.write_slot(1'b0),.read_slot(1'b0),
   .busy(busy),.done(done),.compressed(compressed),.length(length),.row_crc(row_crc),
   .line_addr(line_addr),.line_data(line_data),.encoded_addr(encoded_addr),.encoded_data(encoded_data),.encoded_raw(1'b0));
 end else begin:legacy
  rgb565_line_encoder_legacy #(.WIDTH(WIDTH),.RAM_STYLE(RAM_STYLE)) engine(
   .clk(clk),.rst_n(rst_n),.start(start),.row(row),.busy(busy),.done(done),.compressed(compressed),
   .length(length),.row_crc(row_crc),.line_addr(line_addr),.line_data(line_data),.encoded_addr(encoded_addr),.encoded_data(encoded_data));
 end endgenerate
endmodule

module rgb565_line_encoder_legacy #(parameter WIDTH=1920,parameter RAM_STYLE="block_ram")(
 input clk,rst_n,start,input [10:0] row,
 output reg busy,done,output reg compressed,output reg [12:0] length,
 output reg [31:0] row_crc,output [10:0] line_addr,input [15:0] line_data,
 input [11:0] encoded_addr,output reg [7:0] encoded_data);
 reg [15:0] pixels[0:15];reg [7:0] codes[0:15];reg [15:0] mask;
 reg [15:0] previous;reg [10:0] index;
 reg [3:0] item,emit_item;reg [4:0] small_count;
 reg [12:0] used;reg [31:0] crc;reg [3:0] state;
 reg group_compressed,high_byte,all_same;
 wire [4:0] dr=line_data[15:11]-previous[15:11],db=line_data[4:0]-previous[4:0];
 wire [5:0] dg=line_data[10:5]-previous[10:5];
 wire small_delta=(dr[4:3]=={2{dr[2]}})&&(dg[5:3]=={3{dg[2]}})&&(db[4:2]=={3{db[1]}});
 assign line_addr=index;
 function [31:0] crcbyte;input [31:0] c0;input[7:0] b;reg[31:0] c;integer j;
 begin c=c0^b;for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end endfunction
 wire memory_write=rst_n&&(state==4||state==5||state==6||state==7);
 wire[7:0] memory_byte=state==4?(all_same?8'd2:(small_count>=4?8'd1:8'd0)):
  state==5?mask[7:0]:state==6?mask[15:8]:
  (group_compressed&&mask[emit_item]?codes[emit_item]:(high_byte?pixels[emit_item][15:8]:pixels[emit_item][7:0]));
 // Literal attributes are intentional: this PDS version did not honor a
 // parameter-valued style on the preview RAM, consuming a scarce DRM tile.
 generate if(WIDTH<=640)begin:distributed_bytes
  reg[7:0]memory[0:2047] /* synthesis syn_ramstyle="select_ram" */;
  always @(posedge clk)begin
   if(memory_write)memory[used[10:0]]<=memory_byte;
   encoded_data<=memory[encoded_addr[10:0]];
  end
 end else begin:block_bytes
  reg[7:0]memory[0:4095] /* synthesis syn_ramstyle="block_ram" */;
  always @(posedge clk)begin
   if(memory_write)memory[used[11:0]]<=memory_byte;
   encoded_data<=memory[encoded_addr];
  end
 end endgenerate
 always @(posedge clk)begin
  done<=0;
  if(!rst_n)begin busy<=0;done<=0;state<=0;index<=0;used<=0;length<=0;row_crc<=0;compressed<=0;end
  else case(state)
   0:if(start)begin busy<=1;index<=0;used<=0;item<=0;mask<=0;small_count<=0;previous<=0;all_same<=1;
    crc<=crcbyte(crcbyte(32'hffffffff,row[7:0]),{5'd0,row[10:8]});state<=1;end
   1:state<=2;
   2:state<=3;
   3:begin
    pixels[item]<=line_data;codes[item]<={dr[2:0],dg[2:0],db[1:0]};mask[item]<=small_delta;
    previous<=line_data;small_count<=small_count+small_delta;all_same<=all_same&&(line_data==previous);
    crc<=crcbyte(crcbyte(crc,line_data[7:0]),line_data[15:8]);
    if(item==15)state<=4;
    else begin item<=item+1'b1;index<=index+1'b1;state<=1;end
   end
   4:begin
    group_compressed<=small_count>=4;
    used<=used+1'b1;emit_item<=0;high_byte<=0;state<=all_same?8:(small_count>=4?5:7);
   end
   5:begin used<=used+1'b1;state<=6;end
   6:begin used<=used+1'b1;state<=7;end
   7:begin
    used<=used+1'b1;
    if((group_compressed&&mask[emit_item])||high_byte)begin
     high_byte<=0;
     if(emit_item==15)state<=8;else emit_item<=emit_item+1'b1;
    end else high_byte<=1;
   end
   8:if(index==WIDTH-1)begin
    compressed<=used<WIDTH*2;length<=used<WIDTH*2?used:WIDTH*2;row_crc<=~crc;busy<=0;done<=1;state<=0;
   end else begin index<=index+1'b1;item<=0;mask<=0;small_count<=0;all_same<=1;state<=1;end
   default:state<=0;
  endcase
 end
endmodule
