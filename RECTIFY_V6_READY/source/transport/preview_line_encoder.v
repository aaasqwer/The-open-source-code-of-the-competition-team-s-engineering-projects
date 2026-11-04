`timescale 1ns/1ps
// PGD1 bytes are unchanged. Two 16-pixel groups decouple collection/emission.
module preview_line_encoder #(parameter WIDTH=1280,PACK_RAW=0,OVERLAP_GROUPS=1)(
 input clk,rst_n,start,input [10:0] row,input write_slot,read_slot,
 output busy,done,compressed,output [12:0] length,
 output [31:0] row_crc,output [10:0] line_addr,input [15:0] line_data,
 input [11:0] encoded_addr,output [7:0] encoded_data,input encoded_raw);
 generate if(OVERLAP_GROUPS!=0)begin:fast
  preview_line_encoder_overlap #(.WIDTH(WIDTH),.PACK_RAW(PACK_RAW)) engine(
   .clk(clk),.rst_n(rst_n),.start(start),.row(row),.write_slot(write_slot),.read_slot(read_slot),
   .busy(busy),.done(done),.compressed(compressed),.length(length),.row_crc(row_crc),.line_addr(line_addr),.line_data(line_data),
   .encoded_addr(encoded_addr),.encoded_data(encoded_data),.encoded_raw(encoded_raw));
 end else begin:legacy
  preview_line_encoder_legacy #(.WIDTH(WIDTH),.PACK_RAW(PACK_RAW)) engine(
   .clk(clk),.rst_n(rst_n),.start(start),.row(row),.write_slot(write_slot),.read_slot(read_slot),
   .busy(busy),.done(done),.compressed(compressed),.length(length),.row_crc(row_crc),.line_addr(line_addr),.line_data(line_data),
   .encoded_addr(encoded_addr),.encoded_data(encoded_data),.encoded_raw(encoded_raw));
 end endgenerate
endmodule

module preview_line_encoder_overlap #(parameter WIDTH=1280,PACK_RAW=0,SLOTS=2)(
 input clk,rst_n,start,input [10:0] row,input write_slot,read_slot,
 output reg busy,done,compressed,output reg [12:0] length,
 output reg [31:0] row_crc,output [10:0] line_addr,input [15:0] line_data,
 input [11:0] encoded_addr,output [7:0] encoded_data,input encoded_raw);
 reg [15:0] pixels[0:31];reg [7:0] codes[0:31];
 reg [15:0] masks[0:1];reg [4:0] small_counts[0:1];reg same_groups[0:1];
 reg [1:0] occupied;reg collect_bank,emit_bank;
 reg [2:0] collect_state,emit_state;reg [3:0] item,emit_item;
 reg [10:0] index;reg [7:0] emitted_groups;
 reg [15:0] previous;reg [31:0] crc;reg [12:0] used;
 reg emit_compressed,emit_same,high_byte;
 wire [4:0] dr=line_data[15:11]-previous[15:11],db=line_data[4:0]-previous[4:0];
 wire [5:0] dg=line_data[10:5]-previous[10:5];
 wire small_delta=(dr[4:3]=={2{dr[2]}})&&(dg[5:3]=={3{dg[2]}})&&(db[4:2]=={3{db[1]}});
 assign line_addr=(collect_state==3&&item<15)?index+1'b1:index;
 function [31:0] crcbyte;input [31:0] c0;input [7:0] b;reg [31:0] c;integer j;
 begin c=c0^b;for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end endfunction
 wire delta_byte=emit_compressed&&masks[emit_bank][emit_item];
 wire group_done=(emit_state==1&&emit_same)||(emit_state==4&&emit_item==15&&(delta_byte||high_byte));
 wire memory_write=rst_n&&busy&&emit_state!=0;
 wire [7:0] memory_byte=emit_state==1?(emit_same?8'd2:(emit_compressed?8'd1:8'd0)):
  emit_state==2?masks[emit_bank][7:0]:emit_state==3?masks[emit_bank][15:8]:
  (delta_byte?codes[{emit_bank,emit_item}]:(high_byte?pixels[{emit_bank,emit_item}][15:8]:pixels[{emit_bank,emit_item}][7:0]));
 localparam ENCODED_ADDR_BITS=SLOTS==1?12:13;
 wire [12:0] write_address={write_slot,used[11:0]},read_address={read_slot,encoded_addr};
 reg [7:0] memory[0:(1<<ENCODED_ADDR_BITS)-1] /* synthesis syn_ramstyle="block_ram" */;
 reg [15:0] raw_memory[0:4095] /* synthesis syn_ramstyle="block_ram" */;
 reg [7:0] encoded_q;reg [15:0] raw_q;reg raw_select,raw_high;
 always @(posedge clk)begin
  if(memory_write)memory[write_address[ENCODED_ADDR_BITS-1:0]]<=memory_byte;
  if(PACK_RAW!=0&&rst_n&&busy&&collect_state==3)raw_memory[{write_slot,index}]<=line_data;
  encoded_q<=memory[read_address[ENCODED_ADDR_BITS-1:0]];raw_q<=raw_memory[{read_slot,encoded_addr[11:1]}];
  raw_select<=PACK_RAW!=0&&encoded_raw;raw_high<=encoded_addr[0];
 end
 assign encoded_data=raw_select?(raw_high?raw_q[15:8]:raw_q[7:0]):encoded_q;
 integer k;
 always @(posedge clk)begin
  done<=0;
  if(!rst_n)begin
   busy<=0;compressed<=0;length<=0;row_crc<=0;occupied<=0;collect_bank<=0;emit_bank<=0;
   collect_state<=0;emit_state<=0;item<=0;emit_item<=0;index<=0;emitted_groups<=0;previous<=0;crc<=0;used<=0;
   emit_compressed<=0;emit_same<=0;high_byte<=0;
   for(k=0;k<2;k=k+1)begin masks[k]<=0;small_counts[k]<=0;same_groups[k]<=1;end
  end else if(!busy)begin
   if(start)begin
    busy<=1;occupied<=0;collect_bank<=0;emit_bank<=0;collect_state<=1;emit_state<=0;
    item<=0;emit_item<=0;index<=0;emitted_groups<=0;previous<=0;used<=0;
    crc<=crcbyte(crcbyte(32'hffffffff,row[7:0]),{5'd0,row[10:8]});
    for(k=0;k<2;k=k+1)begin masks[k]<=0;small_counts[k]<=0;same_groups[k]<=1;end
   end
  end else begin
   case(collect_state)
    1:collect_state<=2;
    2:collect_state<=3;
    3:begin
     pixels[{collect_bank,item}]<=line_data;codes[{collect_bank,item}]<={dr[2:0],dg[2:0],db[1:0]};
     masks[collect_bank][item]<=small_delta;small_counts[collect_bank]<=small_counts[collect_bank]+small_delta;
     same_groups[collect_bank]<=same_groups[collect_bank]&&(line_data==previous);previous<=line_data;
     crc<=crcbyte(crcbyte(crc,line_data[7:0]),line_data[15:8]);
     if(item==15)begin
      occupied[collect_bank]<=1;collect_bank<=~collect_bank;
      if(index==WIDTH-1)collect_state<=5;else begin index<=index+1'b1;collect_state<=4;end
     end else begin item<=item+1'b1;index<=index+1'b1;end
    end
    4:if(!occupied[collect_bank])begin
     item<=0;masks[collect_bank]<=0;small_counts[collect_bank]<=0;same_groups[collect_bank]<=1;collect_state<=3;
    end
    default:begin end
   endcase
   case(emit_state)
    0:if(occupied[emit_bank])begin emit_compressed<=small_counts[emit_bank]>=4;emit_same<=same_groups[emit_bank];emit_item<=0;high_byte<=0;emit_state<=1;end
    1:if(!emit_same)emit_state<=emit_compressed?2:4;
    2:emit_state<=3;
    3:emit_state<=4;
    4:if(delta_byte||high_byte)begin high_byte<=0;emit_item<=emit_item+1'b1;end else high_byte<=1;
   endcase
   if(memory_write)used<=used+1'b1;
   if(group_done)begin
    occupied[emit_bank]<=0;emit_bank<=~emit_bank;emitted_groups<=emitted_groups+1'b1;emit_state<=0;
    if(emitted_groups==WIDTH/16-1)begin
     busy<=0;done<=1;collect_state<=0;
     compressed<=(used+13'd1)<WIDTH*2;length<=(used+13'd1)<WIDTH*2?(used+13'd1):WIDTH*2;row_crc<=~crc;
    end
   end
  end
 end
endmodule

`timescale 1ns/1ps
// PGD1: 16-pixel groups, raw or mask + lossless RGB565 component deltas.
// A whole row falls back to raw if the encoded row is not smaller.
module preview_line_encoder_legacy #(parameter WIDTH=1280,PACK_RAW=0)(
 input clk,rst_n,start,input [10:0] row,input write_slot,read_slot,
 output reg busy,done,output reg compressed,output reg [12:0] length,
 output reg [31:0] row_crc,output [10:0] line_addr,input [15:0] line_data,
 input [11:0] encoded_addr,output reg [7:0] encoded_data,input encoded_raw);
 reg [15:0] pixels[0:15];reg [7:0] codes[0:15];reg [15:0] mask;
 reg [15:0] previous;reg [10:0] index;
 reg [3:0] item,emit_item;reg [4:0] small_count;
 reg [12:0] used;reg [31:0] crc;reg [3:0] state;
 reg group_compressed,high_byte,all_same;
 wire [4:0] dr=line_data[15:11]-previous[15:11],db=line_data[4:0]-previous[4:0];
 wire [5:0] dg=line_data[10:5]-previous[10:5];
 wire small_delta=(dr[4:3]=={2{dr[2]}})&&(dg[5:3]=={3{dg[2]}})&&(db[4:2]=={3{db[1]}});
 assign line_addr=(state==3&&item<15)?index+1'b1:index;
 function [31:0] crcbyte;input [31:0] c0;input[7:0] b;reg[31:0] c;integer j;
 begin c=c0^b;for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end endfunction
 wire memory_write=rst_n&&(state==4||state==5||state==6||state==7);
 wire[7:0] memory_byte=state==4?(all_same?8'd2:(small_count>=4?8'd1:8'd0)):
  state==5?mask[7:0]:state==6?mask[15:8]:
  (group_compressed&&mask[emit_item]?codes[emit_item]:(high_byte?pixels[emit_item][15:8]:pixels[emit_item][7:0]));
 // One synchronous write and one synchronous read fit dual-port block RAM.
 // Keep the large encoded buffer off the LUT RAM address/multiplexer fabric.
 reg[7:0]memory[0:8191] /* synthesis syn_ramstyle="block_ram" */;
 reg[15:0]raw_memory[0:4095] /* synthesis syn_ramstyle="block_ram" */;
 reg[7:0]encoded_q;reg[15:0]raw_q;reg raw_select,raw_high;
 always @(posedge clk)begin
  if(memory_write)memory[{write_slot,used[11:0]}]<=memory_byte;
  if(PACK_RAW!=0&&rst_n&&state==3)raw_memory[{write_slot,index}]<=line_data;
  encoded_q<=memory[{read_slot,encoded_addr}];
  raw_q<=raw_memory[{read_slot,encoded_addr[11:1]}];
  raw_select<=PACK_RAW!=0&&encoded_raw;raw_high<=encoded_addr[0];
 end
 always @*encoded_data=raw_select?(raw_high?raw_q[15:8]:raw_q[7:0]):encoded_q;
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
    else begin item<=item+1'b1;index<=index+1'b1;state<=3;end
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
