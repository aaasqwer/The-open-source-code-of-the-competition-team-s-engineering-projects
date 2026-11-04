`timescale 1ns/1ps
module tb_frame_engine;
 localparam W=80,H=72;
 reg [31:0] source_start_tick=123456;wire [63:0] latest_origin;
 reg clk=0;always #4 clk=~clk;
 reg rst_n=0,source_valid=0;reg [1:0] source_frame=0;reg [31:0] source_serial=1;
 reg signed [15:0] angle=0;reg [2:0] display_bank=4;reg network_frame_hold=0;reg [2:0] network_frame=7;
 reg [416:0] lens_request=0;reg freeze_config=0;reg [5:0] gamma_request=6'd32;
 reg [1:0] request_mode=0;reg request_imu=0,request_plane=0,plane_valid=0;
 reg [31:0] control_sequence=0;reg [287:0] normal_matrix,plane_matrix;
 wire [34:0] latest_lens;wire [383:0] latest_lens_params;wire [1:0] held_source;wire [2:0] latest_bank;
 wire signed [15:0] latest_angle;wire frame_valid,busy;wire [31:0] render_serial,debug_position;
 wire [31:0] latest_geometry,latest_control_sequence;
 wire [27:0] araddr,awaddr;wire [3:0] arid,arlen,awid,awlen;wire arvalid,awvalid;
 reg arready=0,awready=0,wready=0,rvalid=0,rlast=0;reg [255:0] rdata=0;reg [3:0] rid=0;
 wire [255:0] wdata;wire [31:0] wstrb;
 reg [31:0] observed_render_id=0;
 always @(posedge clk)begin #2;if(rst_n&&render_serial!=observed_render_id)begin
  if(latest_origin!=={source_serial,source_start_tick})$fatal(1,"rendered source timestamp mismatch");
  observed_render_id=render_serial;
 end end
 rect_frame_engine #(.W(W),.H(H),.COMMIT_GUARD(16)) dut(.*);
 reg pending[0:6];reg [27:0] read_addr[0:6];integer due[0:6];
 reg write_pending=0;reg [27:0] write_addr;
 reg [15:0] output_memory[0:W*H-1];
 integer cycles=0,i,j,choice,bank_id,pixel_offset,errors,testcase,desired;
 function [15:0] source_pixel;
  input integer x,y;reg [4:0] r,b;reg [5:0] g;
  begin r=(x*3+y)%32;g=(x+y*5)%64;b=(x*7+y*11)%32;source_pixel={r,g,b};end
 endfunction
 reg [15:0] expected,got;reg [15:0] gold[0:2*W*H-1];
 always @(posedge clk)if(rst_n)begin
  if(arvalid&&arready)begin
   if(arlen!=0||arid>6||pending[arid])$fatal(1,"AR contract");
   pending[arid]=1;read_addr[arid]=araddr;due[arid]=cycles+3+((cycles*7+arid)%11);
  end
  if(awvalid&&awready)begin
   if(write_pending||awlen!=0||(awaddr>>22)==display_bank)$fatal(1,"AW/ownership contract");write_pending=1;write_addr=awaddr;
  end
  if(write_pending&&wready)begin
   bank_id=write_addr>>22;pixel_offset=(write_addr&28'h03fffff)*2;
   for(i=0;i<16;i=i+1)begin
    if(wstrb[i*2]!=wstrb[i*2+1])$fatal(1,"RGB565 byte mask");
    if(wstrb[i*2])output_memory[pixel_offset+i]=wdata[i*16+:16];
   end
   write_pending=0;
  end
 end
 always @(negedge clk)begin
  cycles=cycles+1;
  if(cycles>500000)$fatal(1,"timeout state=%0d map=%0d ready=%b pending=%b",dut.state,dut.maps.phase,dut.map_ready,dut.pending);
  arready=(cycles%5!=0);awready=!write_pending&&(cycles%7!=0);wready=write_pending&&(cycles%3!=0);
  rvalid=0;rlast=0;choice=-1;
  for(j=6;j>=0;j=j-1)if(pending[j]&&due[j]<=cycles&&choice==-1)choice=j;
  if(choice>=0&&cycles%4!=0)begin
   rid=choice;rvalid=1;rlast=1;pending[choice]=0;
   for(j=0;j<16;j=j+1)begin
    pixel_offset=(read_addr[choice]&28'h03fffff)*2+j;
    rdata[j*16+:16]=source_pixel(pixel_offset%W,pixel_offset/W);
   end
  end
 end
 initial begin
  $readmemh("verification/partial_frame_gold.hex",gold);
  for(i=0;i<7;i=i+1)begin pending[i]=0;due[i]=0;read_addr[i]=0;end
  for(i=0;i<W*H;i=i+1)output_memory[i]=16'hdead;
  normal_matrix={32'h40000000,32'd0,32'd0,32'd0,32'h00100000,32'd0,32'd0,32'd0,32'h00100000};
  plane_matrix={32'h40000000,32'd0,32'd0,32'h01000000,32'h00080000,32'd0,32'h01000000,32'd0,32'h00080000};
  lens_request[416]=1;lens_request[415:384]=2;
  lens_request[0+:32]=100*65536;lens_request[32+:32]=100*65536;lens_request[64+:32]=63*32768;lens_request[96+:32]=63*32768;
  lens_request[256+:32]=10737418;lens_request[288+:32]=10737418;lens_request[320+:32]=1048576;lens_request[352+:32]=1;
  repeat(4)@(negedge clk);rst_n=1;source_valid=1;
  for(testcase=0;testcase<2;testcase=testcase+1)begin
   desired=testcase+1;
   if(testcase==1)begin request_mode=2;control_sequence=1;end
   if(testcase==2)begin
    normal_matrix={32'h40000000,32'd0,32'd0,32'h03f00000,32'd0,32'hfff00000,32'd0,32'h00100000,32'd0};control_sequence=2;
   end
   if(testcase==3)begin request_mode=0;request_plane=1;plane_valid=1;control_sequence=3;end
   if(testcase==4)begin
    request_mode=3;request_plane=0;normal_matrix={32'h40000000,32'd0,32'd0,32'd0,32'h00100000,32'd0,32'd0,32'd0,32'h00100000};control_sequence=4;
   end
   while(render_serial<desired)@(negedge clk);
   if(latest_lens[0])$fatal(1,"mapping fault");errors=0;
   for(i=0;i<W*H;i=i+1)begin
    expected=gold[testcase*W*H+i];
    got=output_memory[i];
    if(got!==expected)begin if(errors<6)$display("mismatch case=%0d pixel=%0d got=%h expected=%h",testcase,i,got,expected);errors=errors+1;end
   end
   if(errors)$fatal(1,"pixel mismatches %0d in case %0d",errors,testcase);
   $display("PASS frame case=%0d bank=%0d cycles=%0d",testcase,latest_bank,cycles);display_bank=latest_bank;
  end
  $display("PASS partial-edge renderer: 80x72 RAW and identity Brown, final 16-pixel column and 8-row strip; random DDR stalls/out-of-order reads");$finish;
 end
endmodule
