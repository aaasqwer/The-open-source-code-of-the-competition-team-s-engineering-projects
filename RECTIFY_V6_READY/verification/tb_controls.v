`timescale 1ns/1ps
module tb_controls;
reg clk=0,rst=0;always #5 clk=~clk;
reg [4:0] keys=31;wire [1:0] mode;wire plane,page,imu,caption,numbers,eval_on,editing,horizontal;wire [2:0] position;wire [10:0] cx,cy;
wire [7:0] ui_flags;wire [31:0] ui_seq;
reg [7:0] data=0;reg rx=0;wire gw,gc,gclear,slot,uu;wire [3:0] gi;wire [31:0] gv,crc;wire [7:0] flags;wire [31:0] command_errors;
wire [287:0] normal,pm;wire pv,busy;wire [31:0] version,errors;
rect_board_ui #(.CLK_HZ(1000),.DEBOUNCE(2),.LONG_PRESS(40)) ui(.clk(clk),.rst_n(rst),.buttons_n(keys),.remote_update(uu),.remote_flags(flags),
 .mode(mode),.ui_flags(ui_flags),.imu_enabled(imu),.plane_held(plane),.imu_page(page),.caption_visible(caption),.numbers_visible(numbers),.evaluation_enabled(eval_on),.editing(editing),.horizontal(horizontal),.position(position),.center_x(cx),.center_y(cy),.sequence_id(ui_seq));
host_command_parser #(.CLK_FREQ_HZ(1000)) parser(.clk(clk),.rst_n(rst),.rx_data(data),.rx_valid(rx),.geo_write(gw),.geo_commit(gc),.geo_clear(gclear),.geo_slot(slot),.geo_index(gi),.geo_value(gv),.geo_crc(crc),.ui_update(uu),.ui_flags(flags),.command_error_count(command_errors));
rect_geometry_bank bank(.clk(clk),.rst_n(rst),.write_word(gw),.commit(gc),.clear(gclear),.slot(slot),.index(gi),.value(gv),.expected_crc(crc),.normal_matrix(normal),.plane_matrix(pm),.plane_valid(pv),.version(version),.errors(errors),.busy(busy));
task cycles;input integer n;begin repeat(n)@(negedge clk);end endtask
task press;input integer key,n;begin keys[key]=0;cycles(n);keys[key]=1;cycles(12);end endtask
task byte_in;input [7:0] b;begin @(negedge clk);data=b;rx=1;@(negedge clk);rx=0;end endtask
reg [7:0] payload[0:7];reg [7:0] sum;integer j;reg [31:0] words[0:8];reg [31:0] crc_value;
task packet;input [7:0] kind,n;begin
 sum=8'ha5+8'h5a+kind+n;byte_in(8'ha5);byte_in(8'h5a);byte_in(kind);byte_in(n);
 for(j=0;j<n;j=j+1)begin sum=sum+payload[j];byte_in(payload[j]);end
 byte_in(sum);cycles(5);
end endtask
task write_matrix;integer k;begin
 for(k=0;k<9;k=k+1)begin payload[0]=1;payload[1]=k;{payload[5],payload[4],payload[3],payload[2]}=words[k];packet(8'h50,6);end
end endtask
task commit_matrix;input [31:0] c;begin payload[0]=1;{payload[4],payload[3],payload[2],payload[1]}=c;packet(8'h51,5);cycles(45);end endtask
initial begin
 $readmemh("verification/control_words.hex",words);$readmemh("verification/control_crc.hex",crc_value_dummy);
 #22;rst=1;cycles(8);
 if(mode!=0||plane||!imu||!numbers||!caption)$fatal(1,"startup state");
 press(0,12);if(mode!=1)$fatal(1,"KEY0 correction");press(0,12);if(mode!=0)$fatal(1,"KEY0 bypass");
 press(1,12);if(mode!=3)$fatal(1,"KEY1 direct split");press(1,12);if(mode!=3)$fatal(1,"KEY1 remains comparison");
 press(0,12);if(mode!=1)$fatal(1,"KEY0 exits comparison to full correction");
 press(2,12);if(imu)$fatal(1,"KEY2 IMU");press(2,60);if(numbers||imu)$fatal(1,"long KEY2 must hide parameters and suppress short");press(2,60);if(!numbers||imu)$fatal(1,"second long KEY2 must restore parameters");
 keys[0]=0;cycles(60);if(!plane||page)$fatal(1,"KEY0 long plane hold");keys[0]=1;cycles(12);if(plane||mode!=1)$fatal(1,"plane release must retain mode");
 keys[4]=0;cycles(12);if(!page||plane)$fatal(1,"reference KEY4 IMU page");cycles(60);if(!page)$fatal(1,"IMU page long hold");keys[4]=1;cycles(12);if(page)$fatal(1,"page release cannot latch");
 press(3,12);if(!eval_on||position!=0)$fatal(1,"ROI on");press(3,12);if(position!=1)$fatal(1,"ROI preset");press(3,60);if(!editing)$fatal(1,"ROI editing");
 press(0,12);if(cx!=160||mode!=1)$fatal(1,"ROI edit remap");press(4,12);if(!horizontal||plane)$fatal(1,"ROI orientation");press(3,60);if(editing)$fatal(1,"ROI edit exit");
 payload[0]=8'h41;packet(8'h53,1);if(mode!=1||imu||numbers)$fatal(1,"host UI flags");
 payload[0]=8'h47;packet(8'h53,1);if(command_errors!=1||mode!=1)$fatal(1,"invalid profile must reject");
 commit_matrix(32'd0);if(pv||errors!=1)$fatal(1,"incomplete matrix exposed");
 write_matrix();if(pv||version!=0)$fatal(1,"partial matrix exposed");
 commit_matrix(crc_value_dummy[0]^1);if(pv||errors!=2)$fatal(1,"bad CRC exposed");
 write_matrix();commit_matrix(crc_value_dummy[0]);
 if(!pv||version!=1||pm[31:0]!=words[0]||pm[287:256]!=words[8])$fatal(1,"CRC commit");
 payload[0]=1;packet(8'h52,1);if(pv||version!=2)$fatal(1,"plane clear");
 $display("PASS key debounce/short/long/ROI/physical plane release, command validation, atomic matrix CRC");$finish;
end
reg [31:0] crc_value_dummy[0:0];
initial begin #10000000;$fatal(1,"watchdog");end
endmodule
