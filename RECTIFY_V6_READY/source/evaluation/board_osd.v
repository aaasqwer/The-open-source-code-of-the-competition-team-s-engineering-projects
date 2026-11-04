`timescale 1ns/1ps
module board_osd(input numbers_visible,input clk,rst_n,vs,hs,de,input [15:0] pixel,
 input model,input [1:0] mode,input [3:0] evaluation_status,input [2:0] evaluation_position,input calibration_ready,calibration_fault,input [351:0] parameters,
 input diagnostic_valid,input [15:0] diagnostic_max,input [7:0] coverage_percent,worst_row,input endpoint_peak,
 input measurement_valid,input [15:0] rms_q8,max_q8,input [8:0] used_rows,input [15:0] range_low,range_high,
 input graph_visible,horizontal,editing,input sfr_graph_visible,sfr_valid,sfr_above,input[3:0]sfr_status,input[15:0]mtf50_q16,sfr_quarter_q12,input sfr_info_valid,input[15:0]sfr_frame_rms,sfr_angle_q8,input[2:0]sfr_samples,
 output reg out_vs,out_hs,out_de,output reg [15:0] out_pixel);
 reg [8:0] model_text[0:47];
 initial $readmemh("source/evaluation/model_text.hex",model_text);
 reg [8:0] status_text[0:1199];reg [8:0] region_text[0:23];
 initial begin $readmemh("source/evaluation/status.hex",status_text);$readmemh("source/evaluation/regions.hex",region_text);end
 reg [10:0] x,y;reg last_de,last_vs;
 reg [8:0] graph_labels[0:127];
 initial $readmemh("source/evaluation/graph_labels.hex",graph_labels);
 reg[8:0] sfr_text[0:351],sfr_caption[0:31];
 initial begin $readmemh("source/evaluation/sfr_status.hex",sfr_text);$readmemh("source/evaluation/sfr_caption.hex",sfr_caption);end
 reg[8:0] tips[0:1199];
 initial $readmemh("source/evaluation/sfr_tips.hex",tips);
 wire[3:0]tip_index=editing?12:mode==2?13:evaluation_status==0?11:evaluation_status!=2?14:
  sfr_status==2?1:sfr_status>=3&&sfr_status<=11?sfr_status-1:0;
 wire[7:0]info_character;reg[4:0]info_address;reg use_info;
 measurement_digits info_digits(clk,rst_n,sfr_info_valid,sfr_frame_rms,sfr_angle_q8,9'd0,info_address,info_character);
 wire sfr_title=sfr_graph_visible&&x>=32&&x<544&&y>=788&&y<820;
 wire[10:0]sfr_local_y=y-788;
 wire[7:0]sfr_character;reg[3:0]sfr_address;reg use_sfr;
 sfr_digits sharpness_digits(clk,rst_n,sfr_valid,sfr_above,mtf50_q16,sfr_quarter_q12,sfr_address,sfr_character);
 wire graph_enabled=evaluation_status!=0&&mode!=2&&graph_visible;
 wire graph_caption=graph_enabled&&x>=680&&x<936&&y>=788&&y<820;
 wire graph_tick=graph_enabled&&x>=1200&&x<1280&&((y>=860&&y<892)||(y>=924&&y<956)||(y>=988&&y<1020));
 wire [6:0] graph_label_address=graph_caption?((x-680)>>4):((y<892?32:y<956?64:96)+((x-1200)>>4));
 wire [10:0] graph_local_y=graph_caption?y-788:(y<892?y-860:y<956?y-924:y-988);
 reg [8:0] text[0:1199];reg [7:0] font[0:8191];
 initial begin $readmemh("source/evaluation/text.hex",text);$readmemh("source/evaluation/font.hex",font);end

 wire [10:0] local_x=x-11'd16,local_y=y-11'd16;
 wire area=x>=16&&x<1200&&y>=16&&y<336;
 wire [3:0] line_index=local_y>>5;wire [6:0] col=local_x>>4;
 wire [7:0] diagnostic_character,coverage_character,row_character;
 reg [4:0] diagnostic_address;reg use_diagnostic;reg use_coverage,use_row;reg [4:0] small_address;
 measurement_digits diagnostic_digits(clk,rst_n,diagnostic_valid,diagnostic_max,16'd0,9'd0,diagnostic_address,diagnostic_character);
 measurement_digits coverage_digits(clk,rst_n,diagnostic_valid,16'd0,16'd0,{1'b0,coverage_percent},small_address,coverage_character);
 measurement_digits row_digits(clk,rst_n,diagnostic_valid,16'd0,16'd0,{1'b0,worst_row},small_address,row_character);
 reg [8:0] ch;reg use_number;reg use_measurement;reg [4:0] measurement_address;wire [7:0] measurement_character;reg use_range;reg [4:0] range_address;wire [7:0] range_character;
 measurement_digits rd(clk,rst_n,measurement_valid,range_low,range_high,used_rows,range_address,range_character);
 measurement_digits md(clk,rst_n,measurement_valid,rms_q8,max_q8,used_rows,measurement_address,measurement_character);reg [7:0] number_address;
 wire [7:0] number_character;wire digits_ready;
 parameter_digits digits(clk,rst_n&&calibration_ready,calibration_ready,parameters,digits_ready,number_address,number_character);
 always @(*)begin
  use_info=0;info_address=0;use_sfr=0;sfr_address=0;use_diagnostic=0;use_coverage=0;use_row=0;diagnostic_address=0;small_address=12;use_range=0;range_address=0;use_number=0;use_measurement=0;measurement_address=0;number_address=0;ch=32;
  if(area)begin ch=text[line_index*120+col];
   if(line_index==5)begin
    ch=status_text[(evaluation_status<=9?evaluation_status:1)*120+col];
    if(col>=7&&col<11)ch=region_text[(evaluation_position<=5?evaluation_position:0)*4+col-7];
   end
   if(line_index==1&&col>=14&&col<26)begin use_number=1;number_address=0+col-14;end
   if(line_index==1&&col>=30&&col<42)begin use_number=1;number_address=12+col-30;end
   if(line_index==1&&col>=46&&col<58)begin use_number=1;number_address=24+col-46;end
   if(line_index==1&&col>=62&&col<74)begin use_number=1;number_address=36+col-62;end
   if(line_index==2&&col>=14&&col<26)begin use_number=1;number_address=48+col-14;end
   if(line_index==2&&col>=30&&col<42)begin use_number=1;number_address=60+col-30;end
   if(line_index==2&&col>=46&&col<58)begin use_number=1;number_address=72+col-46;end
   if(line_index==2&&col>=62&&col<74)begin use_number=1;number_address=84+col-62;end
   if(line_index==3&&col>=14&&col<26)begin use_number=1;number_address=96+col-14;end
   if(line_index==3&&col>=30&&col<42)begin use_number=1;number_address=108+col-30;end
   if(line_index==3&&col>=46&&col<58)begin use_number=1;number_address=120+col-46;end
   if(line_index==6&&col>=15&&col<21)begin use_measurement=1;measurement_address=0+col-15;end
   if(line_index==6&&col>=41&&col<47)begin use_measurement=1;measurement_address=6+col-41;end
   if(line_index==6&&col>=65&&col<68)begin use_measurement=1;measurement_address=12+col-65;end
   if(line_index==7&&col>=15&&col<21)begin use_range=1;range_address=0+col-15;end
   if(line_index==7&&col>=25&&col<31)begin use_range=1;range_address=6+col-25;end
   if(line_index==4&&col==55)ch=horizontal?8'd72:8'd86;
   if(line_index==4&&col==65)ch=editing?8'd89:8'd78;
   if(line_index==0&&col>=58&&col<74)ch=model_text[(calibration_ready?(model?16:0):32)+col-58];
   if(line_index==2&&model&&col==43)ch=8'd112;
   if(line_index==2&&model&&col==44)ch=8'd49;
   if(line_index==2&&model&&col==59)ch=8'd112;
   if(line_index==2&&model&&col==60)ch=8'd50;
   if(line_index==3&&col==67)ch=32;
   if(line_index==3&&col==68)ch=57;
   if(line_index==0&&col==41)ch=8'd48+mode;
   if(line_index==0&&col==55)ch=calibration_fault?8'd69:(digits_ready?8'd49:8'd48);
   if(line_index==9&&col>=17&&col<22)begin use_sfr=1;sfr_address=col-17;end
   if(line_index==9&&col>=42&&col<47)begin use_sfr=1;sfr_address=5+col-42;end
   if(line_index==5&&col>=45&&col<77)ch=sfr_text[((sfr_status==2)?1:(sfr_status>=3&&sfr_status<=11?sfr_status-1:0))*32+col-45];
   if(line_index==7&&col>=44&&col<50)begin use_info=1;info_address=6+col-44;end
   if(line_index==7&&col==63)ch=8'd48+sfr_samples;
   if(line_index==8)begin ch=tips[tip_index*80+col];use_diagnostic=0;use_coverage=0;use_row=0;end
   if(line_index==8&&tip_index==2&&col>=14&&col<20)begin use_info=1;info_address=6+col-14;end
   if(line_index==8&&tip_index==3&&col>=16&&col<22)begin use_info=1;info_address=0+col-16;end
  end
  if(use_info)ch={1'b0,info_character};
  if(use_sfr)ch={1'b0,sfr_character};
  if(use_number)ch={1'b0,number_character};
  if(use_measurement)ch={1'b0,measurement_character};
  if(use_range)ch={1'b0,range_character};
  if(use_diagnostic)ch={1'b0,diagnostic_character};
  if(use_coverage)ch={1'b0,coverage_character};
  if(use_row)ch={1'b0,row_character};
  if(graph_caption||graph_tick)ch=graph_labels[graph_label_address];
  if(sfr_title)ch=sfr_caption[(x-32)>>4];
 end
 reg [8:0] character;reg [7:0] glyph;reg [3:0] row1;reg [2:0] col1,col2;
 reg on1,on2;reg [15:0] p1,p2;reg v1,v2,h1,h2,d1,d2;
 always @(posedge clk)begin
  if(!rst_n)begin x<=0;y<=0;last_de<=0;last_vs<=0;out_vs<=0;out_hs<=0;out_de<=0;out_pixel<=0;on1<=0;on2<=0;v1<=0;v2<=0;h1<=0;h2<=0;d1<=0;d2<=0;end
  else begin
   last_de<=de;last_vs<=vs;
   if(vs&&!last_vs)begin x<=0;y<=0;end
   else begin if(de)x<=x+1'b1;else x<=0;if(last_de&&!de)y<=y+1'b1;end
   character<=ch;row1<=sfr_title?sfr_local_y[4:1]:(graph_caption||graph_tick)?graph_local_y[4:1]:local_y[4:1];
   col1<=sfr_title?((x-32)>>1):graph_tick?((x-1200)>>1):graph_caption?((x-680)>>1):local_x[3:1];on1<=(area||graph_caption||graph_tick||sfr_title)&&de&&numbers_visible;
   glyph<=font[{character,row1}];col2<=col1;on2<=on1;
   p1<=pixel;p2<=p1;v1<=vs;v2<=v1;h1<=hs;h2<=h1;d1<=de;d2<=d1;
   out_pixel<=on2?(glyph[7-col2]?16'hffe0:16'h0841):p2;
   out_vs<=v2;out_hs<=h2;out_de<=d2;
  end
 end
endmodule
