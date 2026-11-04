`timescale 1ns/1ps
// Full-screen video with a translucent Chinese bottom overlay, 60x8 cells.
// Numeric units are unchanged. Text and font ROM add two matched pixel clocks.
module imu_hud_overlay #(parameter HUD_X0=60,HUD_Y0=792,BUILD_DELAY=40000,IMAGE_DIRECTION=-1,RGB888_INPUT=0,
 parameter BUILD_MAJOR=8'h31,BUILD_DIGIT=8'h32,SMOOTHING_MODE=0,PROJECTION_AXIS=-1,REFLOW_PAGE=0)(
 input hold_frame,osd_enable,data_page,
 input pix_clk,rst_n,video_de,video_vs,video_hs,input [11:0] pix_x,pix_y,
 input [15:0] rgb_in,input [23:0] rgb888_in,input [639:0] telemetry,
 output reg [23:0] rgb888_out,output reg [15:0] rgb_out,output reg de_out,vs_out,hs_out,
 output [11:0] out_x,out_y);
 localparam COLS=60,ROWS=8,COUNT=480;
 reg [639:0] data_snapshot;reg [15:0] build_count;
 wire [575:0] values=data_snapshot[575:0];
 wire [31:0] valid=data_snapshot[607:576],status=data_snapshot[639:608];
 localparam ASM_LAYOUT=((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h39) && PROJECTION_AXIS>=0);
 wire [7:0] mounting_char=status[28]?(status[27]?8'h56:8'h48):8'h3f;
 reg [31:0] baud_rate;
 always @* case(status[14:11])
 0:baud_rate=115200;
 1:baud_rate=9600;
 2:baud_rate=57600;
 3:baud_rate=38400;
 4:baud_rate=19200;
 5:baud_rate=230400;
 6:baud_rate=256000;
 7:baud_rate=460800;
 8:baud_rate=921600;
 9:baud_rate=4800;
 10:baud_rate=2400;
 default:baud_rate=0;endcase
 reg [7:0] text_mem[0:2047];reg active_bank,write_bank,text_ready,vs_d;
 reg [3:0] state;reg [9:0] init_index;reg [4:0] field;
 reg [5:0] bit_count;reg [3:0] char_position;
 reg [31:0] binary;reg [39:0] bcd;reg negative;
 integer digit_index,k;reg [39:0] adjusted;reg [7:0] number_char;
 wire [31:0] selected=(field==17)?{16'd0,values[575:560]}:
                         (field==18)?{16'd0,values[559:544]}:(field==19)?baud_rate:values[field*32+:32];
 wire number_valid=(field>=17)?1'b1:valid[field];
 reg [9:0] field_base;reg [3:0] field_digits,field_frac,field_width;reg field_sign;
 always @* begin field_base=0;field_digits=1;field_frac=0;field_sign=0;case(field)
 0:begin field_base=64;field_digits=4;field_frac=1;field_sign=0;end
 1:begin field_base=84;field_digits=4;field_frac=1;field_sign=1;end
 2:begin field_base=104;field_digits=4;field_frac=1;field_sign=1;end
 3:begin field_base=124;field_digits=4;field_frac=1;field_sign=1;end
 4:begin field_base=185;field_digits=5;field_frac=1;field_sign=1;end
 5:begin field_base=205;field_digits=5;field_frac=1;field_sign=1;end
 6:begin field_base=225;field_digits=5;field_frac=1;field_sign=1;end
 7:begin field_base=245;field_digits=5;field_frac=3;field_sign=1;end
 8:begin field_base=265;field_digits=5;field_frac=3;field_sign=1;end
 9:begin field_base=285;field_digits=5;field_frac=3;field_sign=1;end
 10:begin field_base=163;field_digits=10;field_frac=2;field_sign=1;end
 11:begin field_base=303;field_digits=10;field_frac=2;field_sign=1;end
 12:begin field_base=323;field_digits=10;field_frac=2;field_sign=1;end
 13:begin field_base=343;field_digits=5;field_frac=2;field_sign=1;end
 14:begin field_base=144;field_digits=4;field_frac=1;field_sign=1;end
 15:begin field_base=365;field_digits=4;field_frac=0;field_sign=0;end
 16:begin field_base=386;field_digits=2;field_frac=0;field_sign=0;end
 17:begin field_base=403;field_digits=2;field_frac=0;field_sign=0;end
 18:begin field_base=408;field_digits=4;field_frac=0;field_sign=0;end
 19:begin field_base=26;field_digits=6;field_frac=0;field_sign=0;end
 default:;endcase field_width=field_digits+field_sign+(field_frac!=0);end
 reg [15:0] direction;
 always @* begin
 if(values[31:0]<225 || values[31:0]>=3375) direction={8'd139,8'd32};
 else if(values[31:0]<675) direction={8'd129,8'd139};
 else if(values[31:0]<1125) direction={8'd129,8'd32};
 else if(values[31:0]<1575) direction={8'd129,8'd140};
 else if(values[31:0]<2025) direction={8'd140,8'd32};
 else if(values[31:0]<2475) direction={8'd186,8'd140};
 else if(values[31:0]<2925) direction={8'd186,8'd32};
 else direction={8'd186,8'd139};end
 function [7:0] template;input [9:0] address;begin case(address)
 0:template=8'd157;
 1:template=8'd156;
 2:template=8'd170;
 3:template=8'd196;
 5:template=8'd192;
 6:template=8'd160;
 7:template=8'd58;
 8:template=status[0]?8'd166:8'd180;
 9:template=status[0]?8'd151:8'd153;
 11:template=8'd145;
 12:template=8'd136;
 13:template=8'd58;
 14:template=status[10]?8'd163:(status[9]?8'd177:8'd175);
 15:template=status[10]?8'd176:(status[9]?8'd183:8'd147);
 17:template=8'd161;
 18:template=8'd143;
 19:template=8'd58;
 20:template=(status[9] && !status[10] && valid[0])?direction[15-:8]:8'h2d;
 21:template=(status[9] && !status[10] && valid[0])?direction[7-:8]:8'h2d;
 23:template=8'd169;
 24:template=8'd174;
 25:template=8'd58;
 39:template=ASM_LAYOUT?8'd162:8'h20;
 40:template=ASM_LAYOUT?8'd188:8'd162;
 41:template=ASM_LAYOUT?8'd189:8'd188;
 42:template=ASM_LAYOUT?8'h3a:8'd189;
 43:template=ASM_LAYOUT?((PROJECTION_AXIS==3)?8'h4d:(8'h58+PROJECTION_AXIS)):8'h3a;
 44:template=ASM_LAYOUT?8'h20:((PROJECTION_AXIS>=0)?((PROJECTION_AXIS==3)?8'h4d:(8'h58+PROJECTION_AXIS)):(status[16:15]==0?8'd164:(status[16:15]==1?8'd133:8'd183)));
 45:template=ASM_LAYOUT?8'h41:((PROJECTION_AXIS>=0)?8'h20:(status[16:15]==0?8'd173:(status[16:15]==1?8'd131:8'd143)));
 46:template=ASM_LAYOUT?8'h3a:8'h20;
 47:template=ASM_LAYOUT?mounting_char:8'h20;
 48:template=8'h20;
 32:template=((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h35))?8'h49:8'h20;
 33:template=((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h35))?8'h52:8'h20;
 34:template=((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h35))?8'h3a:8'h20;
 35:template=((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h35))?(8'h30+SMOOTHING_MODE):8'h20;
 36:template=(PROJECTION_AXIS>=0)?8'h47:(((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h39))?8'h41:8'h20);
 37:template=(PROJECTION_AXIS>=0 || (BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h39))?8'h3a:8'h20;
 38:template=(PROJECTION_AXIS>=0)?(8'h30+status[26]):(((BUILD_MAJOR>8'h31 || BUILD_DIGIT>=8'h39))?mounting_char:8'h20);
 49:template=8'd86;
 50:template=BUILD_MAJOR;
 51:template=BUILD_DIGIT;
 52:template=(IMAGE_DIRECTION<0)?8'd45:8'd43;
 53:template=8'd49;
 54:template=8'd48;
 55:template=8'd56;
 56:template=8'd48;
 57:template=8'd80;
 58:template=8'd51;
 59:template=8'd48;
 60:template=8'd161;
 61:template=8'd132;
 62:template=8'd187;
 70:template=8'd152;
 80:template=8'd164;
 81:template=8'd173;
 82:template=8'd187;
 91:template=8'd152;
 100:template=8'd133;
 101:template=8'd131;
 102:template=8'd187;
 111:template=8'd152;
 120:template=8'd183;
 121:template=8'd143;
 122:template=8'd187;
 131:template=8'd152;
 140:template=8'd144;
 141:template=8'd134;
 142:template=8'd187;
 151:template=8'd152;
 160:template=8'd171;
 161:template=8'd159;
 176:template=8'd181;
 180:template=8'd187;
 181:template=8'd193;
 182:template=8'd152;
 183:template=8'd88;
 193:template=8'd152;
 194:template=8'd47;
 195:template=8'd178;
 200:template=8'd187;
 201:template=8'd193;
 202:template=8'd152;
 203:template=8'd89;
 213:template=8'd152;
 214:template=8'd47;
 215:template=8'd178;
 220:template=8'd187;
 221:template=8'd193;
 222:template=8'd152;
 223:template=8'd90;
 233:template=8'd152;
 234:template=8'd47;
 235:template=8'd178;
 240:template=8'd137;
 241:template=8'd193;
 242:template=8'd152;
 243:template=8'd88;
 253:template=8'd71;
 260:template=8'd137;
 261:template=8'd193;
 262:template=8'd152;
 263:template=8'd89;
 273:template=8'd71;
 280:template=8'd137;
 281:template=8'd193;
 282:template=8'd152;
 283:template=8'd90;
 293:template=8'd71;
 300:template=8'd175;
 301:template=8'd201;
 316:template=8'd181;
 320:template=8'd168;
 321:template=8'd141;
 336:template=8'd104;
 337:template=8'd80;
 338:template=8'd97;
 340:template=8'd172;
 341:template=8'd152;
 351:template=8'd128;
 360:template=8'd199;
 361:template=8'd184;
 362:template=8'd196;
 363:template=8'd179;
 370:template=8'd152;
 371:template=8'd47;
 372:template=8'd178;
 380:template=8'd137;
 381:template=8'd193;
 382:template=8'd152;
 383:template=8'd196;
 384:template=8'd179;
 389:template=8'd71;
 // V22: U:edge/decoded-byte/scan-wrap. Diagnostic only, not a link lock.
 393:template=(BUILD_MAJOR==8'h32 && BUILD_DIGIT>=8'h32)?8'h55:8'h20;
 394:template=(BUILD_MAJOR==8'h32 && BUILD_DIGIT>=8'h32)?8'h3a:8'h20;
 395:template=(BUILD_MAJOR==8'h32 && BUILD_DIGIT>=8'h32)?(8'h30+status[29]):8'h20;
 396:template=(BUILD_MAJOR==8'h32 && BUILD_DIGIT>=8'h32)?(8'h30+status[30]):8'h20;
 397:template=(BUILD_MAJOR==8'h32 && BUILD_DIGIT>=8'h32)?(8'h30+status[31]):8'h20;
 400:template=8'd198;
 401:template=8'd167;
 402:template=8'd58;
 406:template=8'd185;
 407:template=8'd58;
 414:template=8'd154;
 415:template=8'd58;
 416:template=8'h30+status[20];
 420:template=8'd146;
 421:template=8'd155;
 422:template=8'd58;
 423:template=8'h30+valid[17];
 425:template=8'd191;
 426:template=8'd138;
 427:template=8'd58;
 428:template=8'h30+valid[18];
 430:template=8'd168;
 431:template=8'd141;
 432:template=8'd58;
 433:template=8'h30+valid[19];
 435:template=8'd194;
 436:template=8'd197;
 437:template=8'd58;
 438:template=8'h30+status[4];
 440:template=8'd195;
 441:template=8'd197;
 442:template=8'd58;
 443:template=8'h30+status[5];
 445:template=8'd165;
 446:template=8'd190;
 447:template=8'd58;
 448:template=8'h30+status[6];
 450:template=8'd195;
 451:template=8'd200;
 452:template=8'd58;
 453:template=8'h30+status[21];
 455:template=8'd158;
 456:template=8'd150;
 457:template=8'd58;
 458:template=8'h30+status[22];
 460:template=8'd148;
 461:template=8'd182;
 462:template=8'd58;
 463:template=8'h30+status[23];
 465:template=8'd130;
 466:template=8'd142;
 467:template=8'd197;
 468:template=8'd58;
 469:template=8'h30+status[2];
 471:template=8'd68;
 472:template=8'd68;
 473:template=8'd82;
 474:template=8'd58;
 475:template=8'h30+status[24];
 477:template=8'd69;
 478:template=8'd58;
 479:template=8'h30+status[25];
 default:template=8'h20;endcase end endfunction
 always @* begin
  adjusted=bcd;for(k=0;k<10;k=k+1) if(bcd[k*4+:4]>=5) adjusted[k*4+:4]=bcd[k*4+:4]+3;
  number_char=8'h2d;digit_index=0;
  if(number_valid) begin
   if(field_sign && char_position==0) number_char=negative?8'h2d:8'h2b;
   else if(field_frac!=0 && char_position==field_width-field_frac-1) number_char=8'h2e;
   else begin digit_index=field_digits-1-char_position+field_sign;
    if(field_frac!=0 && char_position>field_width-field_frac-1) digit_index=digit_index+1;
    number_char=8'h30+((bcd>>(digit_index*4))&15);end
  end
 end
 always @(posedge pix_clk or negedge rst_n) begin
  if(!rst_n) begin data_snapshot<=0;active_bank<=0;write_bank<=1;text_ready<=0;vs_d<=0;
   state<=0;init_index<=0;field<=0;bit_count<=0;char_position<=0;binary<=0;bcd<=0;negative<=0;build_count<=0;end
  else begin vs_d<=video_vs;case(state)
   0:if(!hold_frame && video_vs && !vs_d) begin build_count<=0;state<=5;end
   5:if(build_count==BUILD_DELAY) begin data_snapshot<=telemetry;write_bank<=!active_bank;init_index<=0;state<=1;end else build_count<=build_count+1'b1;
   1:begin text_mem[{write_bank,init_index}]<=template(init_index);
      if(init_index==COUNT-1) begin field<=0;state<=2;end else init_index<=init_index+1'b1;end
   2:begin negative<=field_sign && selected[31];binary<=(field_sign && selected[31])?-selected:selected;
      bcd<=0;bit_count<=0;char_position<=0;state<=3;end
   3:begin bcd<={adjusted[38:0],binary[31]};binary<={binary[30:0],1'b0};
      if(bit_count==31) state<=4;else bit_count<=bit_count+1'b1;end
   4:begin text_mem[{write_bank,field_base+{6'd0,char_position}}]<=number_char;
      if(char_position==field_width-1) begin
       if(field==19) begin active_bank<=write_bank;text_ready<=1;state<=0;end
       else begin field<=field+1'b1;state<=2;end
      end else char_position<=char_position+1'b1;end
   default:state<=0;endcase
  end
 end
 wire [11:0] local_x=pix_x-HUD_X0,local_y=pix_y-HUD_Y0;
 wire page_layout=REFLOW_PAGE!=0&&data_page;
 wire in_panel=osd_enable && video_de && (page_layout ? pix_x>=64&&pix_x<1184&&pix_y>=192&&pix_y<816 : pix_x>=HUD_X0 && pix_x<HUD_X0+1800 && pix_y>=HUD_Y0 && pix_y<HUD_Y0+256);
 wire [27:0] column_product=local_x*16'd2185;wire [5:0] column=column_product[21:16];
 wire [2:0] row=local_y[7:5];wire [11:0] cell_x=local_x-column*30;
 wire [11:0] px=pix_x-64,py=pix_y-192;
 wire [5:0] page_col=px/28;wire [3:0] page_row=py/48;
 wire [5:0] cx=px-page_col*28,cy=py-page_row*48;
 wire page_glyph=cx>=2&&cx<26&&cy>=12&&cy<36;
 wire [5:0] tx=cx-6'd2,ty=cy-6'd12;
 wire [6:0] sx={tx,1'b0},sy={ty,1'b0};
 wire [3:0] page_gx=sx/7'd3,page_gy=sy/7'd3;
 wire [4:0] entry=(page_row-2)*2+(page_col>=20);
 reg [9:0] read_address;
 always @*begin
  read_address=row*60+column;
  if(page_layout)begin
   if(page_row==0)read_address=page_col;
   else if(page_row==1)read_address=page_col<20?40+page_col:10'd480;
   else if(page_row<=10)read_address=(1+entry/3)*60+(entry%3)*20+page_col%20;
   else if(page_row==11)read_address=420+page_col;
   else read_address=page_col<20?460+page_col:10'd480;
  end
 end
 reg [7:0] text_q;reg [3:0] font_row1,gx1,gx2;
 wire [15:0] font_bits;
 imu_font_cn font(.clk(pix_clk),.code(text_q),.row(font_row1),.bits(font_bits));
 reg [23:0] rgb888_d1,rgb888_d2;
 reg [15:0] rgb_d1,rgb_d2;reg panel1,panel2,ready1,ready2,de1,vs1,hs1;
 reg [11:0] x1,y1,x2,y2;
 always @(posedge pix_clk) begin
  text_q<=read_address<480?text_mem[{active_bank,read_address}]:8'd32;
  font_row1<=page_layout?page_gy:local_y[4:1];gx1<=page_layout?page_gx:cell_x[4:1];gx2<=gx1;
  rgb888_d1<=RGB888_INPUT ? rgb888_in : {rgb_in[15:11],rgb_in[15:13],rgb_in[10:5],rgb_in[10:9],rgb_in[4:0],rgb_in[4:2]};
  rgb888_d2<=rgb888_d1;
  rgb_d1<=rgb_in;rgb_d2<=rgb_d1;panel1<=in_panel&&(!page_layout||page_glyph);panel2<=panel1;ready1<=text_ready;ready2<=ready1;
  x1<=pix_x;y1<=pix_y;x2<=x1;y2<=y1;
  de1<=video_de;vs1<=video_vs;hs1<=video_hs;de_out<=de1;vs_out<=vs1;hs_out<=hs1;
 end
 assign out_x=x2;assign out_y=y2;
 always @* begin
  rgb888_out=rgb888_d2;
  if(panel2) begin
   if(ready2 && font_bits[15-gx2]) rgb888_out=(page_layout?(y2<288||y2>=720):y2<HUD_Y0+32)?24'h00ffff:24'hffffff;
  end
  rgb_out=rgb_d2;
  if(panel2 && ready2 && font_bits[15-gx2]) rgb_out=(page_layout?(y2<288||y2>=720):y2<HUD_Y0+32)?16'h07ff:16'hffff;
 end
endmodule
