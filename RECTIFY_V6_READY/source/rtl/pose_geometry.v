`timescale 1ns/1ps
// Z-Y-X sensor-body rotation, followed by a fixed oblique screen projection.
// The visualization uses a Q14 sine ROM (1024 phase bins, 0.3515625 degrees per bin). Telemetry keeps full raw precision.
module pose_geometry(
 input clk,rst_n,start,input [47:0] angles,
 output reg [191:0] line_x0,line_y0,line_x1,line_y1,
 output reg busy,done
);
  function signed [15:0] sin_lut;
 input [9:0] phase;
 reg [8:0] index;reg signed [15:0] magnitude;
 begin index=phase[8]?9'd256-{1'b0,phase[7:0]}:{1'b0,phase[7:0]};case(index)
 9'd0:magnitude=16'sd0;
 9'd1:magnitude=16'sd101;
 9'd2:magnitude=16'sd201;
 9'd3:magnitude=16'sd302;
 9'd4:magnitude=16'sd402;
 9'd5:magnitude=16'sd503;
 9'd6:magnitude=16'sd603;
 9'd7:magnitude=16'sd704;
 9'd8:magnitude=16'sd804;
 9'd9:magnitude=16'sd904;
 9'd10:magnitude=16'sd1005;
 9'd11:magnitude=16'sd1105;
 9'd12:magnitude=16'sd1205;
 9'd13:magnitude=16'sd1306;
 9'd14:magnitude=16'sd1406;
 9'd15:magnitude=16'sd1506;
 9'd16:magnitude=16'sd1606;
 9'd17:magnitude=16'sd1706;
 9'd18:magnitude=16'sd1806;
 9'd19:magnitude=16'sd1906;
 9'd20:magnitude=16'sd2006;
 9'd21:magnitude=16'sd2105;
 9'd22:magnitude=16'sd2205;
 9'd23:magnitude=16'sd2305;
 9'd24:magnitude=16'sd2404;
 9'd25:magnitude=16'sd2503;
 9'd26:magnitude=16'sd2603;
 9'd27:magnitude=16'sd2702;
 9'd28:magnitude=16'sd2801;
 9'd29:magnitude=16'sd2900;
 9'd30:magnitude=16'sd2999;
 9'd31:magnitude=16'sd3098;
 9'd32:magnitude=16'sd3196;
 9'd33:magnitude=16'sd3295;
 9'd34:magnitude=16'sd3393;
 9'd35:magnitude=16'sd3492;
 9'd36:magnitude=16'sd3590;
 9'd37:magnitude=16'sd3688;
 9'd38:magnitude=16'sd3786;
 9'd39:magnitude=16'sd3883;
 9'd40:magnitude=16'sd3981;
 9'd41:magnitude=16'sd4078;
 9'd42:magnitude=16'sd4176;
 9'd43:magnitude=16'sd4273;
 9'd44:magnitude=16'sd4370;
 9'd45:magnitude=16'sd4467;
 9'd46:magnitude=16'sd4563;
 9'd47:magnitude=16'sd4660;
 9'd48:magnitude=16'sd4756;
 9'd49:magnitude=16'sd4852;
 9'd50:magnitude=16'sd4948;
 9'd51:magnitude=16'sd5044;
 9'd52:magnitude=16'sd5139;
 9'd53:magnitude=16'sd5235;
 9'd54:magnitude=16'sd5330;
 9'd55:magnitude=16'sd5425;
 9'd56:magnitude=16'sd5520;
 9'd57:magnitude=16'sd5614;
 9'd58:magnitude=16'sd5708;
 9'd59:magnitude=16'sd5803;
 9'd60:magnitude=16'sd5897;
 9'd61:magnitude=16'sd5990;
 9'd62:magnitude=16'sd6084;
 9'd63:magnitude=16'sd6177;
 9'd64:magnitude=16'sd6270;
 9'd65:magnitude=16'sd6363;
 9'd66:magnitude=16'sd6455;
 9'd67:magnitude=16'sd6547;
 9'd68:magnitude=16'sd6639;
 9'd69:magnitude=16'sd6731;
 9'd70:magnitude=16'sd6823;
 9'd71:magnitude=16'sd6914;
 9'd72:magnitude=16'sd7005;
 9'd73:magnitude=16'sd7096;
 9'd74:magnitude=16'sd7186;
 9'd75:magnitude=16'sd7276;
 9'd76:magnitude=16'sd7366;
 9'd77:magnitude=16'sd7456;
 9'd78:magnitude=16'sd7545;
 9'd79:magnitude=16'sd7635;
 9'd80:magnitude=16'sd7723;
 9'd81:magnitude=16'sd7812;
 9'd82:magnitude=16'sd7900;
 9'd83:magnitude=16'sd7988;
 9'd84:magnitude=16'sd8076;
 9'd85:magnitude=16'sd8163;
 9'd86:magnitude=16'sd8250;
 9'd87:magnitude=16'sd8337;
 9'd88:magnitude=16'sd8423;
 9'd89:magnitude=16'sd8509;
 9'd90:magnitude=16'sd8595;
 9'd91:magnitude=16'sd8680;
 9'd92:magnitude=16'sd8765;
 9'd93:magnitude=16'sd8850;
 9'd94:magnitude=16'sd8935;
 9'd95:magnitude=16'sd9019;
 9'd96:magnitude=16'sd9102;
 9'd97:magnitude=16'sd9186;
 9'd98:magnitude=16'sd9269;
 9'd99:magnitude=16'sd9352;
 9'd100:magnitude=16'sd9434;
 9'd101:magnitude=16'sd9516;
 9'd102:magnitude=16'sd9598;
 9'd103:magnitude=16'sd9679;
 9'd104:magnitude=16'sd9760;
 9'd105:magnitude=16'sd9841;
 9'd106:magnitude=16'sd9921;
 9'd107:magnitude=16'sd10001;
 9'd108:magnitude=16'sd10080;
 9'd109:magnitude=16'sd10159;
 9'd110:magnitude=16'sd10238;
 9'd111:magnitude=16'sd10316;
 9'd112:magnitude=16'sd10394;
 9'd113:magnitude=16'sd10471;
 9'd114:magnitude=16'sd10549;
 9'd115:magnitude=16'sd10625;
 9'd116:magnitude=16'sd10702;
 9'd117:magnitude=16'sd10778;
 9'd118:magnitude=16'sd10853;
 9'd119:magnitude=16'sd10928;
 9'd120:magnitude=16'sd11003;
 9'd121:magnitude=16'sd11077;
 9'd122:magnitude=16'sd11151;
 9'd123:magnitude=16'sd11224;
 9'd124:magnitude=16'sd11297;
 9'd125:magnitude=16'sd11370;
 9'd126:magnitude=16'sd11442;
 9'd127:magnitude=16'sd11514;
 9'd128:magnitude=16'sd11585;
 9'd129:magnitude=16'sd11656;
 9'd130:magnitude=16'sd11727;
 9'd131:magnitude=16'sd11797;
 9'd132:magnitude=16'sd11866;
 9'd133:magnitude=16'sd11935;
 9'd134:magnitude=16'sd12004;
 9'd135:magnitude=16'sd12072;
 9'd136:magnitude=16'sd12140;
 9'd137:magnitude=16'sd12207;
 9'd138:magnitude=16'sd12274;
 9'd139:magnitude=16'sd12340;
 9'd140:magnitude=16'sd12406;
 9'd141:magnitude=16'sd12472;
 9'd142:magnitude=16'sd12537;
 9'd143:magnitude=16'sd12601;
 9'd144:magnitude=16'sd12665;
 9'd145:magnitude=16'sd12729;
 9'd146:magnitude=16'sd12792;
 9'd147:magnitude=16'sd12854;
 9'd148:magnitude=16'sd12916;
 9'd149:magnitude=16'sd12978;
 9'd150:magnitude=16'sd13039;
 9'd151:magnitude=16'sd13100;
 9'd152:magnitude=16'sd13160;
 9'd153:magnitude=16'sd13219;
 9'd154:magnitude=16'sd13279;
 9'd155:magnitude=16'sd13337;
 9'd156:magnitude=16'sd13395;
 9'd157:magnitude=16'sd13453;
 9'd158:magnitude=16'sd13510;
 9'd159:magnitude=16'sd13567;
 9'd160:magnitude=16'sd13623;
 9'd161:magnitude=16'sd13678;
 9'd162:magnitude=16'sd13733;
 9'd163:magnitude=16'sd13788;
 9'd164:magnitude=16'sd13842;
 9'd165:magnitude=16'sd13896;
 9'd166:magnitude=16'sd13949;
 9'd167:magnitude=16'sd14001;
 9'd168:magnitude=16'sd14053;
 9'd169:magnitude=16'sd14104;
 9'd170:magnitude=16'sd14155;
 9'd171:magnitude=16'sd14206;
 9'd172:magnitude=16'sd14256;
 9'd173:magnitude=16'sd14305;
 9'd174:magnitude=16'sd14354;
 9'd175:magnitude=16'sd14402;
 9'd176:magnitude=16'sd14449;
 9'd177:magnitude=16'sd14497;
 9'd178:magnitude=16'sd14543;
 9'd179:magnitude=16'sd14589;
 9'd180:magnitude=16'sd14635;
 9'd181:magnitude=16'sd14680;
 9'd182:magnitude=16'sd14724;
 9'd183:magnitude=16'sd14768;
 9'd184:magnitude=16'sd14811;
 9'd185:magnitude=16'sd14854;
 9'd186:magnitude=16'sd14896;
 9'd187:magnitude=16'sd14937;
 9'd188:magnitude=16'sd14978;
 9'd189:magnitude=16'sd15019;
 9'd190:magnitude=16'sd15059;
 9'd191:magnitude=16'sd15098;
 9'd192:magnitude=16'sd15137;
 9'd193:magnitude=16'sd15175;
 9'd194:magnitude=16'sd15213;
 9'd195:magnitude=16'sd15250;
 9'd196:magnitude=16'sd15286;
 9'd197:magnitude=16'sd15322;
 9'd198:magnitude=16'sd15357;
 9'd199:magnitude=16'sd15392;
 9'd200:magnitude=16'sd15426;
 9'd201:magnitude=16'sd15460;
 9'd202:magnitude=16'sd15493;
 9'd203:magnitude=16'sd15525;
 9'd204:magnitude=16'sd15557;
 9'd205:magnitude=16'sd15588;
 9'd206:magnitude=16'sd15619;
 9'd207:magnitude=16'sd15649;
 9'd208:magnitude=16'sd15679;
 9'd209:magnitude=16'sd15707;
 9'd210:magnitude=16'sd15736;
 9'd211:magnitude=16'sd15763;
 9'd212:magnitude=16'sd15791;
 9'd213:magnitude=16'sd15817;
 9'd214:magnitude=16'sd15843;
 9'd215:magnitude=16'sd15868;
 9'd216:magnitude=16'sd15893;
 9'd217:magnitude=16'sd15917;
 9'd218:magnitude=16'sd15941;
 9'd219:magnitude=16'sd15964;
 9'd220:magnitude=16'sd15986;
 9'd221:magnitude=16'sd16008;
 9'd222:magnitude=16'sd16029;
 9'd223:magnitude=16'sd16049;
 9'd224:magnitude=16'sd16069;
 9'd225:magnitude=16'sd16088;
 9'd226:magnitude=16'sd16107;
 9'd227:magnitude=16'sd16125;
 9'd228:magnitude=16'sd16143;
 9'd229:magnitude=16'sd16160;
 9'd230:magnitude=16'sd16176;
 9'd231:magnitude=16'sd16192;
 9'd232:magnitude=16'sd16207;
 9'd233:magnitude=16'sd16221;
 9'd234:magnitude=16'sd16235;
 9'd235:magnitude=16'sd16248;
 9'd236:magnitude=16'sd16261;
 9'd237:magnitude=16'sd16273;
 9'd238:magnitude=16'sd16284;
 9'd239:magnitude=16'sd16295;
 9'd240:magnitude=16'sd16305;
 9'd241:magnitude=16'sd16315;
 9'd242:magnitude=16'sd16324;
 9'd243:magnitude=16'sd16332;
 9'd244:magnitude=16'sd16340;
 9'd245:magnitude=16'sd16347;
 9'd246:magnitude=16'sd16353;
 9'd247:magnitude=16'sd16359;
 9'd248:magnitude=16'sd16364;
 9'd249:magnitude=16'sd16369;
 9'd250:magnitude=16'sd16373;
 9'd251:magnitude=16'sd16376;
 9'd252:magnitude=16'sd16379;
 9'd253:magnitude=16'sd16381;
 9'd254:magnitude=16'sd16383;
 9'd255:magnitude=16'sd16384;
 9'd256:magnitude=16'sd16384;
 default:magnitude=0;endcase sin_lut=phase[9]?-magnitude:magnitude;end endfunction
 // Frame-rate work is serialized inside vertical blanking. One sine lookup
 // and one signed 16x16 multiplier replace six lookups/twelve multipliers.
 // Keep every original Q14 truncation between Rx, Ry and Rz unchanged.
 localparam IDLE=0,TRIG=1,LOAD_POINT=2,
  ROLL_Y_A=3,ROLL_Y_B=4,ROLL_Z_A=5,ROLL_Z_B=6,
  PITCH_X_A=7,PITCH_X_B=8,PITCH_Z_A=9,PITCH_Z_B=10,
  YAW_X_A=11,YAW_X_B=12,YAW_Y_A=13,YAW_Y_B=14,PROJECT=15,EMIT=16;
 reg [4:0] state;
 reg [2:0] trig_index;
 reg [29:0] saved_phases;
 reg [9:0] trig_phase;
 reg signed [15:0] sr,cr,sp,cp,sy,cy;
 // All points have norm <=220. Three Q14 rotations with floor errors keep
 // norm <227 for every ROM phase (exact bound in test_resource_optimization).
 // Signed 9-bit coordinates therefore remove only redundant sign bits.
 reg signed [8:0] xx,yy,zz,yr,zr,xp,zp,xf,yf;
 wire signed [8:0] xr=xx,yp=yr,zf=zp;
 reg signed [8:0] mul_a;
 reg signed [15:0] mul_b;
 reg signed [24:0] accumulator;
 wire signed [15:0] xf16={{7{xf[8]}},xf},yf16={{7{yf[8]}},yf};
 wire signed [31:0] product=mul_a*mul_b;
 wire signed [15:0] trig_value=sin_lut(trig_phase);
 always @*begin
  case(trig_index)
   0:trig_phase=saved_phases[9:0];1:trig_phase=saved_phases[9:0]+10'd256;
   2:trig_phase=saved_phases[19:10];3:trig_phase=saved_phases[19:10]+10'd256;
   4:trig_phase=saved_phases[29:20];default:trig_phase=saved_phases[29:20]+10'd256;
  endcase
  mul_a=0;mul_b=0;
  case(state)
   ROLL_Y_A:begin mul_a=yy;mul_b=cr;end
   ROLL_Y_B:begin mul_a=zz;mul_b=sr;end
   ROLL_Z_A:begin mul_a=yy;mul_b=sr;end
   ROLL_Z_B:begin mul_a=zz;mul_b=cr;end
   PITCH_X_A:begin mul_a=xr;mul_b=cp;end
   PITCH_X_B:begin mul_a=zr;mul_b=sp;end
   PITCH_Z_A:begin mul_a=xr;mul_b=sp;end
   PITCH_Z_B:begin mul_a=zr;mul_b=cp;end
   YAW_X_A:begin mul_a=xp;mul_b=cy;end
   YAW_X_B:begin mul_a=yp;mul_b=sy;end
   YAW_Y_A:begin mul_a=xp;mul_b=sy;end
   YAW_Y_B:begin mul_a=yp;mul_b=cy;end
   default:begin mul_a=0;mul_b=0;end
  endcase
 end
 wire signed [31:0] needle_x=cy*32'sd150;
 wire signed [31:0] needle_y=sy*32'sd150;
 reg signed [11:0] px[0:11],py[0:11];
 reg [3:0] point;
 integer i;
 function integer edge_a;
 input integer n;
 begin case(n)
  0:edge_a=0;1:edge_a=1;2:edge_a=2;3:edge_a=3;
  4:edge_a=4;5:edge_a=5;6:edge_a=6;7:edge_a=7;
  8:edge_a=0;9:edge_a=1;10:edge_a=2;11:edge_a=3;
  default:edge_a=8;
 endcase end endfunction
 function integer edge_b;
 input integer n;
 begin case(n)
  0:edge_b=1;1:edge_b=2;2:edge_b=3;3:edge_b=0;
  4:edge_b=5;5:edge_b=6;6:edge_b=7;7:edge_b=4;
  8:edge_b=4;9:edge_b=5;10:edge_b=6;11:edge_b=7;
  12:edge_b=9;13:edge_b=10;default:edge_b=11;
 endcase end endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   sr<=0;cr<=16384;sp<=0;cp<=16384;sy<=0;cy<=16384;
   xx<=0;yy<=0;zz<=0;yr<=0;zr<=0;xp<=0;zp<=0;xf<=0;yf<=0;
   point<=0;state<=IDLE;busy<=0;done<=0;line_x0<=0;line_y0<=0;line_x1<=0;line_y1<=0;
   trig_index<=0;saved_phases<=0;accumulator<=0;
   for(i=0;i<12;i=i+1) begin px[i]<=0;py[i]<=0;end
  end else begin
   done<=0;
   case(state)
    IDLE:if(start) begin
     saved_phases<={angles[47:38],angles[31:22],angles[15:6]};trig_index<=0;point<=0;busy<=1;state<=TRIG;
    end
    TRIG:begin
     case(trig_index)
      0:sr<=trig_value;1:cr<=trig_value;2:sp<=trig_value;
      3:cp<=trig_value;4:sy<=trig_value;default:cy<=trig_value;
     endcase
     if(trig_index==5)state<=LOAD_POINT;else trig_index<=trig_index+1'b1;
    end
    LOAD_POINT:begin
     case(point)
      0:begin xx<=-150;yy<=-90;zz<=-40;end
      1:begin xx<=150;yy<=-90;zz<=-40;end
      2:begin xx<=150;yy<=90;zz<=-40;end
      3:begin xx<=-150;yy<=90;zz<=-40;end
      4:begin xx<=-150;yy<=-90;zz<=40;end
      5:begin xx<=150;yy<=-90;zz<=40;end
      6:begin xx<=150;yy<=90;zz<=40;end
      7:begin xx<=-150;yy<=90;zz<=40;end
      8:begin xx<=0;yy<=0;zz<=0;end
      9:begin xx<=220;yy<=0;zz<=0;end
      10:begin xx<=0;yy<=200;zz<=0;end
      default:begin xx<=0;yy<=0;zz<=180;end
     endcase
     state<=ROLL_Y_A;
    end
    ROLL_Y_A:begin accumulator<=product;state<=ROLL_Y_B;end
    ROLL_Y_B:begin yr<=(accumulator-product)>>>14;state<=ROLL_Z_A;end
    ROLL_Z_A:begin accumulator<=product;state<=ROLL_Z_B;end
    ROLL_Z_B:begin zr<=(accumulator+product)>>>14;state<=PITCH_X_A;end
    PITCH_X_A:begin accumulator<=product;state<=PITCH_X_B;end
    PITCH_X_B:begin xp<=(accumulator+product)>>>14;state<=PITCH_Z_A;end
    PITCH_Z_A:begin accumulator<=product;state<=PITCH_Z_B;end
    PITCH_Z_B:begin zp<=(-accumulator+product)>>>14;state<=YAW_X_A;end
    YAW_X_A:begin accumulator<=product;state<=YAW_X_B;end
    YAW_X_B:begin xf<=(accumulator-product)>>>14;state<=YAW_Y_A;end
    YAW_Y_A:begin accumulator<=product;state<=YAW_Y_B;end
    YAW_Y_B:begin yf<=(accumulator+product)>>>14;state<=PROJECT;end
    PROJECT:begin
     px[point]<=12'sd1552+xf16-(yf16>>>1);
     py[point]<=12'sd392-zf-((xf16+yf16)>>>2);
     if(point==11) state<=EMIT;else begin point<=point+1'b1;state<=LOAD_POINT;end
    end
    EMIT:begin
     for(i=0;i<15;i=i+1) begin
      line_x0[i*12 +:12]<=px[edge_a(i)];line_y0[i*12 +:12]<=py[edge_a(i)];
      line_x1[i*12 +:12]<=px[edge_b(i)];line_y1[i*12 +:12]<=py[edge_b(i)];
     end
     line_x0[191:180]<=1450;line_y0[191:180]<=390;
     line_x1[191:180]<=32'sd1450+(needle_x>>>14);
     line_y1[191:180]<=32'sd390-(needle_y>>>14);
     busy<=0;done<=1;state<=IDLE;
    end
    default:begin state<=IDLE;busy<=0;end
   endcase
  end
 end
endmodule
