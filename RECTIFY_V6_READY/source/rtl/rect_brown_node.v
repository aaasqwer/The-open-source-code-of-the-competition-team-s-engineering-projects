`timescale 1ns/1ps
// New, shared-multiplier Brown evaluator for mesh construction.
// Ideal/source coordinates: signed Q10 pixels. fx/fy/cx/cy: Q16 pixels;
// k1/k2/p1/p2: Q24; reciprocal focal lengths: Q30. No default camera values.
// A single unsigned 16x16 partial-product bank is shared by every operation.
`include "calibration/profile_config.vh"
module rect_brown_node(
 input clk,rst_n,start,enhanced,input signed [23:0] ideal_x,ideal_y,
 input [319:0] calibration,
 output reg busy,done,bad,output reg signed [23:0] source_x,source_y
);
 reg [7:0] state,resume;
 reg enhanced_cfg;
 reg signed [31:0] radial_residual[0:1024];
 reg signed [31:0] residual_lo,residual_hi;
 wire [10:0] lut_index=r2[22:12]<1024?r2[22:12]:1023;
 initial $readmemh("calibration/radial_residual_q24.hex",radial_residual);
 always @(posedge clk)begin
  residual_lo<=radial_residual[lut_index];residual_hi<=radial_residual[lut_index+1'b1];
 end
 // Additional coefficients are normalized displacements, not a substitute
 // lens model. The same multiplier runs two inverse fixed-point iterations
 // only when building an enhanced mesh with the calibrated default profile.
 localparam [383:0] DEFAULT_WORDS=`RECTIFY_DEFAULT_PARAMS;
 reg signed [31:0] poly_coeff[0:19];reg signed [31:0] poly_c;
 initial $readmemh("calibration/nonradial_residual_q24.hex",poly_coeff);
 reg [4:0] poly_address;reg [2:0] poly_group,poly_term;
 reg poly_axis,poly_iteration,poly_enabled;
 reg signed [31:0] poly_x,poly_y,poly_base_x,poly_base_y,poly_row,poly_total,poly_candidate_x;
 always @(posedge clk)poly_c<=poly_coeff[poly_address];
 reg [319:0] cfg;
 reg signed [31:0] nx,ny,xx,yy,xy,r2,rad,work,dx,dy,tx,ty;
 reg signed [31:0] u,v;
 wire signed [31:0] fx=cfg[0+:32],fy=cfg[32+:32],cx=cfg[64+:32],cy=cfg[96+:32];
 wire signed [31:0] k1=cfg[128+:32],k2=cfg[160+:32],p1=cfg[192+:32],p2=cfg[224+:32];
 wire signed [31:0] ifx=cfg[256+:32],ify=cfg[288+:32];
 reg [31:0] ma,mb,ll,lh,hl,hh;
 reg signed [31:0] operand_a,operand_b;
 reg negative;
 reg [63:0] sum_low,sum_full;
 reg signed [63:0] product;
 task multiply;
  input signed [31:0] a,b;input [7:0] next;
  // Select signed operands first; take absolute values in a separate cycle.
  // One shared negate per operand avoids folding many input expressions into
  // the DSP register path. Arithmetic and the multiplier products are exact.
  begin operand_a<=a;operand_b<=b;resume<=next;state<=198;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=0;resume<=0;poly_address<=0;poly_group<=0;poly_term<=0;poly_axis<=0;poly_iteration<=0;poly_enabled<=0;poly_x<=0;poly_y<=0;poly_base_x<=0;poly_base_y<=0;poly_row<=0;poly_total<=0;poly_candidate_x<=0;enhanced_cfg<=0;cfg<=0;busy<=0;done<=0;bad<=0;source_x<=0;source_y<=0;
   nx<=0;ny<=0;xx<=0;yy<=0;xy<=0;r2<=0;rad<=0;work<=0;dx<=0;dy<=0;tx<=0;ty<=0;u<=0;v<=0;
   ma<=0;mb<=0;operand_a<=0;operand_b<=0;ll<=0;lh<=0;hl<=0;hh<=0;negative<=0;sum_low<=0;sum_full<=0;product<=0;
  end else begin
   done<=0;
   case(state)
    0:if(start)begin enhanced_cfg<=enhanced;poly_enabled<=enhanced&&(calibration==DEFAULT_WORDS[319:0]);cfg<=calibration;u<={{8{ideal_x[23]}},ideal_x};v<={{8{ideal_y[23]}},ideal_y};busy<=1;bad<=0;state<=1;end
    1:multiply(u-(cx>>>6),ifx,2);
    2:begin nx<=product>>>20;state<=3;end
    3:multiply(v-(cy>>>6),ify,4);
    4:begin ny<=product>>>20;poly_base_x<=nx;poly_base_y<=product>>>20;state<=poly_enabled?80:5;end
    5:if(nx>4194304||nx< -4194304||ny>4194304||ny< -4194304)begin bad<=1;state<=99;end
      else multiply(nx,nx,6);
    6:begin xx<=product>>>20;state<=7;end
    7:multiply(ny,ny,8);
    8:begin yy<=product>>>20;r2<=xx+(product>>>20);state<=9;end
    9:multiply(nx,ny,10);
    10:begin xy<=product>>>20;state<=11;end
    11:multiply(k2,r2,12);
    12:begin work<=k1+(product>>>20);state<=13;end
    13:multiply(work,r2,14);
    14:begin
     rad<=32'sd16777216+(product>>>20);
     if(enhanced_cfg)begin
      if(r2<0||r2>`RECTIFY_RADIAL_MAX_R2)begin bad<=1;state<=99;end else state<=40;
     end else state<=15;
    end
    40:multiply(residual_hi-residual_lo,$signed({20'd0,r2[11:0]}),41);
    41:begin rad<=rad+residual_lo+(product>>>12);state<=15;end
    15:multiply(nx,rad,16);
    16:begin dx<=product>>>24;state<=17;end
    17:multiply(ny,rad,18);
    18:begin dy<=product>>>24;state<=19;end
    19:multiply(p1,xy<<<1,20);
    20:begin tx<=product>>>24;state<=21;end
    21:multiply(p2,r2+(xx<<<1),22);
    22:begin dx<=dx+tx+(product>>>24);state<=23;end
    23:multiply(p1,r2+(yy<<<1),24);
    24:begin ty<=product>>>24;state<=25;end
    25:multiply(p2,xy<<<1,26);
    26:begin dy<=dy+ty+(product>>>24);state<=27;end
    27:multiply(fx,dx,28);
    28:begin
     if((product>>>26)+(cx>>>6)>8388607||(product>>>26)+(cx>>>6)< -8388608)bad<=1;
     source_x<=(product>>>26)+(cx>>>6);state<=29;
    end
    29:multiply(fy,dy,30);
    30:begin
     if((product>>>26)+(cy>>>6)>8388607||(product>>>26)+(cy>>>6)< -8388608)bad<=1;
     source_y<=(product>>>26)+(cy>>>6);state<=99;
    end
    80:begin
     poly_x<=poly_base_x;poly_y<=poly_base_y;poly_axis<=0;poly_iteration<=0;
     poly_address<=0;poly_group<=0;poly_term<=0;poly_row<=0;poly_total<=0;state<=81;
    end
    81:multiply(poly_row,poly_y,82);
    82:begin
     poly_row<=(product>>>20)+poly_c;poly_address<=poly_address+1'b1;
     if(poly_term==poly_group)state<=83;else begin poly_term<=poly_term+1'b1;state<=81;end
    end
    83:multiply(poly_total,poly_x,84);
    84:begin
     poly_total<=(product>>>20)+poly_row;poly_row<=0;poly_term<=0;
     if(poly_group==3)state<=85;else begin poly_group<=poly_group+1'b1;state<=81;end
    end
    85:begin
     poly_group<=0;poly_term<=0;poly_row<=0;poly_total<=0;
     if(!poly_axis)begin
      poly_candidate_x<=poly_base_x-(poly_total>>>4);poly_axis<=1;poly_address<=10;state<=81;
     end else if(!poly_iteration)begin
      poly_x<=poly_candidate_x;poly_y<=poly_base_y-(poly_total>>>4);
      poly_iteration<=1;poly_axis<=0;poly_address<=0;state<=81;
     end else begin nx<=poly_candidate_x;ny<=poly_base_y-(poly_total>>>4);state<=5;end
    end
    99:begin busy<=0;done<=1;state<=0;end
    198:begin ma<=operand_a[31]?-operand_a:operand_a;mb<=operand_b[31]?-operand_b:operand_b;negative<=operand_a[31]^operand_b[31];state<=200;end
    200:begin ll<=ma[15:0]*mb[15:0];lh<=ma[15:0]*mb[31:16];hl<=ma[31:16]*mb[15:0];hh<=ma[31:16]*mb[31:16];state<=201;end
    201:begin sum_low<={32'd0,ll}+{16'd0,lh,16'd0};state<=202;end
    202:begin sum_full<=sum_low+{16'd0,hl,16'd0}+{hh,32'd0};state<=203;end
    203:begin product<=negative?-$signed(sum_full):$signed(sum_full);state<=resume;end
    default:begin busy<=0;bad<=1;done<=1;state<=0;end
   endcase
  end
 end
endmodule
