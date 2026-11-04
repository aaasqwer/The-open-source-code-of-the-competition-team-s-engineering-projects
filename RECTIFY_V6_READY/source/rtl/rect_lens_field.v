`timescale 1ns/1ps
// New coordinate mapping core: construct the required Brown model on-chip with
// a shared multiplier, then stream inverse-sampling coordinates from four
// parity RAMs. Grid interval 8px by default; displacements signed Q8 (18 bits/axis).
// Keeping displacement rather than absolute position saves block RAM. A model
// whose node displacement exceeds +/-512px fails boot, never wraps silently.
// Input/output pixels Q12.
// Camera parameters have no defaults. Rebuild only while the consumer holds
// a completed frame. This module is not a DDR controller or a camera top.
module rect_lens_field #(parameter W=1920,H=1080,GRID_SHIFT=3)(
 input clk,rst_n,boot_start,enhanced,input [319:0] calibration,
 output reg ready,fault,output busy,
 input in_valid,input signed [25:0] ideal_x,ideal_y,
 output reg out_valid,out_bad,output reg signed [25:0] source_x,source_y,
 output source_inside
);
 localparam GRID=1<<GRID_SHIFT;
 localparam COLS=(W+GRID-1)/GRID+1,ROWS=(H+GRID-1)/GRID+1;
 localparam GB=$clog2((COLS>ROWS?COLS:ROWS)+1);
 localparam HALF_COLS=(COLS+1)/2,HALF_ROWS=(ROWS+1)/2,DEPTH=HALF_COLS*HALF_ROWS;
 // Native 8px mesh needs 8228 words per parity RAM: only 36 exceed 8K.
 // Keep that small tail in LUT RAM instead of doubling all four block RAMs.
 localparam SPILL=(DEPTH>8192&&DEPTH<=8320),MAIN_DEPTH=SPILL?8192:DEPTH;
 localparam TAIL_DEPTH=SPILL?DEPTH-8192:1,MAW=$clog2(MAIN_DEPTH),TAW=$clog2(TAIL_DEPTH<2?2:TAIL_DEPTH);
 reg [35:0] m00[0:MAIN_DEPTH-1],m01[0:MAIN_DEPTH-1],m10[0:MAIN_DEPTH-1],m11[0:MAIN_DEPTH-1];
 reg [35:0] t00[0:TAIL_DEPTH-1],t01[0:TAIL_DEPTH-1],t10[0:TAIL_DEPTH-1],t11[0:TAIL_DEPTH-1] /* synthesis syn_ramstyle="distributed" */;
 reg [319:0] cfg;reg enhanced_cfg;
 reg [GB-1:0] gx,gy;
 reg [1:0] state;
 reg engine_start;
 wire engine_busy,engine_done,engine_bad;
 wire signed [23:0] nx,ny;
 wire signed [23:0] node_u=$signed({1'b0,gx})<<<(GRID_SHIFT+10);
 wire signed [23:0] node_v=$signed({1'b0,gy})<<<(GRID_SHIFT+10);
 wire signed [24:0] displacement_x=$signed({nx[23],nx})-$signed({node_u[23],node_u});
 wire signed [24:0] displacement_y=$signed({ny[23],ny})-$signed({node_v[23],node_v});
 wire signed [24:0] rounded_x=(displacement_x+25'sd2)>>>2;
 wire signed [24:0] rounded_y=(displacement_y+25'sd2)>>>2;
 wire displacement_bad=rounded_x>131071||rounded_x< -131072||rounded_y>131071||rounded_y< -131072;
 rect_brown_node evaluator(.enhanced(enhanced_cfg),.clk(clk),.rst_n(rst_n),.start(engine_start),.ideal_x(node_u),.ideal_y(node_v),
  .calibration(cfg),.busy(engine_busy),.done(engine_done),.bad(engine_bad),.source_x(nx),.source_y(ny));
 assign busy=state!=0;
 wire [$clog2(DEPTH)-1:0] write_address=(gy>>1)*HALF_COLS+(gx>>1);
 always @(posedge clk)if(state==2&&engine_done&&!engine_bad&&!displacement_bad)begin
  if(SPILL&&write_address>=8192)begin
   case({gy[0],gx[0]})
    0:t00[write_address[TAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};1:t01[write_address[TAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};
    2:t10[write_address[TAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};3:t11[write_address[TAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};
   endcase
  end else begin
   case({gy[0],gx[0]})
    0:m00[write_address[MAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};1:m01[write_address[MAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};
    2:m10[write_address[MAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};3:m11[write_address[MAW-1:0]]<={rounded_x[17:0],rounded_y[17:0]};
   endcase
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin enhanced_cfg<=0;cfg<=0;gx<=0;gy<=0;state<=0;engine_start<=0;ready<=0;fault<=0;end
  else begin
   engine_start<=0;
   case(state)
    0:if(boot_start)begin enhanced_cfg<=enhanced;cfg<=calibration;gx<=0;gy<=0;ready<=0;fault<=0;state<=1;end
    1:if(!engine_busy)begin engine_start<=1;state<=2;end
    2:if(engine_done)begin
     if(engine_bad||displacement_bad)begin fault<=1;ready<=0;state<=0;end
     else if(gx==COLS-1)begin
      gx<=0;if(gy==ROWS-1)begin ready<=1;state<=0;end else begin gy<=gy+1'b1;state<=1;end
     end else begin gx<=gx+1'b1;state<=1;end
    end
    default:begin state<=0;ready<=0;fault<=1;end
   endcase
  end
 end
 wire domain_ok=ideal_x>=0&&ideal_x<=(W-1)*4096&&ideal_y>=0&&ideal_y<=(H-1)*4096;
 wire [GB-1:0] ix=domain_ok?(ideal_x>>>(GRID_SHIFT+12)):0,iy=domain_ok?(ideal_y>>>(GRID_SHIFT+12)):0;
 reg [GB-1:0] ix_registered,iy_registered;
 always @(posedge clk)begin ix_registered<=ix;iy_registered<=iy;end
 wire [GB-1:0] ex=ix_registered[0]?ix_registered+1'b1:ix_registered,ox=ix_registered[0]?ix_registered:ix_registered+1'b1;
 wire [GB-1:0] ey=iy_registered[0]?iy_registered+1'b1:iy_registered,oy=iy_registered[0]?iy_registered:iy_registered+1'b1;
 wire [$clog2(DEPTH)-1:0] a00=(ey>>1)*HALF_COLS+(ex>>1),a01=(ey>>1)*HALF_COLS+(ox>>1);
 wire [$clog2(DEPTH)-1:0] a10=(oy>>1)*HALF_COLS+(ex>>1),a11=(oy>>1)*HALF_COLS+(ox>>1);
 reg [$clog2(DEPTH)-1:0] ra00,ra01,ra10,ra11;
 always @(posedge clk)begin ra00<=a00;ra01<=a01;ra10<=a10;ra11<=a11;end
 reg [35:0] q00,q01,q10,q11,h00,h01,h10,h11;reg [3:0] high_select;
 wire [35:0] r00=high_select[0]?h00:q00,r01=high_select[1]?h01:q01;
 wire [35:0] r10=high_select[2]?h10:q10,r11=high_select[3]?h11:q11;
 reg signed [25:0] ideal_xpre,ideal_ypre,ideal_x0,ideal_y0,ideal_x1,ideal_y1,ideal_x2,ideal_y2,ideal_x3,ideal_y3,ideal_x4,ideal_y4;
 reg signed [25:0] vertical_x4,vertical_y4;
 reg x_oddpre,y_oddpre,vpre,badpre,x_odd0,y_odd0,x_odd,y_odd,v0,v1,v2,v3,v4,bad0,bad1,bad2,bad3,bad4;
 reg [11:0] fxpre,fypre,fx0,fy0,fx1,fx2,fy1,fy2,fy3;
 wire [35:0] a=y_odd?(x_odd?r11:r10):(x_odd?r01:r00);
 wire [35:0] b=y_odd?(x_odd?r10:r11):(x_odd?r00:r01);
 wire [35:0] c=y_odd?(x_odd?r01:r00):(x_odd?r11:r10);
 wire [35:0] d=y_odd?(x_odd?r00:r01):(x_odd?r10:r11);
 function signed [17:0] horizontal;
  input signed [17:0] first,last;input [11:0] f;
  reg signed [18:0] delta;
  reg signed [31:0] sum;
  begin delta=$signed({last[17],last})-$signed({first[17],first});
   sum=($signed({{14{first[17]}},first})<<<12)+delta*$signed({1'b0,f})+32'sd2048;
   horizontal=sum>>>12;end
 endfunction
 function signed [25:0] vertical;
  input signed [17:0] first,last;input [11:0] f;
  reg signed [18:0] delta;
  reg signed [31:0] sum;
  begin delta=$signed({last[17],last})-$signed({first[17],first});
   sum=($signed({{14{first[17]}},first})<<<12)+delta*$signed({1'b0,f})+32'sd128;
   vertical=sum>>>8;end
 endfunction
 // Separate parity/tail RAM selection from the interpolation DSP input.
 reg [35:0] a1,b1,c1,d1;
 always @(posedge clk)begin a1<=a;b1<=b;c1<=c;d1<=d;end
 reg signed [17:0] xt,xb,yt,yb;
 always @(posedge clk)begin
  q00<=m00[ra00[MAW-1:0]];q01<=m01[ra01[MAW-1:0]];q10<=m10[ra10[MAW-1:0]];q11<=m11[ra11[MAW-1:0]];
  if(SPILL)begin
   h00<=t00[ra00[TAW-1:0]];h01<=t01[ra01[TAW-1:0]];h10<=t10[ra10[TAW-1:0]];h11<=t11[ra11[TAW-1:0]];
  end
  high_select<=SPILL?{(ra11>=8192),(ra10>=8192),(ra01>=8192),(ra00>=8192)}:4'd0;
  xt<=horizontal($signed(a1[35:18]),$signed(b1[35:18]),fx2);xb<=horizontal($signed(c1[35:18]),$signed(d1[35:18]),fx2);
  yt<=horizontal($signed(a1[17:0]),$signed(b1[17:0]),fx2);yb<=horizontal($signed(c1[17:0]),$signed(d1[17:0]),fx2);
  ideal_xpre<=ideal_x;ideal_ypre<=ideal_y;ideal_x0<=ideal_xpre;ideal_y0<=ideal_ypre;ideal_x1<=ideal_x0;ideal_y1<=ideal_y0;ideal_x2<=ideal_x1;ideal_y2<=ideal_y1;ideal_x3<=ideal_x2;ideal_y3<=ideal_y2;
  // Keep the existing Q12 arithmetic; separate the DSP output from
  // the ideal-coordinate carry chain. One extra cycle, one coordinate/cycle.
  ideal_x4<=ideal_x3;ideal_y4<=ideal_y3;
  vertical_x4<=vertical(xt,xb,fy3);vertical_y4<=vertical(yt,yb,fy3);
  source_x<=ideal_x4+vertical_x4;source_y<=ideal_y4+vertical_y4;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x_oddpre<=0;y_oddpre<=0;vpre<=0;badpre<=0;fxpre<=0;fypre<=0;x_odd0<=0;y_odd0<=0;v0<=0;bad0<=0;fx0<=0;fy0<=0;x_odd<=0;y_odd<=0;v1<=0;v2<=0;v3<=0;v4<=0;out_valid<=0;bad1<=0;bad2<=0;bad3<=0;bad4<=0;out_bad<=0;fx1<=0;fy1<=0;fy2<=0;fy3<=0;fx2<=0;end
  else if(boot_start&&!busy)begin vpre<=0;badpre<=0;v0<=0;v1<=0;v2<=0;v3<=0;v4<=0;out_valid<=0;bad0<=0;bad1<=0;bad2<=0;bad3<=0;bad4<=0;out_bad<=0;end
  else begin
   x_oddpre<=ix[0];y_oddpre<=iy[0];x_odd0<=x_oddpre;y_odd0<=y_oddpre;x_odd<=x_odd0;y_odd<=y_odd0;
   fxpre<=ideal_x[GRID_SHIFT+11:GRID_SHIFT];fypre<=ideal_y[GRID_SHIFT+11:GRID_SHIFT];fx0<=fxpre;fy0<=fypre;fx1<=fx0;fx2<=fx1;fy1<=fy0;fy2<=fy1;fy3<=fy2;
   vpre<=in_valid&&ready;v0<=vpre;v1<=v0;v2<=v1;v3<=v2;v4<=v3;out_valid<=v4;
   badpre<=!domain_ok||fault;bad0<=badpre;bad1<=bad0;bad2<=bad1;bad3<=bad2;bad4<=bad3;out_bad<=bad4;
  end
 end
 assign source_inside=source_x>=0&&source_x<=(W-1)*4096&&source_y>=0&&source_y<=(H-1)*4096;
endmodule
