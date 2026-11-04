`timescale 1ns/1ps
// Momentary KEY4 data page; a small 3D sensor model sits above the original
// numeric HUD. Video, controls and geometry remain immutable during a photo.
module pose_page_overlay #(parameter DEBOUNCE_CYCLES=1485000,LAYOUT_V8=0)(
 input clk,rst_n,key4_n,hold_frame,
 input vs,hs,de,input [11:0] x,y,input [23:0] rgb,
 input [47:0] angles,input [639:0] telemetry,
 output reg page_active,output reg [639:0] page_telemetry,
 output [11:0] out_x,out_y,output out_vs,out_hs,out_de,output reg [23:0] out_rgb
);
 (* ASYNC_REG="TRUE" *) reg k1,k2;
 reg candidate,stable,vs_last;
 reg [20:0] debounce;
 reg start,pose_valid,geometry_valid;
 wire geometry_done,geometry_busy;
 reg [47:0] saved_angles;
 wire [191:0] gx0,gy0,gx1,gy1;
 reg [191:0] x0,y0,x1,y1;
 pose_geometry geometry(.clk(clk),.rst_n(rst_n),.start(start),.angles(saved_angles),
  .line_x0(gx0),.line_y0(gy0),.line_x1(gx1),.line_y1(gy1),.busy(geometry_busy),.done(geometry_done));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin k1<=1;k2<=1;candidate<=1;stable<=1;debounce<=0;vs_last<=0;
   page_active<=0;page_telemetry<=0;saved_angles<=0;start<=0;pose_valid<=0;geometry_valid<=0;x0<=0;y0<=0;x1<=0;y1<=0;end
  else begin
   k1<=key4_n;k2<=k1;vs_last<=vs;start<=0;
   if(k2!=candidate)begin candidate<=k2;debounce<=0;end
   else if(k2==stable)debounce<=0;
   else if(DEBOUNCE_CYCLES<=1 || debounce==DEBOUNCE_CYCLES-1)begin stable<=k2;debounce<=0;end
   else debounce<=debounce+1'b1;
   if(vs&&!vs_last&&!hold_frame)begin
    page_active<=!stable;page_telemetry<=telemetry;pose_valid<=telemetry[593];
    if(!stable&&!geometry_busy)begin saved_angles<=angles;start<=1;geometry_valid<=0;end
   end
   // Geometry completes in blanking, long before the first active pixel.
   // No endpoint update may occur after photo ownership is acknowledged.
   if(geometry_done&&!hold_frame)begin x0<=gx0;y0<=gy0;x1<=gx1;y1<=gy1;geometry_valid<=1;end
  end
 end
 wire [14:0] hit;
 genvar g;
 generate for(g=0;g<15;g=g+1)begin:edge_raster
  wire signed [12:0] ax=$signed({1'b0,x0[g*12+:12]}),ay=$signed({1'b0,y0[g*12+:12]});
  wire signed [12:0] bx=$signed({1'b0,x1[g*12+:12]}),by=$signed({1'b0,y1[g*12+:12]});
  wire signed [12:0] dx=bx-ax,dy=by-ay;
  wire [12:0] adx=dx<0?-dx:dx,ady=dy<0?-dy:dy;
  reg signed [26:0] p,q,tolerance;
  wire signed [27:0] cross=$signed({p[26],p})-$signed({q[26],q});
  reg bounds,edge_hit;
  assign hit[g]=edge_hit;
  always @(posedge clk)begin
   p<=($signed({1'b0,x})-ax)*dy;q<=($signed({1'b0,y})-ay)*dx;
   tolerance<=(adx+ady)*2;
   bounds<=rst_n&&de&&page_active&&pose_valid&&geometry_valid&&
    x>=1232&&x<1888&&y>=144&&y<640&&
    $signed({1'b0,x})>=(ax<bx?ax:bx)-2&&$signed({1'b0,x})<=(ax>bx?ax:bx)+2&&
    $signed({1'b0,y})>=(ay<by?ay:by)-2&&$signed({1'b0,y})<=(ay>by?ay:by)+2;
   edge_hit<=bounds&&cross<=tolerance&&cross>=-tolerance;
  end
 end endgenerate
 // Labels use the same embedded ASCII font as the existing Chinese HUD.
 // Each glyph is 16x16 expanded to 32x32; only one font read port is needed.
 reg [7:0] letter;reg label_area;
 wire [5:0] col=(x-12'd64)>>5;
 wire [5:0] panel_col=(x-12'd1248)>>5;
 always @*begin
  letter=32;label_area=0;
  if(x>=64&&x<768&&y>=64&&y<96)begin
   label_area=1;
   case(col)
    0:letter="I";1:letter="M";2:letter="U";4:letter="D";5:letter="A";6:letter="T";7:letter="A";
    9:letter="/";11:letter="H";12:letter="O";13:letter="L";14:letter="D";
    16:letter="K";17:letter="E";18:letter="Y";19:letter="4";default:letter=32;
   endcase
  end
  if(x>=1248&&x<1888&&y>=96&&y<128)begin
   label_area=1;
   case(panel_col)
    0:letter="S";1:letter="E";2:letter="N";3:letter="S";4:letter="O";5:letter="R";
    7:letter="F";8:letter="R";9:letter="A";10:letter="M";11:letter="E";default:letter=32;
   endcase
  end
  if(x>=1248&&x<1888&&y>=672&&y<704)begin
   label_area=1;
   case(panel_col)
    0:letter=pose_valid?"L":"N";1:letter=pose_valid?"I":"O";2:letter=pose_valid?"V":" ";3:letter=pose_valid?"E":"I";
    4:letter=pose_valid?" ":"M";5:letter=pose_valid?"X":"U";7:letter="Y";9:letter="Z";default:letter=32;
   endcase
  end
 end
 wire [15:0] glyph;
 reg [7:0] letter_q;reg [3:0] row_q,glyph_x1,glyph_x2;
 imu_pose_font label_font(.clk(clk),.code(letter_q),.row(row_q),.bits(glyph));
 reg [23:0] rgb1,rgb2;reg [11:0] xx1,yy1,xx2,yy2;
 reg [2:0] sync1,sync2;reg page1,page2,label1,label2;
 integer i;
 always @(posedge clk)begin
  letter_q<=letter;row_q<=y[4:1];glyph_x1<=x[4:1];glyph_x2<=glyph_x1;
  rgb1<=rgb;rgb2<=rgb1;xx1<=x;yy1<=y;xx2<=xx1;yy2<=yy1;
  sync1<=rst_n?{vs,hs,de}:0;sync2<=sync1;
  page1<=rst_n&&page_active;page2<=page1;label1<=label_area;label2<=label1;
 end
 always @*begin
  out_rgb=rgb2;
  if(page2)begin
   out_rgb=24'h081018;
   if(LAYOUT_V8!=0)begin
    if(xx2>=48&&xx2<1200&&yy2>=176&&yy2<832)begin
     out_rgb=24'h102830;
     if(xx2==48||xx2==1199||yy2==176||yy2==831)out_rgb=24'h287080;
     if(xx2>=622&&xx2<626&&yy2>=288&&yy2<720)out_rgb=24'h205060;
     if(yy2>=288&&yy2<720&&(yy2-288)%48==0)out_rgb=24'h183840;
    end
   end
   if(xx2>=1232&&xx2<1888&&yy2>=144&&yy2<640)begin
    out_rgb=(xx2[5:0]==0||yy2[5:0]==0)?24'h183038:24'h0c2028;
    for(i=0;i<12;i=i+1)if(hit[i])out_rgb=24'h40d8e8;
    if(hit[12])out_rgb=24'hff5050;if(hit[13])out_rgb=24'h50ff70;if(hit[14])out_rgb=24'h6090ff;
   end
   if(label2&&glyph[15-glyph_x2])out_rgb=24'hffffff;
  end
 end
 assign out_x=xx2;assign out_y=yy2;
 assign out_vs=sync2[2];assign out_hs=sync2[1];assign out_de=sync2[0];
endmodule
