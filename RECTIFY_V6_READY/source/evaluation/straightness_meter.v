`timescale 1ns/1ps
// Per-frame local straightness, x(y) least squares. Perpendicular residuals.
// Input localization: parabolic interpolation of 3 box-gradient samples, Q8.
// Shared 16x16 partial multiplier, restoring divider, and integer square root.
module straightness_meter(input clk,rst_n,cancel,frame_start,frame_ok,
 input [8:0] min_rows,input point_we,input [7:0] point_y,input [6:0] point_x,input [9:0] peak_l,peak_c,peak_r,
 input [7:0] display_y,output [7:0] fitted_x,
 output reg valid,output reg [15:0] rms_q8,max_q8,output reg [8:0] used_rows,output reg [6:0] worst_x,output reg [7:0] worst_y,input [7:0] curve_y,output signed [15:0] curve_q8,output curve_valid,output signed [31:0] fit_slope_q16,fit_intercept_q16);
 reg [7:0] state,resume_mul,resume_div,resume_sqrt;
 reg [15:0] points[0:255];reg [255:0] point_valid;
 reg signed [15:0] curve[0:255];reg [255:0] curve_mask;
 assign curve_q8=curve[curve_y];assign curve_valid=valid&&curve_mask[curve_y];
 reg [7:0] fitted[0:255];assign fitted_x=fitted[display_y];
 reg [8:0] count,idx;reg [7:0] pending_y;reg [15:0] pending_x;
 reg [31:0] sx,sy,syy,sxy;
 reg signed [63:0] t0,t1,denominator,numerator,intercept_num;
 reg signed [31:0] slope,intercept;
 assign fit_slope_q16=slope;assign fit_intercept_q16=intercept;
 reg [31:0] norm_q16;
 reg signed [63:0] predicted;
 wire signed [63:0] signed_residual=($signed({1'b0,points[idx[7:0]],8'b0})-predicted)>>>8;

 reg [31:0] residual;reg [63:0] sum_squares;reg [31:0] largest;
 reg [15:0] pending_rms;reg [6:0] peak_x;reg [7:0] peak_y;
 reg [31:0] ma,mb,pp00,pp01,pp10,pp11;
 reg [63:0] sum0,sum1,sum2;reg neg_mul;reg signed [63:0] product;
 reg [63:0] dividend,divisor,quotient;reg [64:0] remainder;
 reg neg_div;reg [6:0] bit_index;reg signed [63:0] result;
 reg [31:0] sqrt_input;reg [15:0] root;reg [4:0] root_bit;
 wire [15:0] trial=root|(16'd1<<root_bit);
 wire signed [31:0] curvature=2*$signed({1'b0,peak_c})-$signed({1'b0,peak_l})-$signed({1'b0,peak_r});
 wire signed [31:0] peak_difference=$signed({1'b0,peak_r})-$signed({1'b0,peak_l});
 task mul;input signed [31:0] a,b;input [7:0] next;
 begin ma<=a[31]?-a:a;mb<=b[31]?-b:b;neg_mul<=a[31]^b[31];resume_mul<=next;state<=200;end endtask
 task divide;input signed [63:0] a,b;input [7:0] next;
 begin
  if(b==0)begin valid<=0;state<=99;end
  else begin dividend<=a[63]?-a:a;divisor<=b[63]?-b:b;neg_div<=a[63]^b[63];quotient<=0;remainder<=0;bit_index<=63;resume_div<=next;state<=210;end
 end endtask
 task square_root;input [31:0] a;input [7:0] next;
 begin sqrt_input<=a;root<=0;root_bit<=15;resume_sqrt<=next;state<=220;end endtask
 task clear_points;
 begin count<=0;sx<=0;sy<=0;syy<=0;sxy<=0;point_valid<=0;end endtask
 always @(posedge clk)begin
  if(!rst_n||cancel)begin
   state<=0;valid<=0;curve_mask<=0;rms_q8<=0;max_q8<=0;used_rows<=0;worst_x<=0;worst_y<=0;peak_x<=0;peak_y<=0;clear_points();
  end else if(frame_start)begin
   valid<=0;
   if(frame_ok&&count>=min_rows&&state==0)begin state<=10;end
   else begin clear_points();state<=0;end
  end else case(state)
   0:if(point_we)begin
    pending_y<=point_y;pending_x<={1'b0,point_x,8'b0};
    if(curvature>0)divide($signed(peak_difference)*64'sd128,$signed(curvature),1);
    else begin result<=0;state<=1;end
   end
   1:begin
    if(result>128)pending_x<=pending_x+128;
    else if(result< -128)pending_x<=pending_x-128;
    else pending_x<=pending_x+result;
    state<=2;
   end
   2:begin
    points[pending_y]<=pending_x;point_valid[pending_y]<=1;
    sx<=sx+pending_x;sy<=sy+pending_y;count<=count+1'b1;
    mul({24'd0,pending_y},{24'd0,pending_y},3);
   end
   3:begin syy<=syy+product;mul({16'd0,pending_x},{24'd0,pending_y},4);end
   4:begin sxy<=sxy+product;state<=0;end
   // a_Q16 = (N*Sxy_Q8-Sy*Sx_Q8)*256/(N*Syy-Sy^2)
   10:mul({23'd0,count},sxy,11);
   11:begin t0<=product;mul(sy,sx,12);end
   12:begin numerator<=(t0-product)<<<8;mul({23'd0,count},syy,13);end
   13:begin t1<=product;mul(sy,sy,14);end
   14:begin denominator<=t1-product;state<=15;end
   15:if(denominator<=0)begin valid<=0;state<=99;end else divide(numerator,denominator,16);
   16:begin slope<=result;state<=17;end
   17:mul(slope,sy,18);
   18:begin intercept_num<=($signed({1'b0,sx})<<<8)-product;state<=19;end
   19:divide(intercept_num,{55'd0,count},20);
   20:begin intercept<=result;mul(slope,slope,21);end
   21:begin norm_q16<=65536+(product>>16);idx<=0;sum_squares<=0;largest<=0;peak_x<=0;peak_y<=0;state<=30;end
   30:mul(slope,{23'd0,idx},31);
   31:begin predicted<=product+$signed(intercept);state<=32;end
   32:begin
    curve_mask[idx[7:0]]<=point_valid[idx[7:0]];
    curve[idx[7:0]]<=signed_residual>32767?16'sd32767:signed_residual< -32768?-16'sd32768:signed_residual[15:0];
    if(predicted<0||predicted>127*65536)fitted[idx[7:0]]<=0;
    else fitted[idx[7:0]]<=8'h80|((predicted+32768)>>16);
    // rounded Q8 perpendicular correction applied to aggregate below
    if(point_valid[idx[7:0]])begin
     if($signed({1'b0,points[idx[7:0]],8'b0})>=predicted)
      residual<=($signed({1'b0,points[idx[7:0]],8'b0})-predicted+128)>>8;
     else residual<=(predicted-$signed({1'b0,points[idx[7:0]],8'b0})+128)>>8;
     state<=33;
    end else state<=35;
   end
   33:begin if(residual>largest||largest==0)begin largest<=residual;peak_x<=(points[idx[7:0]]+16'd128)>>8;peak_y<=idx[7:0];end mul(residual,residual,34);end
   34:begin sum_squares<=sum_squares+product;state<=35;end
   35:if(idx==255)state<=40;else begin idx<=idx+1'b1;state<=30;end
   40:mul({23'd0,count},norm_q16,41);
   41:begin denominator<=product;state<=42;end
   42:divide($signed(sum_squares<<16),denominator,43);
   43:square_root(result[31:0],44);
   44:begin pending_rms<=result;mul(largest,largest,45);end
   45:begin numerator<=product<<16;state<=46;end
   46:divide(numerator,{32'd0,norm_q16},47);
   47:square_root(result[31:0],48);
   48:begin rms_q8<=pending_rms;max_q8<=result;used_rows<=count;worst_x<=peak_x;worst_y<=peak_y;valid<=1;clear_points();state<=0;end
   99:begin clear_points();state<=0;end
   200:begin pp00<=ma[15:0]*mb[15:0];pp01<=ma[15:0]*mb[31:16];pp10<=ma[31:16]*mb[15:0];pp11<=ma[31:16]*mb[31:16];state<=201;end
   201:begin sum0<={32'd0,pp00}+{16'd0,pp01,16'd0};sum1<={16'd0,pp10,16'd0}+{pp11,32'd0};state<=202;end
   202:begin sum2<=sum0+sum1;state<=203;end
   203:begin product<=neg_mul?-$signed(sum2):$signed(sum2);state<=resume_mul;end
   210:begin
    if({remainder[63:0],dividend[bit_index]}>={1'b0,divisor})begin remainder<={remainder[63:0],dividend[bit_index]}-{1'b0,divisor};quotient[bit_index]<=1;end
    else remainder<={remainder[63:0],dividend[bit_index]};
    if(bit_index==0)state<=211;else bit_index<=bit_index-1'b1;
   end
   211:begin result<=neg_div?-$signed(quotient):$signed(quotient);state<=resume_div;end
   220:mul({16'd0,trial},{16'd0,trial},221);
   221:begin
    if(product<={32'd0,sqrt_input})root<=trial;
    if(root_bit==0)state<=222;else begin root_bit<=root_bit-1'b1;state<=220;end
   end
   222:begin result<={48'd0,root};state<=resume_sqrt;end
   default:state<=0;
  endcase
 end
endmodule

