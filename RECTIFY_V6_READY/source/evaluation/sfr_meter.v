`timescale 1ns/1ps
// Output-code luminance e-SFR: 4x normal-distance bins, central derivative,
// Hann LSF window, 256-point zero-padded DFT, derivative correction.
// No assumed gamma/OECF. Frequency is cycles per measured output pixel.
// ROI RAM is shared with roi_evaluator; rd_data has ONE synchronous read cycle.
module sfr_meter(input clk,rst_n,cancel,start,input fit_valid,
 input signed [31:0] slope_q16,intercept_q16,input [15:0] rms_q8,input [8:0] rows,
 output reg [14:0] rd_addr,input [7:0] rd_data,
 output reg busy,done,valid,curve_valid,output reg [3:0] status,
 output reg [15:0] mtf50_q16,output [15:0] sfr_quarter_q12,
 input [5:0] curve_index,output [15:0] curve_q12);
 reg [7:0] state,resume_div,resume_sqrt;
 reg signed [31:0] a,b,predicted;reg [16:0] invnorm;
 reg [16:0] normal_rom[0:127];reg [15:0] window_rom[0:127];
 reg signed [15:0] trig[0:255];reg [15:0] correction[0:32];
 initial begin
  $readmemh("source/evaluation/sfr_normal.hex",normal_rom);
  $readmemh("source/evaluation/sfr_window.hex",window_rom);
  $readmemh("source/evaluation/sfr_cos.hex",trig);
  $readmemh("source/evaluation/sfr_correction.hex",correction);
 end
 reg [23:0] sums[0:127];reg [11:0] counts[0:127];reg [15:0] esf[0:127];
 reg signed [31:0] lsf[0:127];reg [15:0] spectrum[0:32],published[0:32];
 assign curve_q12=curve_valid&&curve_index<=32?published[curve_index]:16'd0;
 assign sfr_quarter_q12=published[16];
 reg [7:0] ix,iy,bin;reg [8:0] j;reg [5:0] k;
 reg signed [31:0] distance_q16;reg signed [63:0] normal_product;
 wire signed [63:0] bin_number=(normal_product>>>30)+64;
 reg [15:0] used,clipped;reg [31:0] low_sum,high_sum;
 // ESF difference fits signed 17 bits; explicit 34-bit product avoids wide signed multiplier mapping discrepancy.
 reg signed [16:0] derivative;reg signed [33:0] window_product;
 reg [7:0] phase;reg signed [47:0] acc_r,acc_i,pr,pi;
 reg signed [31:0] vr,vi;reg [63:0] square_r,square_i;
 reg [31:0] magnitude,dc;reg [63:0] corrected;
 reg found;reg [15:0] crossing,previous;
 reg [63:0] dividend,divisor,quotient;reg [64:0] remainder;reg [6:0] bitno;
 reg [63:0] result,sqrt_value,trial_square;reg [31:0] root;reg [5:0] root_bit;
 wire [31:0] trial=root|(32'd1<<root_bit);
 wire [31:0] abs_slope=slope_q16[31]?-slope_q16:slope_q16;
 integer z;
 task fail;input[3:0] code;begin status<=code;valid<=0;curve_valid<=0;busy<=0;done<=1;state<=0;end endtask
 task divide;input[63:0] n,d;input[7:0] next;
 begin if(d==0)fail(10);else begin dividend<=n;divisor<=d;quotient<=0;remainder<=0;bitno<=63;resume_div<=next;state<=200;end end endtask
 task sqrt;input[63:0] n;input[7:0] next;
 begin sqrt_value<=n;root<=0;root_bit<=31;resume_sqrt<=next;state<=210;end endtask
 always @(posedge clk)begin
  done<=0;
  if(!rst_n||cancel)begin state<=0;busy<=0;done<=0;valid<=0;curve_valid<=0;status<=1;mtf50_q16<=0;rd_addr<=0;end
  else case(state)
   0:if(start)begin
    busy<=1;status<=1;
    if(!fit_valid||rows<244)fail(10);
    else if(abs_slope<2289||abs_slope>9211)fail(3); // 2 to 8 degrees
    else if(rms_q8>51)fail(4); // curved/jittered edge is not a sharpness target
    else begin a<=slope_q16;b<=intercept_q16;predicted<=intercept_q16;
     invnorm<=normal_rom[abs_slope[13:7]];j<=0;used<=0;clipped<=0;state<=1;end
   end
   1:begin sums[j[6:0]]<=0;counts[j[6:0]]<=0;
    if(j==127)begin ix<=0;iy<=0;state<=2;end else j<=j+1;end
   2:begin rd_addr<={iy,ix[6:0]};distance_q16<=$signed({8'd0,ix,16'd0})-predicted;state<=3;end
   3:begin normal_product<=$signed(distance_q16)*$signed({15'd0,invnorm});state<=4;end
   4:begin
    if(bin_number>=0&&bin_number<128)begin
     sums[bin_number[6:0]]<=sums[bin_number[6:0]]+rd_data;
     counts[bin_number[6:0]]<=counts[bin_number[6:0]]+1;
     used<=used+1;if(rd_data<=4||rd_data>=251)clipped<=clipped+1;
    end
    if(ix==127)begin ix<=0;predicted<=predicted+a;
     if(iy==255)state<=5;else begin iy<=iy+1;state<=2;end
    end else begin ix<=ix+1;state<=2;end
   end
   5:if(used<7000)fail(11);else if(clipped*100>used)fail(5);else begin j<=0;low_sum<=0;high_sum<=0;state<=6;end
   6:if(counts[j[6:0]]<16)fail(6);else divide({32'd0,sums[j[6:0]],8'd0},{52'd0,counts[j[6:0]]},7);
   7:begin esf[j[6:0]]<=result[15:0];
    if(j<8)low_sum<=low_sum+result[15:0];if(j>=120)high_sum<=high_sum+result[15:0];
    if(j==127)state<=8;else begin j<=j+1;state<=6;end
   end
   8:if((high_sum>=low_sum?high_sum-low_sum:low_sum-high_sum)<98304)fail(7);
     else begin j<=0;state<=9;end // plateau difference >=48 codes
   9:begin derivative<=j==0||j==127?0:$signed({1'b0,esf[j+1]})-$signed({1'b0,esf[j-1]});state<=10;end
   10:begin window_product<=$signed(derivative)*$signed({1'b0,window_rom[j[6:0]]});state<=11;end
   11:begin lsf[j[6:0]]<={{13{window_product[33]}},window_product[33:15]};
    if(j==127)begin j<=0;k<=0;acc_r<=0;acc_i<=0;found<=0;state<=12;end
    else begin j<=j+1;state<=9;end
   end
   12:begin phase<=k*j;state<=13;end
   13:begin pr<=$signed(lsf[j[6:0]])*$signed(trig[phase]);pi<=$signed(lsf[j[6:0]])*$signed(trig[(phase+8'd192)&8'hff]);state<=14;end
   14:begin acc_r<=acc_r+pr;acc_i<=acc_i+pi;
    if(j==127)state<=15;else begin j<=j+1;state<=12;end
   end
   15:begin vr<=acc_r>>>14;vi<=acc_i>>>14;state<=16;end
   16:begin square_r<=$signed(vr)*$signed(vr);square_i<=$signed(vi)*$signed(vi);state<=17;end
   17:sqrt(square_r+square_i,18);
   18:begin magnitude<=root;state<=19;end
   19:if(k==0)begin
    if(magnitude<24576)fail(7);else begin dc<=magnitude;spectrum[0]<=4096;previous<=4096;k<=1;j<=0;acc_r<=0;acc_i<=0;state<=12;end
   end else begin corrected<=magnitude*correction[k];state<=20;end
   20:divide(corrected<<12,{18'd0,dc,14'd0},21);
   21:begin spectrum[k]<=result>65535?16'hffff:result[15:0];
    previous<=result>65535?16'hffff:result[15:0];
    if(!found&&result<=2048&&previous>2048)begin
     found<=1;divide(({48'd0,previous}-2048)<<10,{48'd0,previous}-result,22);
    end else state<=23;
   end
   22:begin crossing<=((k-1)<<10)+result;state<=23;end
   23:begin if(k==32)state<=24;
    else begin k<=k+1;j<=0;acc_r<=0;acc_i<=0;state<=12;end
   end
   24:begin
    for(z=0;z<33;z=z+1)published[z]<=spectrum[z];
    curve_valid<=1;valid<=found;status<=found?2:9;mtf50_q16<=found?crossing:0;busy<=0;done<=1;state<=0;
   end
   200:begin
    if({remainder[63:0],dividend[bitno]}>={1'b0,divisor})begin remainder<={remainder[63:0],dividend[bitno]}-{1'b0,divisor};quotient[bitno]<=1;end
    else remainder<={remainder[63:0],dividend[bitno]};
    if(bitno==0)state<=201;else bitno<=bitno-1;
   end
   201:begin result<=quotient;state<=resume_div;end
   210:begin trial_square<=trial*trial;state<=211;end
   211:begin if(trial_square<=sqrt_value)root<=trial;
    if(root_bit==0)state<=212;else begin root_bit<=root_bit-1;state<=210;end
   end
   212:state<=resume_sqrt;
   default:fail(10);
  endcase
 end
endmodule
