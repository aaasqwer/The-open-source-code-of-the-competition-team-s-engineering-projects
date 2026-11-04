`timescale 1ns/1ps
// V4 independent ping-pong capture and analysis. Process a complete ROI as
// soon as its final pixel arrives; vertical sync never aborts a valid job.
// Horizontal edges are measured by transposing coordinates, not rotating video.
module roi_evaluator(input clk,rst_n,vs,de,input [15:0] pixel,
 input enabled,input [2:0] position,input [1:0] mode,
 output [3:0] status,output [15:0] overlay_pixel,
 output measurement_valid,output [15:0] rms_q8,max_q8,output [8:0] used_rows,
 output [7:0] worst_row,coverage_percent,output endpoint_peak,
 input horizontal,editing,input [10:0] center_x,center_y,output graph_visible,output sfr_graph_visible,output sfr_valid,sfr_above,output [3:0] sfr_status,output [15:0] mtf50_q16,sfr_quarter_q12,output reg sfr_info_valid,output reg [15:0] sfr_frame_rms,sfr_angle_q8,output [2:0] sfr_samples);
 reg [10:0] px=0,py=0;reg lv=0,ld=0;
 reg ph=0,pe=0,pen=0;reg [10:0] pcx=0,pcy=0;reg[1:0] pm=0;
 wire changed=ph!=horizontal||pe!=editing||pen!=enabled||pcx!=center_x||pcy!=center_y||pm!=mode;
 wire [10:0] width=horizontal?256:128,height=horizontal?128:256;
 wire [10:0] left=center_x-(width>>1),top=center_y-(height>>1);
 wire inside_roi=px>=left&&px<left+width&&py>=top&&py<top+height;
 wire [10:0] lx=px-left,ly=py-top;
 wire active=enabled&&mode!=2;
 // Suppress the residual panel when it would cover the selected target or while
 // moving the ROI. The original pixels remain visible in the selected area.
 assign graph_visible=active&&!editing&&!(left<1192&&left+width>680&&top<1060&&top+height>788);
 wire [7:0] r={pixel[15:11],pixel[15:13]},g={pixel[10:5],pixel[10:9]},b={pixel[4:0],pixel[4:2]};
 wire [9:0] total={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b};
 reg [7:0] memory[0:65535];reg [15:0] captured=0;
 reg capture_bank=0,compute_bank=1,filled_bank=0,filled_valid=0,capture_active=0;
 wire [14:0] wa=horizontal?{lx[7:0],ly[6:0]}:{ly[7:0],lx[6:0]};
 reg [2:0] state=0;reg core_vs=0;reg [15:0] wait_count=0;reg [6:0] sx=0;reg [7:0] sy=0,gap=0;
 wire run=state==2;
 reg [7:0] sample;reg sample_de=0;reg [10:0] sample_x=0,sample_y=0;
 wire cancel=changed||!active||editing;
 wire [10:0] dx=inside_roi?11'd96+(horizontal?ly:lx):11'd2047;
 wire [10:0] dy=inside_roi?11'd336+(horizontal?lx:ly):11'd2047;
 wire [15:0] core_pixel;wire core_valid;
 wire signed[31:0] fit_slope,fit_intercept;
 reg sfr_start=0;wire sf_busy,sf_done,sf_valid,sf_curve;wire[3:0]sf_status;
 wire[14:0]sf_addr;wire[15:0]sf_mtf,sf_quarter,sf_curve_value;
 // Keep the last completed spectrum while the next ROI is being analysed.
 // A changed target/configuration or a completed invalid result still clears it.
 wire sf_cancel=cancel||(state==5&&wait_count==65535&&!core_valid);
 wire[5:0]sf_query=(px-32)>>4;
 sfr_meter sharpness(.clk(clk),.rst_n(rst_n),.cancel(sf_cancel),.start(sfr_start),.fit_valid(core_valid),
 .slope_q16(fit_slope),.intercept_q16(fit_intercept),.rms_q8(rms_q8),.rows(used_rows),
 .rd_addr(sf_addr),.rd_data(sample),.busy(sf_busy),.done(sf_done),.valid(sf_valid),.curve_valid(sf_curve),
 .status(sf_status),.mtf50_q16(sf_mtf),.sfr_quarter_q12(sf_quarter),.curve_index(sf_query),.curve_q12(sf_curve_value));
 wire [2:0] sf_samples;
 wire sf_window_valid;wire[15:0]sf_low,sf_high;wire[8:0]sf_rows;
 measurement_window sharpness_window(.clk(clk),.rst_n(rst_n),.clear(sf_cancel||(sf_done&&!sf_valid)),
 .sample_valid(sf_done&&sf_valid),.rms_in(sf_mtf),.max_in(sf_quarter),.rows_in(used_rows),
 .valid(sf_window_valid),.rms_median(),.max_median(),.range_low(sf_low),.range_high(sf_high),.rows_min(sf_rows),.samples(sf_samples));
 assign sfr_samples=sf_cancel?3'd0:(sf_window_valid?3'd7:sf_samples);
 wire [31:0] abs_fit_slope=fit_slope[31]?-fit_slope:fit_slope;
 wire [47:0] angle_product=abs_fit_slope*48'd14668;
 reg pending_info;reg [15:0] pending_rms,pending_angle;
 always @(posedge clk)begin
  if(!rst_n||sf_cancel)begin sfr_info_valid<=0;sfr_frame_rms<=0;sfr_angle_q8<=0;pending_info<=0;pending_rms<=0;pending_angle<=0;end
  else begin
   if(sfr_start)begin pending_info<=core_valid;pending_rms<=rms_q8;pending_angle<=angle_product[47:16]>65535?16'hffff:angle_product[31:16];end
   if(sf_done)begin sfr_info_valid<=pending_info;sfr_frame_rms<=pending_rms;sfr_angle_q8<=pending_angle;end
  end
 end
 assign sfr_valid=sf_valid&&!sf_cancel;
 assign mtf50_q16=sf_mtf;assign sfr_quarter_q12=sf_quarter;
 assign sfr_above=sf_status==9&&!sf_cancel;
 assign sfr_status=sf_cancel?4'd1:sf_status;
 assign sfr_graph_visible=active&&!editing&&sf_curve&&!(left<545&&left+width>32&&top<1060&&top+height>788);
 wire sf_plot=sfr_graph_visible&&px>=32&&px<=544&&py>=820&&py<1060;
 wire [23:0]sf_scaled=sf_curve_value*24'd192;
 wire [10:0]sf_offset=sf_scaled[23:12]>224?11'd224:sf_scaled[22:12];
 wire sf_dot=py+1>=1044-sf_offset&&py<=1045-sf_offset;
 wire [15:0]sf_pixel=sf_dot?16'h07e0:((py==1044||py==948||py==852)?16'h7bef:16'h0841);

 roi_edge_core core(.clk(clk),.rst_n(rst_n),.vs(core_vs),.de(sample_de),.pixel(pixel),
 .enabled(enabled&&!editing),.position(3'd1),.mode(mode),.x(sample_x),.y(sample_y),
 .display_x(dx),.display_y(dy),.graph_x(graph_visible?px:11'd2047),.graph_y(graph_visible?py:11'd2047),
 .luminance(sample),.external_cancel(cancel),.status(status),.overlay_pixel(core_pixel),
 .fit_slope_q16(fit_slope),.fit_intercept_q16(fit_intercept),.measurement_valid(core_valid),.rms_q8(rms_q8),.max_q8(max_q8),.used_rows(used_rows),
 .worst_row(worst_row),.coverage_percent(coverage_percent),.endpoint_peak(endpoint_peak));
 assign measurement_valid=core_valid&&!cancel;
 wire move_border=active&&editing&&inside_roi&&(lx<3||lx>=width-3||ly<3||ly>=height-3);
 assign overlay_pixel=de?(move_border?16'hffe0:(sf_plot?sf_pixel:core_pixel)):pixel;
 always @(posedge clk)begin
  // Synchronous memory read, with matching coordinate and DE pipeline.
  sample<=memory[{compute_bank,(state==6?sf_addr:{sy,sx})}];sample_x<=96+sx;sample_y<=336+sy;sample_de<=run;
  sfr_start<=0;
  if(!rst_n)begin px<=0;py<=0;lv<=0;ld<=0;state<=0;captured<=0;sample_de<=0;core_vs<=0;
   capture_bank<=0;compute_bank<=1;filled_bank<=0;filled_valid<=0;capture_active<=0;
   ph<=0;pe<=0;pen<=0;pcx<=0;pcy<=0;pm<=0;sx<=0;sy<=0;gap<=0;wait_count<=0;
  end else begin
   lv<=vs;ld<=de;ph<=horizontal;pe<=editing;pen<=enabled;pcx<=center_x;pcy<=center_y;pm<=mode;
   if(vs&&!lv)begin px<=0;py<=0;end else begin if(de)px<=px+1;else px<=0;if(ld&&!de)py<=py+1;end
   if(cancel)begin captured<=0;capture_active<=0;filled_valid<=0;state<=0;sample_de<=0;core_vs<=0;end
   else begin
    // The producer and consumer own distinct banks. At worst a frame is
    // skipped when both are owned, never replayed half-overwritten.
    if(vs&&!lv)begin
     captured<=0;capture_active<=0;
     if((state==0||compute_bank!=0)&&(!filled_valid||filled_bank!=0))begin capture_bank<=0;capture_active<=1;end
     else if((state==0||compute_bank!=1)&&(!filled_valid||filled_bank!=1))begin capture_bank<=1;capture_active<=1;end
    end
    if(de&&inside_roi&&capture_active)begin
     memory[{capture_bank,wa}]<=total[9:2];captured<=captured+1'b1;
     if(captured==32767)begin filled_bank<=capture_bank;filled_valid<=1;capture_active<=0;end
    end
    if(state==0&&filled_valid)begin
     compute_bank<=filled_bank;filled_valid<=0;sx<=0;sy<=0;gap<=0;wait_count<=0;core_vs<=0;state<=2;
    end
    case(state)

     2:if(sx==127)begin state<=3;gap<=0;end else sx<=sx+1;
     3:if(gap==127)begin sx<=0;if(sy==255)state<=4;else begin sy<=sy+1;state<=2;end end else gap<=gap+1;
     4:begin core_vs<=1;wait_count<=0;state<=5;end
     5:begin
      core_vs<=0;
      // Wait for the actual fit result, not an assumed 5999-clock budget.
      if(wait_count>=8&&core_valid)begin sfr_start<=1;state<=6;end
      else if(wait_count==65535)state<=0;
      else wait_count<=wait_count+1'b1;
     end
     6:if(sf_done)state<=0;
     default:begin end
    endcase
   end
  end
 end
endmodule
