`timescale 1ns/1ps
// New geometry front end. Ping-pong 32x16 coordinate tiles let DDR work on N
// while this pipeline prepares N+1. Every actual output point contributes to
// bounds; no nonlinear four-corner bounding approximation is used.
module rect_coordinate_tiles #(parameter W=1920,H=1080)(
 input clk,rst_n,clear,start,write_bank,input [10:0] tile_x,tile_y,
 input [5:0] tile_w,input [4:0] tile_h,input [1:0] mode,
 input [287:0] inverse_matrix,input signed [17:0] c,s,
 input field_ready,field_fault,output field_valid,output signed [25:0] field_x,field_y,
 input field_out_valid,field_out_bad,field_inside,input signed [25:0] field_source_x,field_source_y,
 input read_bank,input [8:0] read_address,output reg [52:0] read_coordinate,
 output reg [1:0] ready,any_inside,bad,output [87:0] bounds,output busy
);
 localparam [287:0] IDENTITY={32'h40000000,32'd0,32'd0,32'd0,32'h00100000,32'd0,32'd0,32'd0,32'h00100000};
 localparam [287:0] HALF_CENTER={32'h40000000,32'd0,32'd0,32'hfff80000,32'h00100000,32'd0,32'hfff80000,32'd0,32'h00100000};
 reg [52:0] coordinates[0:1023];reg [1:0] phase;reg target;
 reg [10:0] tx,ty;reg [5:0] tw,px;reg [4:0] th,py;reg [1:0] selected_mode;
 reg [287:0] transform;reg signed [17:0] cc,ss;
 reg [9:0] received;reg error,found;
 reg [10:0] min_x,max_x,min_y,max_y;
 reg [43:0] bank_bounds[0:1];assign bounds={bank_bounds[1],bank_bounds[0]};
 assign busy=phase!=0;
 wire [10:0] screen_x=tx+px,screen_y=ty+py;
 wire split=selected_mode==3;
 wire inactive=split&&(screen_y<(H/4)||screen_y>=(H/4+H/2));
 wire raw=selected_mode==0||(split&&screen_x<W/2)||inactive;
 wire [10:0] input_x=inactive?0:split?((screen_x>=W/2?screen_x-W/2:screen_x)<<1)+1:screen_x;
 wire [10:0] input_y=inactive?0:split?((screen_y-H/4)<<1)+1:screen_y;
 wire [10:0] input_tag={inactive,raw,py[3:0],px[4:0]};
 // Separate split-view add/subtract/mux logic from the DSP input registers.
 reg [10:0] mapped_x,mapped_y,mapped_tag;reg [287:0] mapped_matrix;reg mapped_valid;
 always @(posedge clk)begin
  mapped_x<=input_x;mapped_y<=input_y;mapped_tag<=input_tag;
  mapped_matrix<=raw?(split?HALF_CENTER:IDENTITY):transform;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)mapped_valid<=0;else if(clear)mapped_valid<=0;else mapped_valid<=phase==1;
 end
 wire pv,pbad;wire signed [25:0] pu,pvcoord;wire [10:0] ptag;
 rect_projective_pipe transform_pipe(.clk(clk),.rst_n(rst_n&&!clear),.in_valid(mapped_valid),.x(mapped_x),.y(mapped_y),.in_tag(mapped_tag),
  .inverse_matrix(mapped_matrix),.out_valid(pv),.out_bad(pbad),.ideal_x(pu),.ideal_y(pvcoord),.out_tag(ptag));
 wire av,abad;wire signed [25:0] au,avcoord;wire [10:0] atag;
 rect_imu_affine_pipe #(.W(W),.H(H)) leveling(.clk(clk),.rst_n(rst_n&&!clear),.in_valid(pv),.in_bad(pbad),.x(pu),.y(pvcoord),.in_tag(ptag),
  .c(cc),.s(ss),.out_valid(av),.out_bad(abad),.ideal_x(au),.ideal_y(avcoord),.out_tag(atag));
 assign field_valid=av&&!atag[9];assign field_x=au;assign field_y=avcoord;
 // Lens field now has one additional register after vertical interpolation.
 localparam FIELD_LATENCY=7;
 reg [FIELD_LATENCY-1:0] bypass_valid,bypass_bad;
 reg [10:0] tag_pipe[0:FIELD_LATENCY-1];reg signed [25:0] u_pipe[0:FIELD_LATENCY-1],v_pipe[0:FIELD_LATENCY-1];
 integer j;
 always @(posedge clk)begin
  tag_pipe[0]<=atag;u_pipe[0]<=au;v_pipe[0]<=avcoord;
  for(j=1;j<FIELD_LATENCY;j=j+1)begin tag_pipe[j]<=tag_pipe[j-1];u_pipe[j]<=u_pipe[j-1];v_pipe[j]<=v_pipe[j-1];end
  read_coordinate<=coordinates[{read_bank,read_address}];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin bypass_valid<=0;bypass_bad<=0;end
  else if(clear)begin bypass_valid<=0;bypass_bad<=0;end
  else begin bypass_valid<={bypass_valid[FIELD_LATENCY-2:0],av&&atag[9]};bypass_bad<={bypass_bad[FIELD_LATENCY-2:0],abad};end
 end
 wire returned=bypass_valid[FIELD_LATENCY-1]||field_out_valid;
 wire [10:0] tag=tag_pipe[FIELD_LATENCY-1];
 wire signed [25:0] sx=tag[9]?u_pipe[FIELD_LATENCY-1]:field_source_x,sy=tag[9]?v_pipe[FIELD_LATENCY-1]:field_source_y;
 wire numerical_bad=tag[9]?bypass_bad[FIELD_LATENCY-1]:field_out_bad||bypass_bad[FIELD_LATENCY-1]||field_fault;
 wire mapped_inside=sx>=0&&sx<=(W-1)*4096&&sy>=0&&sy<=(H-1)*4096;
 wire good=mapped_inside&&!numerical_bad&&!tag[10];
 wire [10:0] ix=sx[22:12],iy=sy[22:12];
 always @(posedge clk)if(returned&&phase!=0&&!clear)coordinates[{target,tag[8:0]}]<={good,sx,sy};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;target<=0;tx<=0;ty<=0;tw<=0;th<=0;px<=0;py<=0;selected_mode<=0;transform<=IDENTITY;cc<=16384;ss<=0;
   received<=0;error<=0;found<=0;min_x<=W-1;min_y<=H-1;max_x<=0;max_y<=0;ready<=0;any_inside<=0;bad<=0;bank_bounds[0]<=0;bank_bounds[1]<=0;end
  else if(clear)begin phase<=0;ready<=0;any_inside<=0;bad<=0;end
  else begin
   if(start&&phase==0)begin
    phase<=1;target<=write_bank;tx<=tile_x;ty<=tile_y;tw<=tile_w;th<=tile_h;px<=0;py<=0;selected_mode<=mode;transform<=inverse_matrix;cc<=c;ss<=s;
    if(mode==3)begin
     transform[64+:32]<=$signed(inverse_matrix[64+:32])-($signed(inverse_matrix[0+:32])>>>1)-($signed(inverse_matrix[32+:32])>>>1);
     transform[160+:32]<=$signed(inverse_matrix[160+:32])-($signed(inverse_matrix[96+:32])>>>1)-($signed(inverse_matrix[128+:32])>>>1);
     transform[256+:32]<=$signed(inverse_matrix[256+:32])-($signed(inverse_matrix[192+:32])>>>1)-($signed(inverse_matrix[224+:32])>>>1);
    end
    received<=0;error<=0;found<=0;min_x<=W-1;min_y<=H-1;max_x<=0;max_y<=0;ready[write_bank]<=0;bad[write_bank]<=0;
   end
   if(phase==1)begin
    if(px==tw-1)begin px<=0;if(py==th-1)phase<=2;else py<=py+1'b1;end else px<=px+1'b1;
   end
   if(returned&&phase!=0)begin
    received<=received+1'b1;
    if((numerical_bad||!mapped_inside)&&!tag[10])error<=1;
    if(good)begin found<=1;if(ix<min_x)min_x<=ix;if(ix>max_x)max_x<=ix;if(iy<min_y)min_y<=iy;if(iy>max_y)max_y<=iy;end
    if(received==tw*th-1)phase<=3;
   end
   if(phase==3)begin
    bank_bounds[target]<={max_y,min_y,max_x,min_x};any_inside[target]<=found;bad[target]<=error;ready[target]<=1;phase<=0;
   end
  end
 end
endmodule
