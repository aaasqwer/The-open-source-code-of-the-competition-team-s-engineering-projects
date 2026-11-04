`timescale 1ns/1ps
// Five independent debouncers. No key combinations required. Geometry state
// is transferred as one mailbox and latched by the renderer at a frame start.
module rect_board_ui #(parameter CLK_HZ=27000000,DEBOUNCE=CLK_HZ/50,LONG_PRESS=CLK_HZ*7/10)(
 input clk,rst_n,input [4:0] buttons_n,input remote_update,input [7:0] remote_flags,
 output [1:0] mode,output [7:0] ui_flags,output reg imu_enabled,output plane_held,imu_page,
 output reg caption_visible,numbers_visible,evaluation_enabled,editing,horizontal,
 output reg [2:0] position,output reg [10:0] center_x,center_y,
 output reg [31:0] sequence_id
);
 (* async_reg="true" *)reg [4:0] meta,sync;
 reg [4:0] stable,candidate,press_event,release_event,long_event,long_seen;
 reg [31:0] debounce_count[0:4],hold_count[0:4];
 reg corrected,plane_active;reg [1:0] profile;
 assign mode=corrected?(profile==0?2'd1:profile==1?2'd2:2'd3):2'd0;
 assign ui_flags={1'b0,caption_visible,numbers_visible,evaluation_enabled,imu_enabled,profile,corrected};
 assign plane_held=!stable[0]&&long_seen[0]&&!editing;assign imu_page=!stable[4]&&!editing;
 task preset;
  input [2:0] p;
  begin case(p)
   1:begin center_x<=192;center_y<=192;end 2:begin center_x<=1728;center_y<=192;end
   3:begin center_x<=192;center_y<=888;end 4:begin center_x<=1728;center_y<=888;end
   default:begin center_x<=960;center_y<=540;end
  endcase end
 endtask
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin meta<=31;sync<=31;stable<=31;candidate<=31;press_event<=0;release_event<=0;long_event<=0;long_seen<=0;
   for(k=0;k<5;k=k+1)begin debounce_count[k]<=0;hold_count[k]<=0;end
  end else begin
   meta<=buttons_n;sync<=meta;press_event<=0;release_event<=0;long_event<=0;
   for(k=0;k<5;k=k+1)begin
    if(sync[k]!=candidate[k])begin candidate[k]<=sync[k];debounce_count[k]<=0;end
    else if(sync[k]==stable[k])debounce_count[k]<=0;
    else if(debounce_count[k]==DEBOUNCE-1)begin
     stable[k]<=sync[k];debounce_count[k]<=0;
     if(!sync[k])begin press_event[k]<=1;hold_count[k]<=0;long_seen[k]<=0;end
     else release_event[k]<=1;
    end else debounce_count[k]<=debounce_count[k]+1'b1;
    if(!stable[k]&&!long_seen[k])begin
     if(hold_count[k]==LONG_PRESS-1)begin long_seen[k]<=1;long_event[k]<=1;end else hold_count[k]<=hold_count[k]+1'b1;
    end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin corrected<=0;plane_active<=0;profile<=0;imu_enabled<=1;caption_visible<=1;numbers_visible<=1;evaluation_enabled<=0;editing<=0;horizontal<=0;
   position<=0;center_x<=960;center_y<=540;sequence_id<=0;end
  else begin
   if(|press_event|| |release_event|| |long_event)sequence_id<=sequence_id+1'b1;
   // KEY4 is the reference IMU data page; KEY0 long-hold owns plane rectification.
   if(remote_update)begin
    corrected<=remote_flags[0];profile<=remote_flags[2:1]<3?remote_flags[2:1]:0;imu_enabled<=remote_flags[3];
    evaluation_enabled<=remote_flags[4];numbers_visible<=remote_flags[5];caption_visible<=remote_flags[6];sequence_id<=sequence_id+1'b1;
   end else begin
    if(long_event[1])caption_visible<=!caption_visible;
    if(long_event[2])numbers_visible<=!numbers_visible;
    if(long_event[3])begin editing<=!editing;evaluation_enabled<=1;end
    if(release_event[0]&&!long_seen[0])begin
     if(editing)begin center_x<=center_x>=160?center_x-32:128;position<=5;end else if(profile==2)begin profile<=0;corrected<=1;end else corrected<=!corrected;
    end
    if(release_event[1]&&!long_seen[1])begin
     if(editing)begin center_x<=center_x<=1760?center_x+32:1792;position<=5;end else begin profile<=2;corrected<=1;end
    end
    if(release_event[2]&&!long_seen[2])begin
     if(editing)begin center_y<=center_y>=160?center_y-32:128;position<=5;end else imu_enabled<=!imu_enabled;
    end
    if(release_event[3]&&!long_seen[3])begin
     if(editing)begin center_y<=center_y<=920?center_y+32:952;position<=5;end
     else if(!evaluation_enabled)begin evaluation_enabled<=1;position<=0;preset(0);end
     else if(position>=4)begin evaluation_enabled<=0;position<=0;end
     else begin position<=position+1'b1;preset(position+1'b1);end
    end
    if(release_event[4]&&!long_seen[4]&&editing)horizontal<=!horizontal;
   end
  end
 end
endmodule
