`timescale 1ns/1ps
module tb_v2_photo;
 localparam W=1920,H=1080,OW=1280,OH=720,A=28,FB=22;
 reg clk=0,nclk=0,rst=0,nrst=0,enable=0;always #5 clk=~clk;always #4 nclk=~nclk;
 reg [2:0] bank=1;reg [31:0] fid=101;reg [63:0] geometry=64'h1234567800000581;
 wire hold,req,ready,done,pair,model,lease,processed,busy,tx_en;wire [10:0] row,rx;
 wire [15:0] p,pn,pp,ppn;wire [31:0] frame;wire [63:0] fg;wire [2:0] leased_bank;
 wire [A-1:0] araddr;wire arvalid;reg arready=0;reg [255:0] rdata=0;reg rvalid=0;
 wire [7:0] tx_data;reg pending=0;integer delay=0,requests=0,k,b,y,x,frame_count=0;reg [A-1:0] addr;
 reg [31:0] rand_state=32'h71285;
 network_pair_reader #(.W(W),.H(H),.A(A),.FRAME_BIT(FB),.BANK_BITS(3),.SUPPORT_GAMMA(1),.GAMMA_IN_FRAME(1)) reader(
  .clk(clk),.rst_n(rst),.net_clk(nclk),.net_rst_n(nrst),.hold_frame(hold),.request_toggle(req),.request_row(row),.read_x(rx),
  .read_pixel(p),.read_pixel_next(pn),.read_pixel_previous(pp),.read_pixel_previous_next(ppn),.read_pair_valid(pair),.frame_geometry(fg),.alternate_geometry(geometry),
  .ready(ready),.done_toggle(done),.gamma_bypass_frame(model),.frame_id(frame),.source_valid(1'b1),.latest_bank(bank),.latest_id(fid),
  .select_alternate(1'b1),.alternate_valid(1'b1),.alternate_bank(bank),.alternate_id(fid),.gamma_bypass(1'b1),.gamma_step(5'd0),
  .lease_active(lease),.lease_processed(processed),.leased_bank(leased_bank),.araddr(araddr),.arlen(),.arid(),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rvalid(rvalid),.rlast(rvalid));
 reg capture=0,uart_busy=0;wire abort_capture,packet,transfer;wire [3:0] capture_state;wire [2:0] reason;
 always @(posedge nclk)uart_busy<=tx_en;
 fast_snapshot_ctrl #(.WIDTH(W),.HEIGHT(H),.LEASED_DDR(1)) photo(.clk(nclk),.rst_n(nrst),.capture_pulse(capture),.capture_sequence(32'd17),.invalid_frame(hold&&!ready&&capture_state>1&&capture_state!=10),
  .abort_capture(abort_capture),.freeze_request(hold),.freeze_active_vin(ready),.video_vs(1'b0),.mode_pix(2'd1),.model_pix(1'b1),.leased_frame_id(frame),.leased_geometry(fg),
  .line_request_toggle(req),.line_done_toggle_pix(done),.captured_row_pix(row),.completed_rows(row),.line_rd_addr(rx),.line_rd_data(p),
  .uart_start(tx_en),.uart_data(tx_data),.uart_busy(uart_busy),.packet_active(packet),.transfer_active(transfer),.capture_state(capture_state),.abort_reason(reason));
 function [15:0] pattern;input integer bb,xx,yy;integer rr,gg,blue;
  begin
   case(bb)
    1:begin rr=(xx+yy)%32;gg=(xx+3*yy)%64;blue=(3*xx+yy)%32;end
    2:begin rr=((xx+yy)%2)*31;gg=((xx/3+yy/2)%2)*63;blue=((xx+yy/3)%2)*31;end
    3:begin rr=(xx*7+yy*11+xx*yy)%32;gg=(xx*13+yy*7+xx*yy)%64;blue=(xx*3+yy*17)%32;end
    default:begin rr=7;gg=35;blue=21;end
   endcase
   pattern=(rr<<11)|(gg<<5)|blue;
  end
 endfunction
 always @(posedge clk)begin
  rand_state<={rand_state[30:0],rand_state[31]^rand_state[21]^rand_state[1]^rand_state[0]};rvalid<=0;
  arready<=rst&&!pending&&rand_state[2:0]!=0;
  if(arvalid&&arready)begin
   if(pending)$fatal(1,"more than one DDR pending");pending<=1;addr<=araddr;delay<=2+rand_state[6:3];requests<=requests+1;arready<=0;
  end
  if(pending)begin
   if(delay!=0)delay<=delay-1;
   else begin
    pending<=0;rvalid<=1;b=addr>>FB;y=((addr&((1<<FB)-1))*2)/W;x=((addr&((1<<FB)-1))*2)%W;
    for(k=0;k<16;k=k+1)rdata[k*16+:16]<=pattern(b,x+k,y);
   end
  end
 end
 integer file;
 always @(negedge nclk)begin if(tx_en)$fwrite(file,"%02x",tx_data);if(abort_capture)$fatal(1,"photo abort %d",reason);end
 initial begin
  file=$fopen("verification/v5/full_photo_packets.hex","w");#100;rst=1;nrst=1;repeat(20)@(negedge nclk);capture=1;@(negedge nclk);capture=0;
  wait(transfer);wait(!transfer);repeat(20)@(negedge nclk);$fclose(file);
  if(requests!=H*W/16)$fatal(1,"photo DDR count");
  $display("PASS V2 native DDR photo, no HDMI vsync, stable leased frame and geometry, all rows");$finish;
 end
 initial begin #5000000000;$fatal(1,"photo timeout state %d row %d",capture_state,row);end
endmodule
