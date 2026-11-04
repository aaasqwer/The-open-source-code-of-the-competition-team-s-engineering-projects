`timescale 1ns/1ps
module tb_pc4_window;
 parameter WINDOW=4,LOSS=0;
 reg clk=0,rst=0,cap=0,start=0,active=1,au=0;always #4 clk=~clk;
 reg [7:0] data=0;reg [63:0]ack=0;wire busy,ip,abort,fault,en;wire [7:0]tx;
 ethernet_image_tx #(.WINDOW(WINDOW),.CHUNK_BYTES(1400),.COALESCE_TICKS(8),.ACK_TICKS(15000),.MAX_RETRIES(4)) dut(
 .clk(clk),.rst_n(rst),.capture(cap),.capture_sequence(32'h34567890),.ack_update(au),.ack(ack),.image_start(start),.image_data(data),.image_packet_active(active),
 .image_busy(busy),.image_pending(ip),.abort(abort),.fault(fault),.tx_en(en),.tx_data(tx));
 reg [7:0] bytes[0:1600];reg last=0;integer used=0,cycle=0,fd,frames=0,expected=0;
 integer seqs[0:255],due[0:255],push=0,pop=0,sequence,first_cycle=0;
 reg dropped=0,delayed=0;reg [1023:0] path;
 always @(negedge clk)begin
  cycle=cycle+1;au=0;
  if(en)begin bytes[used]=tx;used=used+1;$fwrite(fd,"%02x",tx);end
  if(!en&&last)begin
   $fwrite(fd,"\n");frames=frames+1;
   sequence={bytes[61],bytes[60],bytes[59],bytes[58]};
   if(LOSS&&sequence==1&&!dropped)dropped=1;
   else if(sequence<=expected)begin
    if(sequence==expected)expected=expected+1;
    seqs[push]=sequence;due[push]=cycle+5000;
    if(LOSS&&sequence==0&&!delayed)begin due[push]=cycle+15100;delayed=1;end
    push=push+1;
   end
   used=0;
  end
  last=en;
  if(pop<push&&cycle>=due[pop])begin ack={seqs[pop],32'h34567890};au=1;pop=pop+1;end
  if(abort||fault)$fatal(1,"unexpected abort");
 end
 integer k;
 initial begin
  if(!$value$plusargs("OUT=%s",path))path="verification/pc4_window.hex";fd=$fopen(path,"w");
  repeat(5)@(negedge clk);rst=1;repeat(3)@(negedge clk);cap=1;@(negedge clk);cap=0;first_cycle=cycle;
  // Wrong session and ACK beyond sent data must not retire any slot.
  ack=64'hfffffffe_deadbeef;au=1;@(negedge clk);au=0;
  for(k=0;k<14101;k=k+1)begin
   while(busy)@(negedge clk);data=(k*73+(k>>4))&255;start=1;@(negedge clk);start=0;@(negedge clk);
  end
  active=0;wait(!ip);repeat(3)@(negedge clk);
  if(expected!=11)$fatal(1,"missing chunks %0d",expected);
  $fclose(fd);$display("PASS WINDOW=%0d LOSS=%0d CYCLES=%0d FRAMES=%0d",WINDOW,LOSS,cycle-first_cycle,frames);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
