`timescale 1ns/1ps
module tb_photo_encoder_exact;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0;reg [10:0]row=0;reg [11:0]ea=0;
 wire ba,bb,da,db,ca,cb;wire[12:0]la,lb;wire[31:0]cra,crb;
 wire[10:0]aa,ab;wire[7:0]oa,ob;
 // Match network_pair_reader: one synchronous row-cache read per clock.
 reg[15:0]mem[0:1919];reg[15:0]a1=0,b1=0;
 always @(posedge clk)begin a1<=mem[aa];b1<=mem[ab];end
 rgb565_line_encoder #(.WIDTH(1920),.OVERLAP_GROUPS(1)) fast(
 .clk(clk),.rst_n(rst_n),.start(start),.row(row),.busy(ba),.done(da),.compressed(ca),.length(la),.row_crc(cra),
 .line_addr(aa),.line_data(a1),.encoded_addr(ea),.encoded_data(oa));
 rgb565_line_encoder #(.WIDTH(1920),.OVERLAP_GROUPS(0)) reference(
 .clk(clk),.rst_n(rst_n),.start(start),.row(row),.busy(bb),.done(db),.compressed(cb),.length(lb),.row_crc(crb),
 .line_addr(ab),.line_data(b1),.encoded_addr(ea),.encoded_data(ob));
 function[31:0]crcbyte;input[31:0]c0;input[7:0]b;reg[31:0]c;integer j;
 begin c=c0^b;for(j=0;j<8;j=j+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crcbyte=c;end endfunction
 integer p,k,cycles,tfast,tref;reg fa,fb;reg[31:0]gold;reg[15:0]seed=16'h8231;
 initial begin
  repeat(6)@(negedge clk);rst_n=1;
  for(p=0;p<6;p=p+1)begin
   row=p*131;
   for(k=0;k<1920;k=k+1)begin
    seed={seed[14:0],seed[15]^seed[13]^seed[12]^seed[10]};
    case(p)
     0:mem[k]=0;
     1:mem[k]=16'h7bef;
     2:mem[k]=((k/16)%2)?16'hffff:16'h0000;
     3:mem[k]={k[7:3],k[7:2],k[7:3]};
     4:mem[k]=seed;
     5:mem[k]=k%32<24?16'h0841:seed;
    endcase
   end
   gold=crcbyte(crcbyte(32'hffffffff,row[7:0]),{5'd0,row[10:8]});
   for(k=0;k<1920;k=k+1)gold=crcbyte(crcbyte(gold,mem[k][7:0]),mem[k][15:8]);
   @(negedge clk);start=1;@(negedge clk);start=0;fa=0;fb=0;cycles=0;tfast=0;tref=0;
   while(!fa||!fb)begin
    @(negedge clk);cycles=cycles+1;
    if(da)begin fa=1;tfast=cycles;end
    if(db)begin fb=1;tref=cycles;end
    if(cycles>20000)$fatal(1,"encoder timeout");
   end
   if(ca!==cb||la!==lb||cra!==crb||cra!==~gold)$fatal(1,"metadata/CRC mismatch pattern=%0d fast=(%0d,%0d,%08x) reference=(%0d,%0d,%08x) gold=%08x",p,ca,la,cra,cb,lb,crb,~gold);
   if(tfast>=tref)$fatal(1,"overlap did not reduce cycles");
   if(ca)for(k=0;k<la;k=k+1)begin
    ea=k;repeat(2)@(negedge clk);
    if(oa!==ob)$fatal(1,"compressed byte mismatch pattern=%0d byte=%0d fast=%02x reference=%02x",p,k,oa,ob);
   end
   $display("PASS exact 1920-pixel row pattern=%0d compressed=%0d length=%0d fast=%0d reference=%0d",p,ca,la,tfast,tref);
  end
  $display("PASS single-slot photo encoder exact compressed bytes and independent raw-pixel CRC across six patterns");$finish;
 end
 initial begin #4000000;$fatal(1,"watchdog");end
endmodule
