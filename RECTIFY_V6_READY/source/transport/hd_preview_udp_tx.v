`timescale 1ns/1ps
// PGV2 fragments with streaming application CRC and Ethernet FCS. No row copy.
module hd_preview_udp_tx #(parameter WIDTH=1280,HEIGHT=720,FRAGMENT=1400)(
 input clk,rst_n,start,input[31:0]session,frame,input[10:0]row,input[1:0]mode,input model,compressed,input[12:0]length,input[31:0]row_crc,
 output[11:0]src_addr,input[7:0]src_data,output reg busy,done,output reg tx_en,output reg[7:0]tx_data);
 reg[2:0]state;reg[12:0]index,total,offset,n;reg[31:0]sid,fid,rcrc,acrc,ecrc,afinal,efinal;reg[10:0]r;reg[1:0]m;reg mdl,codec;reg[7:0]abyte,ebyte;reg av,ev;
 wire[15:0]app_len=36+n,ip_len=28+app_len,udp_len=8+app_len;
 wire[31:0]sum=32'h4500+ip_len+32'h4000+32'h4011+32'hc0a8+32'h010b+32'hffff+32'hffff;
 wire[16:0]fold=sum[15:0]+sum[31:16];wire[15:0]ip_checksum=~(fold[15:0]+fold[16]);
 // The synchronous source is consumed only in payload cycles (index >= 74).
 // Read one byte ahead without gating the address through frame-length compares.
 // Header/CRC cycles may read an unused address; their frame bytes ignore it.
 assign src_addr=offset+index-13'd73;
 function[31:0]next_crc;input[31:0]c;input[7:0]b;reg[31:0]v;integer k;begin v=c^b;for(k=0;k<8;k=k+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;next_crc=v;end endfunction
 function[7:0]app_byte;input[12:0]i;
 begin case(i)
 0:app_byte="P";1:app_byte="G";2:app_byte="V";3:app_byte="2";
 4,5,6,7:app_byte=sid>>((i-4)*8);8,9,10,11:app_byte=fid>>((i-8)*8);
 12:app_byte=r[7:0];13:app_byte={5'd0,r[10:8]};14:app_byte=WIDTH&255;15:app_byte=WIDTH>>8;16:app_byte=HEIGHT&255;17:app_byte=HEIGHT>>8;
 18:app_byte={6'd0,m};19:app_byte=mdl;20:app_byte=codec;21:app_byte=2;
 22:app_byte=total[7:0];23:app_byte={3'd0,total[12:8]};24:app_byte=offset[7:0];25:app_byte={3'd0,offset[12:8]};26:app_byte=n[7:0];27:app_byte={3'd0,n[12:8]};
 28,29,30,31:app_byte=rcrc>>((i-28)*8);default:app_byte=src_data;
 endcase end endfunction
 function[7:0]frame_byte;input[12:0]i;
 begin case(i)
 0,1,2,3,4,5:frame_byte=255;6:frame_byte=2;7:frame_byte='h67;8:frame_byte='h60;9:frame_byte='h10;10:frame_byte=0;11:frame_byte=1;
 12:frame_byte=8;13:frame_byte=0;14:frame_byte='h45;15:frame_byte=0;16:frame_byte=ip_len[15:8];17:frame_byte=ip_len[7:0];18,19:frame_byte=0;20:frame_byte='h40;21:frame_byte=0;22:frame_byte=64;23:frame_byte=17;
 24:frame_byte=ip_checksum[15:8];25:frame_byte=ip_checksum[7:0];26:frame_byte=192;27:frame_byte=168;28:frame_byte=1;29:frame_byte=11;30,31,32,33:frame_byte=255;
 34,36:frame_byte='h1f;35,37:frame_byte='h90;38:frame_byte=udp_len[15:8];39:frame_byte=udp_len[7:0];40,41:frame_byte=0;default:frame_byte=app_byte(i-42);
 endcase end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;done<=0;tx_en<=0;tx_data<=0;index<=0;total<=0;offset<=0;n<=0;sid<=0;fid<=0;r<=0;m<=0;mdl<=0;codec<=0;rcrc<=0;acrc<=0;ecrc<=0;afinal<=0;efinal<=0;abyte<=0;ebyte<=0;av<=0;ev<=0;end
  else begin
   done<=0;tx_en<=0;av<=0;ev<=0;if(av)acrc<=next_crc(acrc,abyte);if(ev)ecrc<=next_crc(ecrc,ebyte);
   case(state)
    0:if(start)begin busy<=1;sid<=session;fid<=frame;r<=row;m<=mode;mdl<=model;codec<=compressed;total<=length;rcrc<=row_crc;offset<=0;n<=length>FRAGMENT?FRAGMENT:length;index<=0;state<=1;end
    1:begin tx_en<=1;tx_data<=index==7?8'hd5:8'h55;if(index==7)begin index<=0;acrc<=32'hffffffff;ecrc<=32'hffffffff;state<=2;end else index<=index+1'b1;end
    2:begin tx_en<=1;tx_data<=frame_byte(index);ebyte<=frame_byte(index);ev<=1;if(index>=42)begin abyte<=app_byte(index-42);av<=1;end
     if(index==73+n)begin index<=0;state<=3;end else index<=index+1'b1;
    end
    3:begin tx_en<=1;ev<=1;if(index==0)begin afinal<=~next_crc(acrc,abyte);tx_data<=~next_crc(acrc,abyte);ebyte<=~next_crc(acrc,abyte);end else begin tx_data<=afinal>>(index*8);ebyte<=afinal>>(index*8);end
     if(index==3)begin index<=0;state<=4;end else index<=index+1'b1;
    end
    4:begin tx_en<=1;if(index==0)begin efinal<=~next_crc(ecrc,ebyte);tx_data<=~next_crc(ecrc,ebyte);end else tx_data<=efinal>>(index*8);if(index==3)begin index<=0;state<=5;end else index<=index+1'b1;end
    5:if(index==11)begin index<=0;if(offset+n==total)begin busy<=0;done<=1;state<=0;end else begin offset<=offset+n;n<=total-offset-n>FRAGMENT?FRAGMENT:total-offset-n;state<=1;end end else index<=index+1'b1;
    default:state<=0;
   endcase
  end
 end
endmodule
