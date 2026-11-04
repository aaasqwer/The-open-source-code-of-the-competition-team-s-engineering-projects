`timescale 1ns/1ps
module sfr_digits(input clk,rst_n,valid,above,input[15:0]mtf_q16,quarter_q12,
 input[3:0]address,output [7:0]character);
 reg[4:0]state;reg[15:0]v0,v1,b0,b1,d0,d1;
 function[15:0] adjust;input[15:0]v;integer i;begin
  adjust=v;for(i=0;i<4;i=i+1)if(v[i*4+:4]>=5)adjust[i*4+:4]=v[i*4+:4]+3;
 end endfunction
 wire[15:0]a0=adjust(b0),a1=adjust(b1);
 wire[31:0]s0=mtf_q16*32'd1000+32768,s1=quarter_q12*32'd1000+2048;
 always@(posedge clk)begin
  if(!rst_n)begin state<=0;v0<=0;v1<=0;b0<=0;b1<=0;d0<=0;d1<=0;end
  else if(state==0)begin v0<=s0>>16;v1<=(s1>>12)>9999?9999:s1>>12;b0<=0;b1<=0;state<=1;end
  else if(state<=16)begin b0<={a0[14:0],v0[15]};b1<={a1[14:0],v1[15]};v0<=v0<<1;v1<=v1<<1;state<=state+1;end
  else begin d0<=b0;d1<=b1;state<=0;end
 end
 wire[3:0]local_addr=address>=5?address-5:address;
 wire[15:0]selected=address<5?d0:d1;
 wire[3:0]digit=local_addr==0?selected[15:12]:local_addr==2?selected[11:8]:local_addr==3?selected[7:4]:selected[3:0];
 assign character=valid?(local_addr==1?8'd46:8'd48+digit):
  (above&&address<5?(address==0?8'd62:address==1?8'd48:address==2?8'd46:address==3?8'd53:8'd48):
  (local_addr==0?8'd78:local_addr==1?8'd47:local_addr==2?8'd65:8'd32));
endmodule
