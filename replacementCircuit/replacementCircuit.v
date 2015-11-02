
module D_FF(input clk,input set,input reset,output reg Q);
  always@(negedge clk)
  begin
    if(reset==1'b1)
      Q=0;
      
    else
      begin
        if(set)
          Q=1;
      end
  end
endmodule

module decoder2to4(input [1:0] muxOut,output reg[3:0] decOut);
  always@(muxOut)
  case(muxOut)
    2'b00: decOut = 4'b0001;
    2'b01: decOut = 4'b0010;
    2'b10: decOut = 4'b0100;
    2'b11: decOut = 4'b1000;
  endcase
endmodule
  
module mux(input [1:0] LineIndex,input[1:0] LRUWay,input Hit,output reg [1:0] muxOut);
  always@(LineIndex or LRUWay or Hit)
  case(Hit)
    1'b0: muxOut = LRUWay;
    1'b1: muxOut = LineIndex;
  endcase  
endmodule

module NxN_DFFs(input clk,input reset,input[3:0] decOut,output [3:0] NxNOut);
  //D_FF(input clk,input set,input reset,output reg Q);
  wire q00, q01, q02, q03; 
  wire q10, q11, q12, q13;
  wire q20, q21, q22, q23;
  wire q30, q31, q32, q33;


  wire reset1;
  wire reset2;
  wire reset3;
  wire reset4;
 
 
  assign reset1 = reset || decOut[0];
  assign reset2 = reset || decOut[1];
  assign reset3 = reset || decOut[2];
  assign reset4 = reset || decOut[3];
  
  
  D_FF dff00(clk, decOut[0], reset1, q00);
  D_FF dff01(clk, decOut[0], reset2, q01);
  D_FF dff02(clk, decOut[0], reset3, q02);
  D_FF dff03(clk, decOut[0], reset4, q03);
  
  assign NxNOut[0] = q00 || q01 || q02 || q03;
  
  
  D_FF dff10(clk, decOut[1], reset1, q10);
  D_FF dff11(clk, decOut[1], reset2, q11);
  D_FF dff12(clk, decOut[1], reset3, q12);
  D_FF dff13(clk, decOut[1], reset4, q13);
  
  assign NxNOut[1] = q10 || q11 || q12 || q13;
  
  D_FF dff20(clk, decOut[2], reset1, q20);
  D_FF dff21(clk, decOut[2], reset2, q21);
  D_FF dff22(clk, decOut[2], reset3, q22);
  D_FF dff23(clk, decOut[2], reset4, q23);
  
  assign NxNOut[2] = q20 || q21 || q22 || q23;
  
  
  D_FF dff30(clk, decOut[3], reset1, q30);
  D_FF dff31(clk, decOut[3], reset2, q31);
  D_FF dff32(clk, decOut[3], reset3, q32);
  D_FF dff33(clk, decOut[3], reset4, q33);
  
  assign NxNOut[3] = q30 || q31 || q32 || q33;
  
  endmodule

module prio_Enc(input reset, input [3:0]NxNOut,output reg [1:0] LRUWay);
  always@(reset or NxNOut)
  begin
    if (reset)
      LRUWay = 2'b00;
    else
    begin
      if ( NxNOut[0] == 0)
        LRUWay = 2'b00;
      else if (NxNOut[1] == 0)
        LRUWay = 2'b01;
      else if (NxNOut[2] == 0)
        LRUWay = 2'b10;
      else
        LRUWay = 2'b11;
    end
  end
endmodule

module LRU(input [1:0] LineIndex,input clk,input reset, input Hit , output [1:0] LRUWay, output [1:0] mOut, output [3:0]
dOut, output [3:0] nOut);
    
    mux mux1(LineIndex, LRUWay, Hit, mOut);
    decoder2to4 decoder2to4_1(mOut, dOut);
    NxN_DFFs nndffs(clk, reset, dOut, nOut);
    prio_Enc pe(reset, nOut, LRUWay);
        
endmodule



module testbench;
reg [1:0] LineIndex;
reg clk;
reg reset;
reg Hit;
wire [1:0] LRUWay;
wire [1:0] mOut;
wire [3:0] dOut, nOut;
LRU uut (.LineIndex(LineIndex), .clk(clk), .reset(reset), .Hit(Hit), .LRUWay(LRUWay), .mOut(mOut), .dOut(dOut),
.nOut(nOut) );
always
#5 clk=~clk;
initial
begin
LineIndex = 0;
reset = 1;
Hit = 0;
clk = 0;
$monitor($time," Current_LRUWay=%d Hit=%d LineIndex=%d ",LRUWay,Hit,LineIndex);
#8 Hit=1;
#2 reset=0; LineIndex=3'd0;
#10 LineIndex=3'd1;
#10 LineIndex=3'd2;
#10 LineIndex=3'd3;
#10 Hit=0; LineIndex=3'd1;
#10 LineIndex=3'd0;
#10 LineIndex=3'd1;
#10 LineIndex=3'd2;
#10 LineIndex=3'd3;
#10 $finish;
end
endmodule