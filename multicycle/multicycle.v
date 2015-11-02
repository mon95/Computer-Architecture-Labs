
// Instruction Memory Design
module D_ff_Mem (input clk, input reset, input regWrite, input decOut1b,input init, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=init;
	else
		if(regWrite == 1 && decOut1b==1) begin q=d; end
	end
endmodule

module register_Mem(input clk,input reset,input regWrite,input decOut1b,input [15:0]init, input [15:0] d_in, output [15:0] q_out);
	D_ff_Mem dMem0 (clk,reset,regWrite,decOut1b,init[0],d_in[0],q_out[0]);
	D_ff_Mem dMem1 (clk,reset,regWrite,decOut1b,init[1],d_in[1],q_out[1]);
	D_ff_Mem dMem2 (clk,reset,regWrite,decOut1b,init[2],d_in[2],q_out[2]);
	D_ff_Mem dMem3 (clk,reset,regWrite,decOut1b,init[3],d_in[3],q_out[3]);
	
	D_ff_Mem dMem4 (clk,reset,regWrite,decOut1b,init[4],d_in[4],q_out[4]);
	D_ff_Mem dMem5 (clk,reset,regWrite,decOut1b,init[5],d_in[5],q_out[5]);
	D_ff_Mem dMem6 (clk,reset,regWrite,decOut1b,init[6],d_in[6],q_out[6]);
	D_ff_Mem dMem7 (clk,reset,regWrite,decOut1b,init[7],d_in[7],q_out[7]);

	D_ff_Mem dMem8 (clk,reset,regWrite,decOut1b,init[8],d_in[8],q_out[8]);
	D_ff_Mem dMem9 (clk,reset,regWrite,decOut1b,init[9],d_in[9],q_out[9]);
	D_ff_Mem dMem10 (clk,reset,regWrite,decOut1b,init[10],d_in[10],q_out[10]);
	D_ff_Mem dMem11 (clk,reset,regWrite,decOut1b,init[11],d_in[11],q_out[11]);
	
	D_ff_Mem dMem12 (clk,reset,regWrite,decOut1b,init[12],d_in[12],q_out[12]);
	D_ff_Mem dMem13 (clk,reset,regWrite,decOut1b,init[13],d_in[13],q_out[13]);
	D_ff_Mem dMem14 (clk,reset,regWrite,decOut1b,init[14],d_in[14],q_out[14]);
	D_ff_Mem dMem15 (clk,reset,regWrite,decOut1b,init[15],d_in[15],q_out[15]);
endmodule

module decoder4to16( input [3:0] destReg, output reg [15:0] decOut);
	always@(destReg)
	case(destReg)
			4'b0000: decOut=16'b0000000000000001; 
			4'b0001: decOut=16'b0000000000000010;
			4'b0010: decOut=16'b0000000000000100;
			4'b0011: decOut=16'b0000000000001000;
			4'b0100: decOut=16'b0000000000010000;
			4'b0101: decOut=16'b0000000000100000;
			4'b0110: decOut=16'b0000000001000000;
			4'b0111: decOut=16'b0000000010000000;
			4'b1000: decOut=16'b0000000100000000; 
			4'b1001: decOut=16'b0000001000000000;
			4'b1010: decOut=16'b0000010000000000;
			4'b1011: decOut=16'b0000100000000000;
			4'b1100: decOut=16'b0001000000000000;
			4'b1101: decOut=16'b0010000000000000;
			4'b1110: decOut=16'b0100000000000000;
			4'b1111: decOut=16'b1000000000000000;
	endcase
endmodule

module mux16to1( input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15, input [3:0] Sel, output reg [15:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15 or Sel)
	case (Sel)
				4'b0000: outBus=outR0;
				4'b0001: outBus=outR1;
				4'b0010: outBus=outR2;
				4'b0011: outBus=outR3;
				4'b0100: outBus=outR4;
				4'b0101: outBus=outR5;
				4'b0110: outBus=outR6;
				4'b0111: outBus=outR7;
				4'b1000: outBus=outR8;
				4'b1001: outBus=outR9;
				4'b1010: outBus=outR10;
				4'b1011: outBus=outR11;
				4'b1100: outBus=outR12;
				4'b1101: outBus=outR13;
				4'b1110: outBus=outR14;
				4'b1111: outBus=outR15;
	endcase
endmodule

module Mem(input clk, input reset,input memWrite,input memRead, input [15:0] pc, input [15:0] dataIn,output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,decOut;

	decoder4to16 dec0( pc[4:1], decOut);
	
	register_Mem r0(clk,reset,memWrite,decOut[0],16'b 101_0000_0001_00001,dataIn,Qout0); //addi $r1,$r0,1
	register_Mem r1(clk,reset,memWrite,decOut[1],16'b 101_0000_0010_00010,dataIn,Qout1); //addi $r2,$r0,2
	register_Mem r2(clk,reset,memWrite,decOut[2],16'b 000_0001_0010_00011,dataIn,Qout2); //add $r3,$r1,$r2
	register_Mem r3(clk,reset,memWrite,decOut[3],16'b 001_0010_0010_00000,dataIn,Qout3); //mul $r2,$r2
	
	register_Mem r4(clk,reset,memWrite,decOut[4],16'b 100_0000_0000_00100,dataIn,Qout4); //mflo $r4
	register_Mem r5(clk,reset,memWrite,decOut[5],16'b 000_0100_0011_00111,dataIn,Qout5); //add $r7, $r4, $r3
	register_Mem r6(clk,reset,memWrite,decOut[6],16'b 111_0100_0111_11010,dataIn,Qout6); //sw $r7,26($r4)
	register_Mem r7(clk,reset,memWrite,decOut[7],16'b 000_0111_0001_00111,dataIn,Qout7);  //add $r7,$r7,$r1
	
	register_Mem r8(clk,reset,memWrite,decOut[8],16'b 000_0000_0111_01000,dataIn,Qout8); //add $r8, $r0, $r7
	register_Mem r9(clk,reset,memWrite,decOut[9],16'b 110_0100_0111_11010,dataIn,Qout9); //lw $r7, 26($r4)
	register_Mem r10(clk,reset,memWrite,decOut[10],16'b 000_1000_0100_01100,dataIn,Qout10); //add $r12,$r8,$r4
	register_Mem r11(clk,reset,memWrite,decOut[11],16'b 010_1100_0111_00000,dataIn,Qout11); //div $r12,$r7
	
	register_Mem r12(clk,reset,memWrite,decOut[12],16'b 011_0000_0000_00101,dataIn,Qout12); //mfhi $r5
	register_Mem r13(clk,reset,memWrite,decOut[13],16'b 000_0101_0001_00110,dataIn,Qout13); //add $r6, $r5, $r1
	register_Mem r14(clk,reset,memWrite,decOut[14],16'd0,dataIn,Qout14);
	register_Mem r15(clk,reset,memWrite,decOut[15],16'd0,dataIn,Qout15);	//This memory location stores contents of $r7
	
	mux16to1 mMem (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc[4:1],IR);
endmodule
//Instruction Memory Design Ends

//Register File Design
module D_ff (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1'b1)
		q=0;
	else
		if(regWrite == 1'b1 && decOut1b==1'b1) begin q=d; end
	end
endmodule

module register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
endmodule

module registerSet( input clk, input reset, input regWrite, input [15:0] decOut, input [15:0] writeData,  output [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15 );
	register16bit r0 (clk, reset, regWrite, decOut[0] , writeData , outR0 );
	register16bit r1 (clk, reset, regWrite, decOut[1] , writeData , outR1 );
	register16bit r2 (clk, reset, regWrite, decOut[2] , writeData , outR2 );
	register16bit r3 (clk, reset, regWrite, decOut[3] , writeData , outR3 );
	register16bit r4 (clk, reset, regWrite, decOut[4] , writeData , outR4 );
	register16bit r5 (clk, reset, regWrite, decOut[5] , writeData , outR5 );
	register16bit r6 (clk, reset, regWrite, decOut[6] , writeData , outR6 );
	register16bit r7 (clk, reset, regWrite, decOut[7] , writeData , outR7 );
	register16bit r8 (clk, reset, regWrite, decOut[8] , writeData , outR8 );
	register16bit r9 (clk, reset, regWrite, decOut[9] , writeData , outR9 );
	register16bit r10 (clk, reset, regWrite, decOut[10] , writeData , outR10 );
	register16bit r11(clk, reset, regWrite, decOut[11] , writeData , outR11 );
	register16bit r12 (clk, reset, regWrite, decOut[12] , writeData , outR12 );
	register16bit r13 (clk, reset, regWrite, decOut[13] , writeData , outR13 );
	register16bit r14 (clk, reset, regWrite, decOut[14] , writeData , outR14 );
	register16bit r15 (clk, reset, regWrite, decOut[15] , writeData , outR15 );
endmodule

module registerFile(input clk, input reset, input regWrite, input [3:0] srcRegA, input [3:0] srcRegB, 
		input [3:0] destReg,  input [15:0] writeData, output [15:0] outBusA, output [15:0] outBusB );
	wire [15:0] decOut;
	wire [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15;
	decoder4to16 d0 (destReg,decOut);
	registerSet rSet0(clk, reset, regWrite, decOut, writeData, outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15);
	mux16to1 m1(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,srcRegA,outBusA);
	mux16to1 m2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,srcRegB,outBusB);
endmodule
//Register File Design Ends

//D_ff for IR register
module D_ff_IR (input IRWrite, input d, output reg q);
	always @ (IRWrite or d)
		begin
		if(IRWrite)
			q=d;
		end
endmodule

//Register to be used for IR register
module register16bit_IR(IRWrite, writeData, outBus);

	input IRWrite;
	input [15:0] writeData;
	output [15:0] outBus;
	D_ff_IR d0(IRWrite, writeData[0], outBus[0]);
	D_ff_IR d1(IRWrite, writeData[1], outBus[1]);
	D_ff_IR d2(IRWrite, writeData[2], outBus[2]);
	D_ff_IR d3(IRWrite, writeData[3], outBus[3]);
	D_ff_IR d4(IRWrite, writeData[4], outBus[4]);
	D_ff_IR d5(IRWrite, writeData[5], outBus[5]);
	D_ff_IR d6(IRWrite, writeData[6], outBus[6]);
	D_ff_IR d7(IRWrite, writeData[7], outBus[7]);
	D_ff_IR d8(IRWrite, writeData[8], outBus[8]);
	D_ff_IR d9(IRWrite, writeData[9], outBus[9]);
	D_ff_IR d10(IRWrite, writeData[10], outBus[10]);
	D_ff_IR d11(IRWrite, writeData[11], outBus[11]);
	D_ff_IR d12(IRWrite, writeData[12], outBus[12]);
	D_ff_IR d13(IRWrite, writeData[13], outBus[13]);
	D_ff_IR d14(IRWrite, writeData[14], outBus[14]);
	D_ff_IR d15(IRWrite, writeData[15], outBus[15]);
endmodule

module mux2to1_16bits(input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);
					 
	//Write your code here
	always@(in1 or in2 or sel)
	case(sel)
	 1'b0: muxout = in1;
	 1'b1: muxout = in2;  
	endcase
					 
endmodule

module mux2to1_4bits(input [3:0] in1, input [3:0] in2, input sel, output reg [3:0] muxout);
					 
	//Write your code here
	always@(in1 or in2 or sel)
	case(sel)
	 1'b0: muxout = in1;
	 1'b1: muxout = in2;  
	endcase
					 
endmodule

module signExt5to16( input [4:0] offset, output reg [15:0] signExtOffset);
						 
	//Write your code here
	always@(offset)
	begin
	 signExtOffset[15:5] = {11{offset[4]}};
	 signExtOffset[4:0] = offset[4:0];
	end
					 
endmodule

module mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3, input [1:0] sel, output reg [15:0] muxout);
					 
	//Write your code heremux4
	always@(in1 or in2 or sel)
	case(sel)
	 2'b00: muxout = in1;
	 2'b01: muxout = in2;
	 2'b10: muxout = in3;	  
	endcase
	
					 
endmodule

					 
module ctrlCkt	(input clk, input reset, input [2:0] opcode, output reg PCWrite,output reg memRead,output reg memToReg,
				 output reg memWrite,output reg ALUsrcA,output reg regWrite,output reg regDest,output reg IRWrite,output reg IorD,
				 output reg loWrite,output reg hiWrite,output reg [1:0] toReg,output reg [1:0] ALUOp,output reg [1:0]ALUsrcB);
					 
	//Write your code here
	reg[3:0] state;	
	always@(negedge clk or reset)
	if (reset == 1)
	 begin
	    state = 4'b0000; 
	    
      PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b00;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b0;
	 end
	 
	else
	  
	begin
	  
	case(state)
	 4'b0000: 
	   begin
	    state = 4'b0001;
	       
	    PCWrite = 1'b1;
	    IorD = 1'b1;
	    memRead = 1'b1;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b1;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b01;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b00;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b0; 
	 end
	 
	 4'b0001: 
	   begin
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b00;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b0;
	     case(opcode)
	       3'b000:   //add instr
	       begin
	         state = 4'b0010;
	       end  
	       
	       //fall through for mul/div possible??
	       3'b001:   //mul
	       begin
	         state = 4'b0100;
	       end
	       
	       3'b010:   //div
	       begin
	         state = 4'b0100;
	       end
	       
	       3'b011:   //mfhi 
	       begin
	         state = 4'b0101;
	       end
	       
	       3'b100:   //mflo
	       begin
	         state = 4'b0101;
	       end
	       
	       3'b101:   //addi
	       begin
	         state = 4'b0110;
	       end
	       
	       3'b110:   //lw
	       begin
	         state = 4'b1000;
	       end
	       
	       3'b111:   //lw
	       begin
	         state = 4'b1000;
	       end
	     endcase
	    end    //END OF CASE FOR STATE 1
	     
	  4'b0010: 
	   begin
	    state = 4'b0011;  
      	    PCWrite = 1'b0; 
	         IorD = 1'b0;
      	    memRead = 1'b0;
  	        memWrite = 1'b0;
         	 IRWrite = 1'b0;
  	        ALUsrcA = 1'b1;
         	 ALUsrcB = 2'b00;
    	      ALUOp = 2'b00;
         	 hiWrite = 1'b0;
  	        loWrite = 1'b0;
         	 toReg = 2'b00;
	         memToReg = 1'b0;
         	 regDest = 1'b0;
    	      regWrite = 1'b0;
	    
	 end
	 
	 4'b0011: 
	   begin
	    state = 4'b0000;
	      
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b10;
    	 memToReg = 1'b0;
    	 regDest = 1'b1;
    	 regWrite = 1'b1;
	 end
	 
	 4'b0100: 
	   begin
	    state = 4'b0000;
	      
	    
      	    PCWrite = 1'b0; 
	         IorD = 1'b0;
      	    memRead = 1'b0;
  	        memWrite = 1'b0;
         	 IRWrite = 1'b0;
  	        ALUsrcA = 1'b1;
         	 ALUsrcB = 2'b00;
         	 
    	      case(opcode)
    	       3'b001: ALUOp = 2'b01;    //mul 
    	       3'b010: ALUOp = 2'b10;    //div
  	       endcase
  	       
         	 hiWrite = 1'b1;
  	        loWrite = 1'b1;
         	 toReg = 2'b00;
	         memToReg = 1'b0;
         	 regDest = 1'b0;
    	      regWrite = 1'b0;  
	   
	   end
  
   4'b0101: 
	   begin
	    state = 4'b0000;
	    
      	    PCWrite = 1'b0; 
	         IorD = 1'b0;
      	    memRead = 1'b0;
  	        memWrite = 1'b0;
         	 IRWrite = 1'b0;
  	        ALUsrcA = 1'b0;
         	 ALUsrcB = 2'b00;
    	      ALUOp = 2'b00;
         	 hiWrite = 1'b0;
  	        loWrite = 1'b0;
         	 case(opcode)
    	       3'b100: toReg = 2'b01;  //mflo
    	       3'b011: ALUOp = 2'b00;    //mfhi
  	       endcase
         	 
	         memToReg = 1'b0;
         	 regDest = 1'b1;
    	      regWrite = 1'b1;
	 end
	 
	 
	 4'b0110: 
	   begin
	    state = 4'b0111;
	    
	    
      	    PCWrite = 1'b0; 
	         IorD = 1'b0;
      	    memRead = 1'b0;
  	        memWrite = 1'b0;
         	 IRWrite = 1'b0;
  	        ALUsrcA = 1'b1;
         	 ALUsrcB = 2'b10;
    	      ALUOp = 2'b00;
         	 hiWrite = 1'b0;
  	        loWrite = 1'b0;
         	 toReg = 2'b00;
	         memToReg = 1'b0;
         	 regDest = 1'b0;
    	      regWrite = 1'b0;
	 
	 end
	 
	  4'b0111: 
	   begin
	    state = 4'b0000;
	      
	    
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b10;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b1;
	 end
	 
	 
	 
	 4'b1000: 
	 begin
	   
      	    PCWrite = 1'b0; 
	         IorD = 1'b0;
      	    memRead = 1'b0;
  	        memWrite = 1'b0;
         	 IRWrite = 1'b0;
  	        ALUsrcA = 1'b1;
         	 ALUsrcB = 2'b10;
    	      ALUOp = 2'b00;
         	 hiWrite = 1'b0;
  	        loWrite = 1'b0;
         	 toReg = 2'b00;
	         memToReg = 1'b0;
         	 regDest = 1'b0;
    	      regWrite = 1'b0;
	 
	 case(opcode)
	     3'b110:
	   begin
	    state = 4'b1001;  
	  end
	 
	   3'b111:
	 begin
	    state = 4'b1011;
	    
	 end
	 
	endcase
	end
	4'b1001: 
	   begin
	    state = 4'b1010;
	        
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b1;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b10;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b0;
	    
	 end
	 
	  4'b1010: 
	   begin
	    state = 4'b0000;
	      
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b0;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b00;
    	 memToReg = 1'b1;
    	 regDest = 1'b0;
    	 regWrite = 1'b1;
	 end
	 
	  4'b1011: 
	   begin
	    state = 4'b0000;
	      
	    PCWrite = 1'b0;
	    IorD = 1'b0;
	    memRead = 1'b0;
  	   memWrite = 1'b1;
    	 IRWrite = 1'b0;
    	 ALUsrcA = 1'b0;
    	 ALUsrcB = 2'b00;
    	 ALUOp = 2'b00;
    	 hiWrite = 1'b0;
    	 loWrite = 1'b0;
    	 toReg = 2'b10;
    	 memToReg = 1'b0;
    	 regDest = 1'b0;
    	 regWrite = 1'b0;
	 end
	 

	endcase
end
endmodule


//For Multiplication lo register stores least significant 16 bits of result and hi register stores most significant 16 bits of result
//For Division lo register stores quotient of result and hi register stores remainder of result
module alu(input [15:0] aluIn1, input [15:0] aluIn2, input [1:0] aluOp, output reg [15:0] aluOut1,output reg [15:0] aluOut2);
					 
	//Write your code here
	always@(aluIn1 or aluIn2 or aluOp)
	case(aluOp)
	  2'b00: 
	   begin
	     aluOut1 = aluIn1 + aluIn2;
	     aluOut2 = 0;
	   end
	     
	  2'b01: 
	   begin
	     {aluOut2, aluOut1} = aluIn1 * aluIn2;
	   end
	   
	   2'b10:
	   begin
	     aluOut1 = aluIn1 / aluIn2;
	     aluOut2 = aluIn1 % aluIn2;
	   end
	 endcase 
	 
endmodule

//top module
module multiCycle(input clk, input reset, output [15:0] Result );
				 
	//Write your code here
	//Create instance of register16bit_IR register for IR register. 		
	
	wire pcWrite;
	wire[15:0] aluOut1;
	wire[15:0] pcout;
	
	wire IorD;
	wire[15:0] pcmemMuxOut;
	
	wire[15:0] regRt;
	wire[15:0] regRs;
	wire memWrite;
	wire memRead;
	
	wire[15:0] IRin;
	wire[15:0] IR;
	wire IRWrite;

	wire[15:0] mdrOut;
	wire[1:0] aluOp;
	
	wire regDest;
	wire memToReg;
	
	wire[3:0] mux_4bit_out;
	
	wire hiWrite;
  wire loWrite;
  wire[15:0] hiOut;
  wire[15:0] loOut;
  wire[15:0] actAluOut;
  
  wire[1:0] toReg;
  
  
  wire aluSrcA;
  wire[1:0] aluSrcB;
  wire[15:0] aluIn2;
  wire[15:0] aluIn1;
    
          wire[15:0] signExtOffset;
  
  wire[15:0] aluOut2;
  
  wire[15:0] writeData;
 wire[15:0] outBusA;
 wire[15:0] outBusB;

  
    /*
  		 
module ctrlCkt	(input clk, input reset, input [2:0] opcode, output reg PCWrite,output reg memRead,output reg memToReg,
				 output reg memWrite,output reg ALUsrcA,output reg regWrite,output reg regDest,output reg IRWrite,output reg IorD,
				 output reg loWrite,output reg hiWrite,output reg [1:0] toReg,output reg [1:0] ALUOp,output reg [1:0]ALUsrcB);
	*/
	
  //register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	register16bit PC(clk,reset, pcWrite, 1'b1, aluOut1, pcout);

  //	mux2to1_16bits (input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);	
	mux2to1_16bits mux_pc_mem(Result, pcout, IorD, pcmemMuxOut);
	
	// Mem(input clk, input reset,input memWrite,input memRead, input [15:0] pc, input [15:0] dataIn,output [15:0] IR );
	Mem mem1(clk, reset, memWrite, memRead, pcmemMuxOut, regRt, IRin);
	
	//register16bit_IR(IRWrite, writeData, outBus);
	//
	/*input IRWrite;
	input [15:0] writeData;
	output [15:0] outBus;
	*/
	register16bit_IR instrReg(IRWrite, IRin, IR); 

  register16bit MDR(clk,reset, 1'b1, 1'b1, IRin, mdrOut);
  
 
  //mux2to1_4bits(input [3:0] in1, input [3:0] in2, input sel, output reg [3:0] muxout);		
  mux2to1_4bits mux2to1_4bits1(IR[8:5], IR[3:0], regDest, mux_4bit_out);
     
  //module signExt5to16( input [4:0] offset, output reg [15:0] signExtOffset);
  signExt5to16 signExt5to161(IR[4:0], signExtOffset);
  
  
//  mux2to1_16bits (input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);
  
  mux2to1_16bits muxToRegFile(Result, mdrOut, memToReg, writeData);
 
  
 /*
  registerFile(input clk, input reset, input regWrite, input [3:0] srcRegA, input [3:0] srcRegB, 
		input [3:0] destReg,  input [15:0] writeData, output [15:0] outBusA, output [15:0] outBusB );
 */
 
registerFile regFile(clk, reset, regWrite, IR[12:9], IR[8:5], mux_4bit_out, writeData, outBusA, outBusB);

register16bit A(clk,reset, 1'b1, 1'b1, outBusA, regRs);
register16bit B(clk,reset, 1'b1, 1'b1, outBusB, regRt);


  mux2to1_16bits muxToAluIn1(pcout, regRs, aluSrcA, aluIn1);
 // mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3, input [1:0] sel, output reg [15:0] muxout);			 
  mux4to1_16bits  mux4to1_16bits1(regRt, 16'b0000000000000010, signExtOffset, aluSrcB, aluIn2);
  
  
 
  
  //alu(input [15:0] aluIn1, input [15:0] aluIn2, input [1:0] aluOp, output reg [15:0] aluOut1,output reg [15:0] aluOut2);
				alu alu1(aluIn1, aluIn2, aluOp, aluOut1, aluOut2);
				
				
  //register16bit MDR(clk,reset, 1'b1, 1'b1, IRin, mdrOut);
register16bit hi(clk,reset, hiWrite, 1'b1, aluOut2, hiOut);
register16bit lo(clk,reset, loWrite, 1'b1, aluOut1, loOut);
register16bit ALUOUT(clk,reset, 1'b1, 1'b1, aluOut1, actAluOut);
mux4to1_16bits  mux4to1_16bits2(hiOut, loOut, actAluOut, toReg, Result);	 
ctrlCkt ctrlCkt1(clk, reset, IR[15:13], pcWrite, memRead, memToReg, memWrite, aluSrcA, regWrite, regDest, IRWrite, IorD, loWrite, hiWrite, toReg, aluOp, aluSrcB);

endmodule







module multiCycleTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	multiCycle uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10  reset=0;	
		
		#570 $finish; 
	end
endmodule


