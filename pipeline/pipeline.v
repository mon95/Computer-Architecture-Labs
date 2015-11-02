
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

module Mem(input clk, input reset,input memWrite,input memRead, input [15:0] pc, input [15:0] dataIn,output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,decOut;
	
	decoder4to16 dec0( pc[4:1], decOut);
	
	register_Mem r0(clk,reset,memWrite,decOut[0],16'b 101_0000_1000_01111,dataIn,Qout0); //addi $r8,$r0,15
	register_Mem r1(clk,reset,memWrite,decOut[1],16'b 101_0000_0101_00101,dataIn,Qout1); //addi $r5,$r0,5
	register_Mem r2(clk,reset,memWrite,decOut[2],16'b 101_0000_0010_00010,dataIn,Qout2); //addi $r2,$r0,2
	register_Mem r3(clk,reset,memWrite,decOut[3],16'b 101_0000_0000_00000,dataIn,Qout3); //addi $r0,$r0,0
	
	register_Mem r4(clk,reset,memWrite,decOut[4],16'b 001_1000_1000_00000,dataIn,Qout4); //mul $r8,$r8
	register_Mem r5(clk,reset,memWrite,decOut[5],16'b 100_0000_0000_01001,dataIn,Qout5); //mflo $r9
	register_Mem r6(clk,reset,memWrite,decOut[6],16'b 000_0101_0101_01010,dataIn,Qout6); //add $r10,$r5,$r5
	register_Mem r7(clk,reset,memWrite,decOut[7],16'b 101_0000_0100_00100,dataIn,Qout7);  //addi $r4,$r0,4
	
	register_Mem r8(clk,reset,memWrite,decOut[8],16'b 111_0000_0101_00000,dataIn,Qout8); //sw $r5,$r0(0)
	register_Mem r9(clk,reset,memWrite,decOut[9],16'b 010_1000_0010_00000,dataIn,Qout9); //div $r8,$r2
	register_Mem r10(clk,reset,memWrite,decOut[10],16'b 011_0000_0000_00001,dataIn,Qout10); //mfhi $r1
	register_Mem r11(clk,reset,memWrite,decOut[11],16'b 100_0000_0000_00111,dataIn,Qout11); //mflo $r7
	
	register_Mem r12(clk,reset,memWrite,decOut[12],16'b 000_0000_0000_00000,dataIn,Qout12);	//add $r0,$r0,$r0
	register_Mem r13(clk,reset,memWrite,decOut[13],16'b 110_0000_0110_00000,dataIn,Qout13); //lw $r6,$r0(0)
	register_Mem r14(clk,reset,memWrite,decOut[14],16'b 000_0101_0001_00101,dataIn,Qout14); //add $r5,$r5,$r1
	register_Mem r15(clk,reset,memWrite,decOut[15],16'b 000_0010_0001_00011,dataIn,Qout15); //add $r3,$r2,$r1
	
	mux16to1 mMem (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc[4:1],IR);
endmodule

//Memory Design Ends

// Register File Design
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
		register16bit r0 (clk, reset, 1'b0, decOut[0] , writeData , outR0 );
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

//Register 4 bits
module register4bit( input clk, input reset, input regWrite, input decOut1b, input [3:0] writeData, output  [3:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
endmodule

//Register 1 bit
module register1bit( input clk, input reset, input regWrite, input decOut1b, input writeData, output outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData, outR);
endmodule

//Register 2 bits
module register2bit( input clk, input reset, input regWrite, input decOut1b, input [1:0]writeData, output [1:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
endmodule

module adder(input [15:0] in1, input [15:0] in2, output reg [15:0] adder_out);
	always@(in1 or in2)
		adder_out = in1 +in2;
endmodule

module signExt5to16( input [4:0] offset, output reg [15:0] signExtOffset);
	always@(offset)
	begin
			signExtOffset={{11{offset[4]}},offset[4:0]};
	end
endmodule

module mux2to1_16bits(input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);
	 always@(in1 or in2 or sel)
	 begin
		case(sel)
			1'b0 : muxout = in1;
			1'b1 : muxout = in2;			
		endcase
	 end
endmodule

module mux2to1_4bits(input [3:0] in1, input [3:0] in2, input sel, output reg [3:0] muxout);
	 always@(in1 or in2 or sel)
	 begin
		case(sel)
			0 : muxout = in1;
			1 : muxout = in2;
		endcase
	 end
endmodule

//For Multiplication lo register stores least significant 16 bits of result and hi register stores most significant 16 bits of result
//For Division lo register stores quotient of result and hi register stores remainder of result
module alu(input [15:0] aluIn1, input [15:0] aluIn2, input [1:0] aluOp, output reg [15:0] aluOut1,output reg [15:0] aluOut2);
 reg [31:0] temp;
	always@(aluIn1 or aluIn2 or aluOp)
	begin
		case(aluOp)
			2'b00: aluOut1 = aluIn1 + aluIn2;
			2'b01: 
			begin
				temp = aluIn1 * aluIn2;
				aluOut1 = temp[15:0];
				aluOut2 = temp[31:16];
			end
			2'b10:
			begin
				aluOut1 = aluIn1 / aluIn2;
				aluOut2 = aluIn1 % aluIn2;
			end					 
		endcase
	end
endmodule

module mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3,input [15:0] in4, input [1:0] sel, output reg [15:0] muxout);
	 always@(in1 or in2 or in3 or in4 or sel)
	 begin
		case(sel)
			2'b00 : muxout = in1;
			2'b01 : muxout = in2;
			2'b10 : muxout = in3;
			2'b11 : muxout = in4;
		endcase
	 end
endmodule


module ctrlCkt	( 	input [2:0] opcode, output reg aluSrcB, output reg [1:0] aluOp, output reg hiWrite, output reg loWrite, output reg regDst, output reg memRead, output reg memWrite,output reg regWrite, output reg[1:0] toReg);
	
	//Write your code here
	always@(opcode)
	case(opcode)
	   3'b000: 
	   begin 
	     aluSrcB = 0;
	     aluOp = 0;
	     regDst = 1;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 1;
	     toReg = 2'b11;
	   end  
	   
	   3'b001: 
	   begin 
	     aluSrcB = 0;
	     aluOp = 1;
	     regDst = 0;
	     hiWrite = 1;
	     loWrite = 1;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 0;
	     toReg = 2'b00;
	   end  
	   
	   3'b010: 
	   begin 
	     aluSrcB = 0;
	     aluOp = 2'b10;
	     regDst = 0;
	     hiWrite = 1;
	     loWrite = 1;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 0;
	     toReg = 2'b00;
	   end  
	   
	   3'b011: 
	   begin 
	     aluSrcB = 0;
	     aluOp = 0;
	     regDst = 1;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 1;
	     toReg = 2'b00;
	   end  
	   
	   3'b100: 
	   begin 
	     aluSrcB = 0;
	     aluOp = 0;
	     regDst = 1;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 1;
	     toReg = 2'b01;
	   end  
	   
	   3'b101: 
	   begin 
	     aluSrcB = 1;
	     aluOp = 0;
	     regDst = 0;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 0;
	     memWrite = 0;
	     regWrite = 1;
	     toReg = 2'b11;
	   end  
	   
	   3'b110: 
	   begin 
	     aluSrcB = 1;
	     aluOp = 0;
	     regDst = 0;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 1;
	     memWrite = 0;
	     regWrite = 1;
	     toReg = 2'b10;
	   end  
	
	
	3'b111: 
	   begin 
	     aluSrcB = 1;
	     aluOp = 0;
	     regDst = 0;
	     hiWrite = 0;
	     loWrite = 0;
	     memRead = 0;
	     memWrite = 1;
	     regWrite = 0;
	     toReg = 2'b00;
	   end 
	    
	   endcase
	   
endmodule

module IF_ID(input clk, input reset,input regWrite, input decOut1b,input [15:0] instr, output [15:0] p0_intr);

	//Write your code here
	
	//register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	register16bit register16bit_IF_ID(clk, reset, regWrite, decOut1b, instr, p0_intr);
	
endmodule

module ID_EX(input clk, input reset,input regWrite, input decOut1b,input [15:0] regOut1,input [15:0] regOut2,
  input [15:0] sExtOut,input [3:0] inst_Rt,input [3:0] inst_Rd,input ctr_aluSrcB,input [1:0] ctr_aluOp, input ctr_hiWrite, input ctr_loWrite, 
  input ctr_regDst, input ctr_memRead, input ctr_memWrite,input ctr_regWrite, input[1:0] ctr_toReg,
  output [15:0] p1_regOut1,output [15:0] p1_regOut2,output [15:0] p1_sExtOut,output [3:0] p1_rt,output [3:0] p1_rd,
  output p1_aluSrcB, output [1:0] p1_aluOp, output p1_hiWrite, output p1_loWrite, output p1_regDst, output p1_memRead,
  output p1_memWrite,output p1_regWrite, output[1:0] p1_toReg);

	//Write your code here
	register16bit register16bit_ID_EX_1(clk, reset, regWrite, decOut1b, regOut1, p1_regOut1);    //Reg[Rs]
	register16bit register16bit_ID_EX_2(clk, reset, regWrite, decOut1b, regOut2, p1_regOut2);    //Reg[Rt]
	register16bit register16bit_ID_EX_3(clk, reset, regWrite, decOut1b, sExtOut, p1_sExtOut);    //sExt value
	
	//register4bit( input clk, input reset, input regWrite, input decOut1b, input [3:0] writeData, output  [3:0] outR );
	register4bit register4bit_RT(clk, reset, regWrite, decOut1b, inst_Rt, p1_rt);
  register4bit register4bit_RD(clk, reset, regWrite, decOut1b, inst_Rd, p1_rd);
  
  //Control signals
  register1bit register1bit_aluSrcB(clk, reset, regWrite, decOut1b, ctr_aluSrcB, p1_aluSrcB);
  register2bit register2bit_aluOp(clk, reset, regWrite, decOut1b, ctr_aluOp, p1_aluOp);
  	
	register1bit register1bit_ctr_hiWrite(clk, reset, regWrite, decOut1b, ctr_hiWrite, p1_hiWrite);
  register1bit register1bit_ctr_loWrite(clk, reset, regWrite, decOut1b, ctr_loWrite, p1_loWrite);
  
	register1bit register1bit_ctr_regDst(clk, reset, regWrite, decOut1b, ctr_regDst, p1_regDst);
	register1bit register1bit_ctr_memRead(clk, reset, regWrite, decOut1b, ctr_memRead, p1_memRead);	
	register1bit register1bit_ctr_memWrite(clk, reset, regWrite, decOut1b, ctr_memWrite, p1_memWrite);	

  register1bit register1bit_ctr_ctr_regWrite(clk, reset, regWrite, decOut1b, ctr_regWrite, p1_regWrite);	
  register2bit register2bit_ctr_toReg(clk, reset, regWrite, decOut1b, ctr_toReg, p1_toReg);

endmodule

module EX_MEM(input clk, input reset,input regWrite, input decOut1b,input [15:0] hiOut, input [15:0] loOut,
   input [15:0] aluOut,input [15:0] p1_regOut2,input [3:0] mux4bOut,input p1_memRead, input p1_memWrite,input p1_regWrite,
	input[1:0] p1_toReg,output[15:0] p2_hiOut,output [15:0] p2_loOut,output [15:0] p2_aluOut,
	output [15:0] p2_regOut2,output[3:0] p2_mux4bOut,output p2_memRead, output p2_memWrite,
	output p2_regWrite, output[1:0] p2_toReg );
	
	//Write your code here
	register16bit register16bit_EX_MEM_1(clk, reset, regWrite, decOut1b, hiOut, p2_hiOut);
  register16bit register16bit_EX_MEM_2(clk, reset, regWrite, decOut1b, loOut, p2_loOut);
	register16bit register16bit_EX_MEM_3(clk, reset, regWrite, decOut1b, aluOut, p2_aluOut);
	register16bit register16bit_EX_MEM_4(clk, reset, regWrite, decOut1b, p1_regOut2, p2_regOut2);
	
	register4bit register4bit_RT_OR_RD(clk, reset, regWrite, decOut1b, mux4bOut, p2_mux4bOut);
  
  //control
  register1bit register1bit_p1memRead(clk, reset, regWrite, decOut1b, p1_memRead, p2_memRead);
 	register1bit register1bit_p1memWrite(clk, reset, regWrite, decOut1b, p1_memWrite, p2_memWrite);	
	
	 
	register1bit register1bit_p1regWrite(clk, reset, regWrite, decOut1b, p1_regWrite, p2_regWrite);	
  register2bit register2bit_p1toReg(clk, reset, regWrite, decOut1b,p1_toReg, p2_toReg);
    
	
endmodule

module MEM_WB(input clk, input reset,input regWrite, input decOut1b,input [15:0] p2_hiOut, input [15:0] p2_loOut,
   input [15:0] p2_aluOut,input [15:0] memOut,input [3:0] p2_mux4bOut,input p2_regWrite,input[1:0] p2_toReg,
	output[15:0] p3_hiOut,output [15:0] p3_loOut,output [15:0] p3_aluOut,
	output [15:0] p3_memOut,output[3:0] p3_mux4bOut,output p3_regWrite, output[1:0] p3_toReg );

	//Write your code here
	register16bit register16bit_MEM_WB_1(clk, reset, regWrite, decOut1b, p2_hiOut, p3_hiOut);
  register16bit register16bit_MEM_WB_2(clk, reset, regWrite, decOut1b, p2_loOut, p3_loOut);
  register16bit register16bit_MEM_WB_3(clk, reset, regWrite, decOut1b, p2_aluOut, p3_aluOut);
	register16bit register16bit_MEM_WB_4(clk, reset, regWrite, decOut1b, memOut, p3_memOut);

	
	register4bit register4bit_MEM_WB_1(clk, reset, regWrite, decOut1b, p2_mux4bOut, p3_mux4bOut);
	
	register1bit register1bit_p2regWrite(clk, reset, regWrite, decOut1b, p2_regWrite, p3_regWrite);
	register2bit register2bit_p2toReg(clk, reset, regWrite, decOut1b,p2_toReg, p3_toReg);
	
	endmodule

//topModule
module pipeline(input clk, input reset, output [15:0] Result );
	
	//Write your code here
	
	wire[15:0] adder_out, pc_out, IR, p0_intr, writeData, regOut1, regOut2,signExtOffset;
	wire memWrite, memRead, regWrite;
	
	wire ctr_regDst, ctr_memRead, ctr_memWrite, ctr_regWrite, ctr_aluSrcB, ctr_hiWrite, ctr_loWrite;
	wire[1:0] ctr_aluOp, ctr_toReg,  p1_aluOp, p1_toReg, p2_toReg, p3_toReg;
	
	wire[15:0] p1_regOut1, p1_regOut2, p1_sExtOut;
	wire[3:0] p1_rt, p1_rd, mux4bOut, p2_mux4bOut;
	
	wire p1_aluSrcB, p1_hiWrite, p1_loWrite, p1_regDst, p1_memRead, p1_memWrite, p1_regWrite;
	wire p2_memRead, p2_memWrite, p2_regWrite, p3_regWrite;
	
	wire[15:0] aluIn2;
	wire[15:0] p2_hiOut, p2_loOut, p2_aluOut;
	
	
	wire[15:0] memOut;
	  
  wire[15:0] p3_hiOut, p3_loOut, p3_aluOut, p3_memOut;
  wire[3:0] p3_mux4bOut;
  wire[15:0] aluOut2, aluOut1, hiOut, loOut;
  
	wire[15:0] p2_regOut2;
	
	register16bit PC(clk, reset, 1'b1, 1'b1, adder_out, pc_out);
	adder pc_adder(16'b0000000000000010, pc_out, adder_out);
	
	Mem instr_Mem(clk, reset, 1'b0, 1'b1, pc_out, 16'b0 ,IR );
	IF_ID IF_ID_PipelineReg(clk, reset, 1'b1, 1'b1, IR, p0_intr);
	
	
	/*
	registerFile(input clk, input reset, input regWrite, input [3:0] srcRegA, input [3:0] srcRegB, 
		input [3:0] destReg,  input [15:0] writeData, output [15:0] outBusA, output [15:0] outBusB );
		errors
		ID_EX(input clk, input reset,input regWrite, input decOut1b,input [15:0] regOut1,input [15:0] regOut2,
  input [15:0] sExtOut,input [3:0] inst_Rt,input [3:0] inst_Rd,input ctr_aluSrcB,input [1:0] ctr_aluOp, input ctr_hiWrite, input ctr_loWrite, 
  input ctr_regDst, input ctr_memRead, input ctr_memWrite,input ctr_regWrite, input[1:0] ctr_toReg,
  output [15:0] p1_regOut1,output [15:0] p1_regOut2,output [15:0] p1_sExtOut,output [3:0] p1_rt,output [3:0] p1_rd,
  output p1_aluSrcB, output [1:0] p1_aluOp, output p1_hiWrite, output p1_loWrite, output p1_regDst, output p1_memRead,
  output p1_memWrite,output p1_regWrite, output[1:0] p1_toReg);
		
		EX_MEM(input clk, input reset,input regWrite, input decOut1b,input [15:0] hiOut, input [15:0] loOut,
   input [15:0] aluOut,input [15:0] p1_regOut2,input [3:0] mux4bOut,input p1_memRead, input p1_memWrite,input p1_regWrite,
	input[1:0] p1_toReg,output[15:0] p2_hiOut,output [15:0] p2_loOut,output [15:0] p2_aluOut,
	output [15:0] p2_regOut2,output[3:0] p2_mux4bOut,output p2_memRead, output p2_memWrite,
	output p2_regWrite, output[1:0] p2_toReg );
	
	
	MEM_WB(input clk, input reset,input regWrite, input decOut1b,input [15:0] p2_hiOut, input [15:0] p2_loOut,
   input [15:0] p2_aluOut,input [15:0] memOut,input [3:0] p2_mux4bOut,input p2_regWrite,input[1:0] p2_toReg,
	output[15:0] p3_hiOut,output [15:0] p3_loOut,output [15:0] p3_aluOut,
	output [15:0] p3_memOut,output[3:0] p3_mux4bOut,output p3_regWrite, output[1:0] p3_toReg );
		*/
		// signExt5to16( input [4:0] offset, output reg [15:0] signExtOffset);
		registerFile registerFile1(clk, reset, p3_regWrite, IR[12:9], IR[8:5], p3_mux4bOut,  Result, regOut1, regOut2);
		signExt5to16 signExt5to161(IR[4:0], signExtOffset);

		ID_EX ID_EX_pipelineReg(clk, reset, 1'b1, 1'b1, regOut1, regOut2, signExtOffset, IR[8:5] , IR[3:0], ctr_aluSrcB, ctr_aluOp, ctr_hiWrite, ctr_loWrite,  ctr_regDst, ctr_memRead, ctr_memWrite, ctr_regWrite, ctr_toReg,   p1_regOut1, p1_regOut2, p1_sExtOut, p1_rt, p1_rd, p1_aluSrcB,  p1_aluOp, p1_hiWrite, p1_loWrite, p1_regDst, p1_memRead, p1_memWrite, p1_regWrite, p1_toReg);

 	//	ctrlCkt	( 	input [2:0] opcode, output reg aluSrcB, output reg [1:0] aluOp, output reg hiWrite, output reg loWrite, output reg regDst, output reg memRead, output reg memWrite,output reg regWrite, output reg[1:0] toReg);
	
		ctrlCkt	cntrl( IR[15:13], ctr_aluSrcB, ctr_aluOp, ctr_hiWrite, ctr_loWrite, ctr_regDst, ctr_memRead, ctr_memWrite, ctr_regWrite, ctr_toReg);
	
	 //mux2to1_16bits(input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxout);
	 mux2to1_16bits  mux2to1_16bits_aluSRCMUX(p1_regOut2, p1_sExtOut, p1_aluSrcB, aluIn2);
	 mux2to1_4bits mux2to1_4bits1(p1_rt, p1_rd, p1_regDst, mux4bOut);
	 
	 
	 // register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	 register16bit HI(clk, reset, p1_hiWrite, 1'b1, aluOut2, hiOut);
	 register16bit LO(clk, reset, p1_loWrite, 1'b1, aluOut1, loOut);
	 
	  alu alu1(p1_regOut1, aluIn2, p1_aluOp, aluOut1, aluOut2);
	 
	 
	 EX_MEM EX_MEM_pipelineReg(clk, reset, 1'b1, 1'b1, hiOut, loOut, aluOut1, p1_regOut2, mux4bOut, p1_memRead, p1_memWrite, p1_regWrite, p1_toReg, p2_hiOut, p2_loOut, p2_aluOut, p2_regOut2, p2_mux4bOut, p2_memRead, p2_memWrite, p2_regWrite, p2_toReg );
	 
	 
	 //Mem(input clk, input reset,input memWrite,input memRead, input [15:0] pc, input [15:0] dataIn,output [15:0] IR );
  Mem dataMem(clk, reset, p2_memWrite, p2_memRead, p2_aluOut, p2_regOut2, memOut);

    MEM_WB MEM_WB_pipelineREG(clk, reset, 1'b1, 1'b1, p2_hiOut, p2_loOut, p2_aluOut, memOut, p2_mux4bOut, p2_regWrite, p2_toReg, p3_hiOut, p3_loOut, p3_aluOut,	 p3_memOut, p3_mux4bOut, p3_regWrite, p3_toReg);


  //mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3,input [15:0] in4, input [1:0] sel, output reg [15:0] muxout);
	mux4to1_16bits muxfinal(p3_hiOut, p3_loOut, p3_memOut, p3_aluOut, p3_toReg, Result);
	
endmodule

module pipelineTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	pipeline uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10  reset=0;	
		
		#210 $finish; 
	end
endmodule


