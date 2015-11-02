// Instruction Memory Design
module D_ff_IM( input reset, input d, output reg q);
	always@(reset)
	if(reset)
		q=d;
endmodule

module register_IM(input reset, input [15:0] d_in, output [15:0] q_out);
	D_ff_IM dIM0 (reset, d_in[0], q_out[0]);
	D_ff_IM dIM1 (reset, d_in[1], q_out[1]);
	D_ff_IM dIM2 (reset, d_in[2], q_out[2]);
	D_ff_IM dIM3 (reset, d_in[3], q_out[3]);
	D_ff_IM dIM4 (reset, d_in[4], q_out[4]);
	D_ff_IM dIM5 (reset, d_in[5], q_out[5]); 
	D_ff_IM dIM6 (reset, d_in[6], q_out[6]);
	D_ff_IM dIM7 (reset, d_in[7], q_out[7]);
	D_ff_IM dIM8 (reset, d_in[8], q_out[8]);
	D_ff_IM dIM9 (reset, d_in[9], q_out[9]);
	D_ff_IM dIM10 (reset, d_in[10], q_out[10]);
	D_ff_IM dIM11 (reset, d_in[11], q_out[11]);
	D_ff_IM dIM12 (reset, d_in[12], q_out[12]);
	D_ff_IM dIM13 (reset, d_in[13], q_out[13]);
	D_ff_IM dIM14 (reset, d_in[14], q_out[14]);
	D_ff_IM dIM15 (reset, d_in[15], q_out[15]);
endmodule

module IM(  input reset, input [4:0] pc_5bits, output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15;
	register_IM rIM0(reset, 16'b111_0010_0001_00001,Qout0); 		//addi $r1, $r2, 1 
	register_IM rIM1(reset, 16'b000_0001_0001_0_0010, Qout1); 		//add $r2, $r1,$r1
	register_IM rIM2(reset, 16'b111_0010_0011_00001, Qout2); 		//addi $r3,$r2,1
	register_IM rIM3(reset, 16'b010_0011_0010_0_0000, Qout3); 		//mul $r3,$r2
	register_IM rIM4(reset, 16'b101_0000_0000_0_0110, Qout4); 		//mflo r6
	register_IM rIM5(reset, 16'b110_0000_0100_00100, Qout5); 		//ori $r4,$r0,4
	register_IM rIM6(reset, 16'b001_0001_0100_0_0101, Qout6); 		//or $r5,$rl,$r4
	register_IM rIM7(reset, 16'b111_0000_1111_01111, Qout7);		//addi $r15, $r0, 15
	register_IM rIM8(reset, 16'b011_1111_0010_00000, Qout8); 		//div $r15,$r2
	register_IM rIM9(reset, 16'b101_0000_0000_0_0111, Qout9); 		//mflo r7
	register_IM rIM10(reset, 16'b100_0000_0000_0_1000, Qout10); 	//mfhi r8
	register_IM rIM11(reset, 16'b111_1110_1001_11010, Qout11); 	//addi $r9,$r14,-6
	register_IM rIM12(reset, 16'b110_0100_1110_10000, Qout12); //	ori $r15,$r4, -16
	register_IM rIM13(reset, 16'b010_1001_1111_0_0000, Qout13); //mul $r9,$15	
	register_IM rIM14(reset, 16'b100_0000_0000_0_1010, Qout14); 	//mfhi r10
	register_IM rIM15(reset, 16'b101_0000_0000_0_1011, Qout15); 	//mflo r11
	mux16to1 mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc_5bits[4:1],IR);
endmodule
//Instruction Memory Design Ends
// Register File Design
module D_ff (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		if(regWrite == 1 && decOut1b==1) begin q=d; end
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
	registerSet rSet0( clk, reset, regWrite, decOut, writeData, outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15);
	mux16to1 m1(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,srcRegA,outBusA);
	mux16to1 m2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,srcRegB,outBusB);
endmodule
//Register File Design Ends

module ctrlCkt	( 	input [2:0] opcode, output reg regWrite, output reg regDst, output reg [1:0] aluSrc, output reg [1:0] aluOp, output reg hiWrite, output reg loWrite, output reg [1:0] toReg);

//Write your code here

endmodule

module mux2to1_4bits(input [3:0] in1, input [3:0] in2, input sel, output reg [3:0] muxout);

//Write your code here

endmodule

module adder(input [15:0] in1, input [15:0] in2, output reg [15:0] adder_out);

//Write your code here

endmodule

module signExt5to16( input [4:0] offset, output reg [15:0] signExtOffset);

//Write your code here

endmodule

module zeroExt5to16( input [4:0] offset, output reg [15:0] zeroExtOffset);

//Write your code here

endmodule

module mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3, input [1:0] sel, output reg [15:0] muxout);

//Write your code here

endmodule

//For Multiplication lo register stores least significant 16 bits of result and hi register stores most significant 16 bits of result
//For Division lo register stores quotient of result and hi register stores remainder of result
module alu(input [15:0] aluIn1, input [15:0] aluIn2, input [1:0] aluOp, output reg [15:0] aluOut1,output reg [15:0] aluOut2);

//Write your code here

endmodule

module singleCycle(input clk, input reset, output [15:0] Result );

//Write your code here

endmodule

module singleCycleTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	singleCycle uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#5  reset=0;	
		#165 $finish; 
	end
endmodule