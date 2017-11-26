module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);

   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;

   output [31:0] data_result;
   output isNotEqual, isLessThan, overflow;

   // YOUR CODE HERE //

   wire[31:0] new_data_operandB; // new_data_operandB will hold the ideal signal of B, either inverted or not
   wire[5:0] decoded_ALUopcode;
   wire g0, g1, g2, g3, p0, p1, p2, p3;
   wire c8, c16, c24, c32, c31;

   // TODO Decode the ctrl_ALUopcode signal (4 --> 6 values)
   decoder4_6 op_codes(.in(ctrl_ALUopcode), .out(decoded_ALUopcode));

   // The first c_in will be decoded_ALUopcode[1], which is the sub signal
   // set new_data_operandB to either data_operandB (if not subbing) or !data_operandB (if subbing)
   genvar i;
   generate
   	for(i = 0; i<32; i = i + 1) begin: loop1
   		// y_new = xor(sub_signal, y)
   		xor xor1(new_data_operandB[i], data_operandB[i], decoded_ALUopcode[1]);
   	end
   endgenerate

   // set up the alu8 modules with proper inputs based on slide 5.9
   alu8 block0(.x(data_operandA[7:0]), 		.y(new_data_operandB[7:0]), 	.cin(decoded_ALUopcode[1]), 	.op_code(decoded_ALUopcode), 	.g(g0), 	.p(p0), 	.final_out(data_result[7:0]));
   subblock0 s0(g0, p0, decoded_ALUopcode[1], c8);

   alu8 block1(.x(data_operandA[15:8]), 	.y(new_data_operandB[15:8]), 	.cin(c8), 	.op_code(decoded_ALUopcode), 	.g(g1), 	.p(p1), 	.final_out(data_result[15:8]));
   subblock1 s1(g0, g1, p0, p1, decoded_ALUopcode[1], c16);

   alu8 block2(.x(data_operandA[23:16]), 	.y(new_data_operandB[23:16]), 	.cin(c16), 	.op_code(decoded_ALUopcode), 	.g(g2), 	.p(p2), 	.final_out(data_result[23:16]));
   subblock2 s2(g0, g1, g2, p0, p1, p2, decoded_ALUopcode[1], c24);

   alu8 block3(.x(data_operandA[31:24]), 	.y(new_data_operandB[31:24]), 	.cin(c24), 	.op_code(decoded_ALUopcode), 	.g(g3), 	.p(p3), 	.final_out(data_result[31:24]), .cout_penultimate(c31));
   subblock3 s3(g0, g1, g2, g3, p0, p1, p2, p3, decoded_ALUopcode[1], c32);

   xor xorOVF(overflow, c32, c31); // potential BUG - overflow might be xor(c31,c32)
   // isNotEqual is true if data_result is not 0
   or not_equal(isNotEqual, data_result[0], data_result[1], data_result[2], data_result[3], data_result[4], data_result[5], data_result[6], data_result[7], data_result[8], data_result[9], data_result[10], data_result[11], data_result[12], data_result[13], data_result[14], data_result[15], data_result[16], data_result[17], data_result[18], data_result[19], data_result[20], data_result[21], data_result[22], data_result[23], data_result[24], data_result[25], data_result[26], data_result[27], data_result[28], data_result[29], data_result[30], data_result[31]);
   // isLessThan true if sub value is negative, which means data_result[31] is negative
   or orLT(isLessThan, data_result[31], overflow);
	

   wire[31:0] leftShift, rightShift;
   // shift A by specified amount
   left_shifter  lsll(.in(data_operandA), .out(leftShift),  .ctrl_shiftamt(ctrl_shiftamt));
   right_shifter rsra(.in(data_operandA), .out(rightShift), .ctrl_shiftamt(ctrl_shiftamt));

   // tri-state to output wire
   assign data_result = decoded_ALUopcode[4] ? leftShift  : 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;
   assign data_result = decoded_ALUopcode[5] ? rightShift : 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;


endmodule

// from slide 5.9
module subblock0(g0, p0, c0, c8);
	input g0, p0, c0;
	output c8;

	wire w1;

	and a1(w1, p0, c0);
	or  o1(c8, g0, w1);

endmodule

// from slide 5.9
module subblock1(g0, g1, p0, p1, c0, c16);
	input g0, g1, p0, p1, c0;
	output c16;

	wire w1, w2;

	and a1(w1, p1, g0);
	and a2(w2, p1, p0, c0);
	or  o1(c16, g1, w1, w2);

endmodule

// from slide 5.9
module subblock2(g0, g1, g2, p0, p1, p2, c0, c24);
	input g0, g1, g2, p0, p1, p2, c0;
	output c24;

	wire w1, w2, w3;

	and a1(w1, p2, g1);
	and a2(w2, p2, p1, g0);
	and a3(w3, p2, p1, p0, c0);
	or  o1(c24, g2, w1, w2, w3);

endmodule

// from slide 5.9
module subblock3(g0, g1, g2, g3, p0, p1, p2, p3, c0, c32);
	input g0, g1, g2, g3, p0, p1, p2, p3, c0;
	output c32;

	wire w1, w2, w3, w4;

	and a1(w1, p3, g2);
	and a2(w2, p3, p2, g1);
	and a3(w3, p3, p2, p1, g0);
	and a4(w4, p3, p2, p1, p0, c0);
	or  o1(c32, g3, w1, w2, w3, w4);

endmodule






/*
	Decodes a 4-bit value into 16 bits, but only returns the lowest 6 bits.
*/
module decoder4_6(in, out);
	input[4:0] in;
	output[5:0] out;

	wire[4:0] no;

	not n0(no[0], in[0]);
	not n1(no[1], in[1]);
	not n2(no[2], in[2]);
	not n3(no[3], in[3]);
	not n4(no[4], in[4]);

	and a0(out[0], no[4], no[3], no[2], no[1], no[0]);
	and a1(out[1], no[4], no[3], no[2], no[1], in[0]);
	and a2(out[2], no[4], no[3], no[2], in[1], no[0]);
	and a3(out[3], no[4], no[3], no[2], in[1], in[0]);
	and a4(out[4], no[4], no[3], in[2], no[1], no[0]);
	and a5(out[5], no[4], no[3], in[2], no[1], in[0]);

endmodule





/*
	8-bit adder, subtracter, and, or block 

	Inverting for subtraction is handled before calling this module

	final_out[8] - sum, and, or based on op_code. Tri-Stated signal.
	g[1] - generate function
	p[1] - propagate function
*/
module alu8(x, y, cin, op_code, g, p, final_out, cout_penultimate);
	input[7:0] x, y;
	input cin;
	input[5:0] op_code;

	output g, p, cout_penultimate;
	output[7:0] final_out;

	wire[7:0] my_or, my_and, my_not, sum;
	wire[8:0] c_outs;

	assign c_outs[0] = cin;
	

	// compute my_or, my_and, my_not
	genvar i;
	generate
		for(i=0; i < 8; i = i+1) begin: loop1
			// p# is equal to x# + y#
			or or1(my_or[i], x[i], y[i]);
			// g# is equal to x# & y#
			and and1(my_and[i], x[i], y[i]);
			// my_not# = not(y#)
			not not1(my_not[i], y[i]);
		end
	endgenerate

	// propagate (p) is and(all my_or signals)
	and and2(p, my_or[0], my_or[1], my_or[2], my_or[3], my_or[4], my_or[5], my_or[6], my_or[7]);

	// generate  (g) 
	generate_signal big_g(.p(my_or), .g(my_and), .out(g));

	// compute actual sum
	genvar j;
	generate
		for(j=0; j < 8; j = j+1) begin: loop2
			// instantiate full adder
			full_adder fa(.a(x[j]), .b(y[j]), .cin(c_outs[j]), .cout(c_outs[j+1]), .s(sum[j]));
			// TODO make this able to do subtraction
		end
	endgenerate


	//  put all of these signals (sum, my_and, my_or) tri-state selector
	//  create tri-state buffer for all of these signal
	tri_selector my_sel(.out(final_out), .op_code(op_code), .sum(sum), .my_and(my_and), .my_or(my_or));

	// outputs the penultimate cout for overflow calculations
	assign cout_penultimate = c_outs[7];
	
endmodule






// tri-state buffer to drive final_out using opcodes for add, sub, my_and, my_or
module tri_selector(out, op_code, sum, my_and, my_or);
	input[7:0] sum, my_or, my_and;
	input[5:0] op_code;
	output[7:0] out;

	// op_code designates which signal is driven to bus. 
	assign out = op_code[0] ? sum    : 8'bzzzzzzzz; // Potential BUG
	assign out = op_code[1] ? sum    : 8'bzzzzzzzz;
	assign out = op_code[2] ? my_and : 8'bzzzzzzzz;
	assign out = op_code[3] ? my_or  : 8'bzzzzzzzz;

endmodule






// generate signal in a adder8 module
module generate_signal(p, g, out);
	input[7:0] p, g;
	output out;

	wire[7:0] t;
	// starts with g7
	and a0(t[0], g[7], 1'b1); 
	and a1(t[1], g[6], p[7]); 
	and a2(t[2], g[5], p[7], p[6]); 
	and a3(t[3], g[4], p[7], p[6], p[5]); 
	and a4(t[4], g[3], p[7], p[6], p[5], p[4]); 
	and a5(t[5], g[2], p[7], p[6], p[5], p[4], p[3]); 
	and a6(t[6], g[1], p[7], p[6], p[5], p[4], p[3], p[2]); 
	and a7(t[7], g[0], p[7], p[6], p[5], p[4], p[3], p[2], p[1]); 
	// ends with p1-7g0

	// G = sum of all temporary (t) values
	or final_or(out, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);

endmodule 






/*
	1-bit Full Adder module.
*/
module full_adder(a, b, cin, cout, s);
	input a, b, cin;
	output cout, s;

	wire s1, c1, c2;

	half_adder a1(.a(a), .b(b), .cout(c1), .s(s1));
	half_adder a2(.a(s1), .b(cin), .cout(c2), .s(s));

	// or or1(cout, c1, c2);
	or or1(cout, c1, c2);

endmodule

module half_adder(a, b, cout, s);
	input a, b;
	output cout, s;

	// cout == a & b
	and and1(cout, a, b);
	// s == xor(a, b)
	xor xor1(s, a, b);

endmodule









// BARELL SHIFTER COMPONENTS!!!



/*
	left_shifter
	Shifts inputs to the left specified by ctrl_shiftamt[5].
*/
module left_shifter(in, out, ctrl_shiftamt);
	input[31:0] in;
	input[4:0] ctrl_shiftamt;
	output[31:0] out;

	// s# wires are the outputs of the sll shifters
	wire[31:0] s16, s8, s4, s2, s1;
	// s# wires are the outputs of the muxes after the shifters
	wire[31:0] m16, m8, m4, m2; 

	sll_16 shift16(.in(in), .out(s16));
	assign m16 = ctrl_shiftamt[4] ? s16 : in;

	sll_8  shift8(.in(m16), .out(s8));
	assign m8  = ctrl_shiftamt[3] ? s8  : m16;

	sll_4  shift4(.in(m8),  .out(s4));	
	assign m4  = ctrl_shiftamt[2] ? s4  : m8;
	
	sll_2  shift2(.in(m4),  .out(s2));	
	assign m2  = ctrl_shiftamt[1] ? s2  : m4;

	sll_1  shift1(.in(m2),  .out(s1));	
	assign out  = ctrl_shiftamt[0] ? s1  : m2;

endmodule





// sll modules (1, 2, 4, 8, 16 bit shifters)
module sll_1(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 1; i < 32; i = i+1) begin: loop1
			// shift bits by amount
			assign out[i] = in[i-1];
		end
	endgenerate

	// pad LSB with proper number of 0's
	genvar j;
	generate
		for (j = 0; j < 1; j=j+1) begin: loop2
			assign out[j] = 1'b0;
		end
	endgenerate

endmodule

module sll_2(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 2; i < 32; i=i+1) begin: loop1
			// shift bits by 2
			assign out[i] = in[i-2];
		end
	endgenerate

	// pad LSB with proper number of 0's
	genvar j;
	generate
		for (j = 0; j < 2; j = j+1) begin: loop2
			assign out[j] = 1'b0;
		end
	endgenerate

endmodule

module sll_4(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 4; i < 32; i = i+1) begin: loop1
			// shift bits by 4
			assign out[i] = in[i-4];
		end
	endgenerate

	// pad LSB with proper number of 0's
	genvar j;
	generate
		for (j = 0; j < 4; j = j+1) begin: loop2
			assign out[j] = 1'b0;
		end
	endgenerate

endmodule

module sll_8(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 8; i < 32; i = i+1) begin: loop1
			// shift bits by 8
			assign out[i] = in[i-8];
		end
	endgenerate

	// pad LSB with proper number of 0's
	genvar j;
	generate
		for (j = 0; j < 8; j = j+1) begin: loop2
			assign out[j] = 1'b0;
		end
	endgenerate

endmodule

module sll_16(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 16; i < 32; i = i+1) begin: loop1
			// shift bits by 16
			assign out[i] = in[i-16];
		end
	endgenerate

	// pad LSB with proper number of 0's
	genvar j;
	generate
		for (j = 0; j < 16; j = j+1) begin: loop2
			assign out[j] = 1'b0;
		end
	endgenerate

endmodule








/*
	right_shifter
	Shifts inputs to the left specified by ctrl_shiftamt[5].
*/
module right_shifter(in, out, ctrl_shiftamt);
	input[31:0] in;
	input[4:0] ctrl_shiftamt;
	output[31:0] out;

	// s# wires are the outputs of the sll shifters
	wire[31:0] s16, s8, s4, s2, s1;
	// s# wires are the outputs of the muxes after the shifters
	wire[31:0] m16, m8, m4, m2; 

	sra_16 shift16(.in(in), .out(s16));
	assign m16 = ctrl_shiftamt[4] ? s16 : in;

	sra_8  shift8(.in(m16), .out(s8));
	assign m8  = ctrl_shiftamt[3] ? s8  : m16;

	sra_4  shift4(.in(m8),  .out(s4));	
	assign m4  = ctrl_shiftamt[2] ? s4  : m8;
	
	sra_2  shift2(.in(m4),  .out(s2));	
	assign m2  = ctrl_shiftamt[1] ? s2  : m4;

	sra_1  shift1(.in(m2),  .out(s1));	
	assign out  = ctrl_shiftamt[0] ? s1  : m2;

endmodule



// sra modules (1, 2, 4, 8, 16 bit shifters)
module sra_1(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 0; i < 32 - 1; i = i+1) begin: loop1
			// shift bits by 1
			assign out[i] = in[i + 1];
		end
	endgenerate

	// pad MSB with proper number of MSB values
	genvar j;
	generate
		for (j = 32-1; j < 32; j = j+1) begin: loop2
			assign out[j] = in[31];
		end
	endgenerate

endmodule

module sra_2(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 0; i < 32 - 2; i = i+1) begin: loop1
			// shift bits by 2
			assign out[i] = in[i + 2];
		end
	endgenerate

	// pad MSB with proper number of MSB values
	genvar j;
	generate
		for (j = 32-2; j < 32; j = j+1) begin: loop2
			assign out[j] = in[31];
		end
	endgenerate

endmodule

module sra_4(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 0; i < 32 - 4; i = i+1) begin: loop1
			// shift bits by 4
			assign out[i] = in[i + 4];
		end
	endgenerate

	// pad MSB with proper number of MSB values
	genvar j;
	generate
		for (j = 32-4; j < 32; j = j+1) begin: loop2
			assign out[j] = in[31];
		end
	endgenerate

endmodule

module sra_8(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 0; i < 32 - 8; i = i+1) begin: loop1
			// shift bits by 8
			assign out[i] = in[i + 8];
		end
	endgenerate

	// pad MSB with proper number of MSB values
	genvar j;
	generate
		for (j = 32-8; j < 32; j = j+1) begin: loop2
			assign out[j] = in[31];
		end
	endgenerate

endmodule

module sra_16(in, out);
	input[31:0] in;
	output[31:0] out;

	// shift values
	genvar i;
	generate 
		for(i = 0; i < 32 - 16; i = i+1) begin: loop1
			// shift bits by 16
			assign out[i] = in[i + 16];
		end
	endgenerate

	// pad MSB with proper number of MSB values
	genvar j;
	generate
		for (j = 32-16; j < 32; j = j+1) begin: loop2
			assign out[j] = in[31];
		end
	endgenerate

endmodule

