module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY,  /* REMOVE */ mult_counter, divd_counter);
    input [31:0] data_operandA, data_operandB;
    input ctrl_MULT, ctrl_DIV, clock;

    output [31:0] data_result;
    output data_exception, data_resultRDY;

    wire[31:0] mult_result, divd_result;
    wire divd_ready, mult_ready, not_clock, either_signal, de_mult, de_divd;

    wire divd_or_mult;

	 // REMOVE
	 output[5:0] mult_counter, divd_counter;
	 
    // Your code here
    mult_module my_mult(data_operandA, data_operandB, clock, ctrl_MULT, mult_result, de_mult, mult_ready, /* REMOVE */ mult_counter);
    divd_module my_divd(data_operandA, data_operandB, clock, ctrl_DIV, divd_result, de_divd, divd_ready,  /* REMOVE */ divd_counter);

    // hold most recent value of mult or division in divd_or_mult
	 not invclock(not_clock, clock);
	 or o1(either_signal, ctrl_DIV, ctrl_MULT);
    dflipflop m_dflipflop(.d(ctrl_DIV), .clk(clock), .clrn(1'b1), .ena(either_signal), .q(divd_or_mult));

    // choose between mult_result and divd_result
    assign data_result = divd_or_mult ? divd_result : mult_result;

    // choose between mult_ready and divd_ready
    assign data_resultRDY = divd_or_mult ? divd_ready : mult_ready;

    // choose between de_mult and de_divd
    assign data_exception = divd_or_mult ? de_divd : de_mult;


endmodule



module divd_module(
	input[31:0] dividend,
	input[31:0] divisor,
	input clock,
	input ctrl_DIVD,

	output[31:0] data_result, 
	output data_exception, 
	output data_resultRDY,
	
	// remove 
	output[5:0] counter
	);

	wire divisor_is_neg, dividend_is_neg, shift_MSB;
	wire quotient_needs_inversion, not_clock;
	wire[31:0] positive_dividend, positive_divisor, quotient_index, wire_to_hold;
	wire[63:0] divisor_64, remainder, rem_hold, sub_val, d_val, not_divisor_64, shifted_divisor_64;

	wire[32:0] quotient;

	wire[5:0] count, notcount;
	
	// REMOVE
	assign counter = count;

	// enstantiate counter
	counter33down divd_count(.clock(clock), .reset(ctrl_DIVD), .out(count));
	genvar i;
   	generate
   	for(i=0; i < 6; i = i + 1) begin: loop1
			not notcounterinputs(notcount[i], count[i]);
   	end
   	endgenerate

	and a1(data_resultRDY, count[0], count[1], count[2], count[3], count[4], count[5]);


	// create inverted clock
	not invclock(not_clock, clock);
	
	assign dividend_is_neg = dividend[31];
	assign divisor_is_neg  = divisor[31];
	
	xor xor1(quotient_needs_inversion, divisor_is_neg, dividend_is_neg);

	neg_or_pos neg_dividend(.data_in(dividend), .data_out(positive_dividend), .take_neg(dividend_is_neg));
	neg_or_pos neg_divisor(.data_in(divisor),   .data_out(positive_divisor),  .take_neg(divisor_is_neg));

	// initial values of divisor are <divisor + 0000..0000>
	assign divisor_64[63:33] = ctrl_DIVD ? positive_divisor[30:0] : shifted_divisor_64[63:33];
	assign divisor_64[32:0]  = ctrl_DIVD ? 33'd0                  : shifted_divisor_64[32:0];
	
	// shift bits 62:0 from divisor_64
	genvar x;
   	generate
   	for(x=0; x < 63; x = x + 1) begin: loopx
			// dflipflop
			dflipflop sh_dflipflop(.d(divisor_64[x+1]), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(shifted_divisor_64[x]));
   	end
   endgenerate
	
	// shift divisor on clock to hold either 0 or positive_divisor
	assign shift_MSB = ctrl_DIVD ? positive_divisor[31] : 1'b0;
	dflipflop sh_MSB_dflipflop(.d(shift_MSB), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(shifted_divisor_64[63])); 
	
	// initial values of remainder are <0000..0000 + dividend>
	assign rem_hold[63:32] = ctrl_DIVD ? 32'd0             : d_val[63:32];
	assign rem_hold[31:0]  = ctrl_DIVD ? positive_dividend : d_val[31:0];
	
	// write into dflipflop with clock signal
	genvar j;
   	generate
   	for(j=0; j < 64; j = j + 1) begin: loop2
			dflipflop m_dflipflop(.d(rem_hold[j]), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(remainder[j]));
   	end
   	endgenerate
	// end dflipflop write
	
	// 64-bit adder (needs to be fast for division)
	// inputs (a = remainder, b = NOT(divisor_64), cin = 1)
	genvar f;
   	generate
   	for(f=0; f < 64; f = f + 1) begin: loopf
			not nd6(not_divisor_64[f], shifted_divisor_64[f]);
   	end
   	endgenerate

   wire[4:0] add_or_sub;
   wire lower_not_equal, upper_not_equal, lower_greater_than, upper_greater_than;
   
	// set 4 MSBs of this wire to 0, the last will be 0/1 depending on cout of lower_alu. This tells you if you should add or sub next
   //assign add_or_sub[4:1] = 4'b0000;
	//alu lower_alu(.data_operandA(remainder[31:0]), .data_operandB(not_divisor_64[31:0]), .ctrl_ALUopcode(5'b00001), .ctrl_shiftamt(5'b00000), .data_result(sub_val[31:0]), .cout(add_or_sub[0]));
	//alu upper_alu(.data_operandA(remainder[63:32]), .data_operandB(not_divisor_64[63:32]), .ctrl_ALUopcode(add_or_sub), .ctrl_shiftamt(5'b00000), .data_result(sub_val[63:32]));

	adder64 a32l(remainder, not_divisor_64, sub_val, 1'b1);
	
	// sub_val is the value of subtraction. 64th bit means divisor was greater than remainder
	wire greater_than, not_greater_than;
	
	assign greater_than = sub_val[63];
	not ngt(not_greater_than, greater_than);
	assign d_val = not_greater_than ? sub_val : remainder;


	// write into quotient with !greater_than using a decoder and tri-state signals
	decoder q_decode(.in(count[4:0]), .out(quotient_index));
	genvar k;
   	generate
   	for(k=0; k < 32; k = k + 1) begin: loop3
			assign wire_to_hold[k] = quotient_index[k] ? not_greater_than : quotient[k];
			dflipflop m_dflipflop(.d(wire_to_hold[k]), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(quotient[k]));
   	end
   	endgenerate

	// flip data_result_hold if needed using neg_or_pos
	neg_or_pos nop1(.data_in(quotient[31:0]), .data_out(data_result), .take_neg(quotient_needs_inversion));

	wire[31:0] not_divisor;
	// data_exception == and(not(divisor 31-->0))
	genvar m;
   	generate
   	for(m=0; m < 32; m = m + 1) begin: loop4
			not n1123(not_divisor[m], divisor[m]);
   	end
   	endgenerate
	and check_0s(data_exception, not_divisor[31], not_divisor[30], not_divisor[29], not_divisor[28], not_divisor[27], not_divisor[26], not_divisor[25], not_divisor[24], not_divisor[23], not_divisor[22], not_divisor[21], not_divisor[20], not_divisor[19], not_divisor[18], not_divisor[17], not_divisor[16], not_divisor[15], not_divisor[14], not_divisor[13], not_divisor[12], not_divisor[11], not_divisor[10], not_divisor[9], not_divisor[8], not_divisor[7], not_divisor[6], not_divisor[5], not_divisor[4], not_divisor[3], not_divisor[2], not_divisor[1], not_divisor[0]);



endmodule



/*
Chooses between negative and positive values for data 
*/
module neg_or_pos(data_in, data_out, take_neg);
	input[31:0] data_in;
	input take_neg;
	output[31:0] data_out;

	wire[31:0] not_in;
	wire[31:0] negated_val;

	// invert data_in in preparation for subtraction
	genvar i;
   	generate
   	for(i=0; i < 32; i = i + 1) begin: loop1
			not not1(not_in[i], data_in[i]);
   	end
   	endgenerate
	// end data_in not

	// negate data_in --> negated_val
	adder32 negate(.data_a(not_in), .data_b(32'd0), .sum(negated_val), .sub(1'b1));

	// choose neg or pos val
	assign data_out = take_neg ? negated_val : data_in;

endmodule


/*
counter33_down

Provided by ECE350 teaching staff.
Modified from counter8 to count to 16 rather than 8.
Clock cycle forces out to next value therefore counts up with clock. 
*/
module counter33down(clock, reset, out);
     input clock, reset;
     output [5:0] out;
     reg [5:0] out;

     always@(posedge clock) begin
			 if(reset) begin
				out = 32;
			 end
          casex({reset, out})
               6'd32: out = 31;
               6'd31: out = 30;
               6'd30: out = 29;
               6'd29: out = 28;
               6'd28: out = 27;
               6'd27: out = 26;
               6'd26: out = 25;
               6'd25: out = 24;
               6'd24: out = 23;
               6'd23: out = 22;
               6'd22: out = 21;
               6'd21: out = 20;
               6'd20: out = 19;
               6'd19: out = 18;
               6'd18: out = 17;
               6'd17: out = 16;
               6'd16: out = 15;
               6'd15: out = 14;
               6'd14: out = 13;
               6'd13: out = 12;
               6'd12: out = 11;
               6'd11: out = 10;
               6'd10: out = 9;
               6'd9: out = 8;
               6'd8: out = 7;
               6'd7: out = 6;
               6'd6: out = 5;
               6'd5: out = 4;
               6'd4: out = 3;
               6'd3: out = 2;
               6'd2: out = 1;
               6'd1: out = 0;
               6'd0: out = 63;
					6'd63: out = 62;
					6'd62: out = 62;
               default: out = 32;
          endcase
     end
endmodule
















/*
Multiplier 

Inputs
- data_operandA = Multiplicand
- data_operandB = Multiplier
- ctrl_MULT = signal to start operation
- clock = doing its clock thing

Outputs
- data_result = product
- data_exception = flag if mult overflow
- data_resultRDY = high when completed computation
*/

module mult_module (
	input[31:0] multiplicand,
	input[31:0] multiplier,
	input clock,
	input ctrl_MULT,

	output[31:0] data_result, 
	output data_exception, 
	output data_resultRDY,
	
	// REMOVE
	output[5:0] counter
);

	
	
	wire[31:0] not_multiplicand, sub_val, add_val, add_sub_val, d_val, d_val_tri;
	wire[5:0] count;
	wire[64:0] product, shifted_product;
	wire not_all_same;

	wire sub_line, hold_do_nothing, do_nothing, b, a, not_data_resultRDY, clock_and, not_clock_and;

	// write multiplier into lower bits of product -- assumes shifted_product will move everything over 1 before we read that value into rest of system
	assign product[64:34] = ctrl_MULT ? 31'd0            : d_val[31:1];
	assign product[33]    = ctrl_MULT ? multiplier[31]   : d_val[0];
	assign product[32:2]  = ctrl_MULT ? multiplier[30:0] : shifted_product[32:2];
	assign product[1:0]   = ctrl_MULT ? 2'd0 				  : shifted_product[1:0];

	// shift product
	genvar x;
   	generate
   	for(x=0; x < 64; x = x + 1) begin: loopx
			// dflipflop
			dflipflop sh_dflipflop(.d(product[x+1]), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(shifted_product[x]));
   	end
   endgenerate
	// top bit of product held as top bit of shifted (logical shift)
	dflipflop sh_MSB_dflipflop(.d(product[64]), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(shifted_product[64])); 
	
	
	// write 2 LSBs into b, a
	assign b = shifted_product[1];
	assign a = shifted_product[0];

	/// Control Logic
	// Enstantiate counter, held on reg 'count', reset on ctrl_MULT
	counter32 mult_count(.clock(clock), .reset(ctrl_MULT), .out(count));

	// REMOVE
	assign counter = count;
	
	// do_nothing = not(xor(a,b))
	xor x1(hold_do_nothing, a, b);
	not n1(do_nothing, hold_do_nothing);

	// sub_line = b
	assign sub_line = b;

	// data_resultRDY = mult_count hits 31, which means all count[4:0] is 1
	and and_ready(data_resultRDY, count [5], count[4], count[3], count[2], count[1], count[0]);
	/// Control Logic

	// invert multiplicand in preparation for subtraction
	genvar i;
   	generate
   	for(i=0; i < 32; i = i + 1) begin: loop1
			not not1(not_multiplicand[i], multiplicand[i]);
   	end
   	endgenerate
	// end multiplicand not
	
	// sub and add functions
	// use 33-64 because product is 65 long
	adder32 m_sub(.data_a(shifted_product[64:33]), .data_b(not_multiplicand), .sum(sub_val), .sub(1'b1));
	adder32 m_add(.data_a(shifted_product[64:33]), .data_b(multiplicand), 	  .sum(add_val), .sub(1'b0));

	// tri-state between add and sub based on sub_line
	assign add_sub_val = sub_line ? sub_val : add_val;
	// tri-state between add_sub_val and 00000...000
	assign d_val = do_nothing ? shifted_product[64:33] : add_sub_val;

	// prepare clock signal for dflipflop
	not not2(not_data_resultRDY, data_resultRDY);
	and and1(clock_and, not_data_resultRDY, clock);
	not not3(not_clock_and, clock_and);


	// assign final result 
	assign data_result = shifted_product[32:1];

	/// data_exception conditions
	wire top32, three_xor, three_xor_hold, all_1, all_0, not_all_1, not_all_0;
	
	// if top 32 bits are not all 1's or all 0's
	//nand check_1s(not_all_1, shifted_product[64], shifted_product[63], shifted_product[62], shifted_product[61], shifted_product[60], shifted_product[59], shifted_product[58], shifted_product[57], shifted_product[56], shifted_product[55], shifted_product[54], shifted_product[53], shifted_product[52], shifted_product[51], shifted_product[50], shifted_product[49], shifted_product[48], shifted_product[47], shifted_product[46], shifted_product[45], shifted_product[44], shifted_product[43], shifted_product[42], shifted_product[41], shifted_product[40], shifted_product[39], shifted_product[38], shifted_product[37], shifted_product[36], shifted_product[35], shifted_product[34], shifted_product[33]);
	//or  check_0s(not_all_0, shifted_product[64], shifted_product[63], shifted_product[62], shifted_product[61], shifted_product[60], shifted_product[59], shifted_product[58], shifted_product[57], shifted_product[56], shifted_product[55], shifted_product[54], shifted_product[53], shifted_product[52], shifted_product[51], shifted_product[50], shifted_product[49], shifted_product[48], shifted_product[47], shifted_product[46], shifted_product[45], shifted_product[44], shifted_product[43], shifted_product[42], shifted_product[41], shifted_product[40], shifted_product[39], shifted_product[38], shifted_product[37], shifted_product[36], shifted_product[35], shifted_product[34], shifted_product[33]);
	assign not_all_1 = ~& shifted_product[64:33];
	assign not_all_0 =  | shifted_product[64:33];
	
	and a934(not_all_same, not_all_1, not_all_0);

	// three-input xor between MSB of multiplicand, multiplier, and MSB of data_result
	xor(three_xor_hold, data_result[31], multiplier[31]);
	xor(three_xor, three_xor_hold, multiplicand[31]);
	
	wire all_0_cand, all_0_lier, all_0_either, three_xor_finished;
	assign all_0_cand = ~| multiplicand;
	assign all_0_lier = ~| multiplier;
	
	nor nor2(all_0_either, all_0_cand, all_0_lier);
	and a151(three_xor_finished, three_xor, all_0_either);
	
	// data_exception final
	or or_DE(data_exception, three_xor_finished, not_all_same);

endmodule // mult_module



/*
counter32

Provided by ECE350 teaching staff.
Modified from counter8 to count to 16 rather than 8.
Clock cycle forces out to next value therefore counts up with clock. 
*/
module counter32(clock, reset, out);
     input clock, reset;
     output [5:0] out;
     reg [5:0] out;

     always@(posedge clock) begin
			 if(reset) begin
				out = 0;
			 end
          casex({reset, out})
               6'd0: out = 1;
               6'd1: out = 2;
               6'd2: out = 3;
               6'd3: out = 4;
               6'd4: out = 5;
               6'd5: out = 6;
               6'd6: out = 7;
               6'd7: out = 8;
               6'd8: out = 9;
               6'd9: out = 10;
               6'd10: out = 11;
               6'd11: out = 12;
               6'd12: out = 13;
               6'd13: out = 14;
               6'd14: out = 15;
               6'd15: out = 16;
               6'd16: out = 17;
               6'd17: out = 18;
               6'd18: out = 19;
               6'd19: out = 20;
               6'd20: out = 21;
               6'd21: out = 22;
               6'd22: out = 23;
               6'd23: out = 24;
               6'd24: out = 25;
               6'd25: out = 26;
               6'd26: out = 27;
               6'd27: out = 28;
               6'd28: out = 29;
               6'd29: out = 30;
               6'd30: out = 31;
               6'd31: out = 63;
					6'd63: out = 62;
					// sit in 62 during idle
					6'd62: out = 62;
               default: out = 0;
          endcase
     end
endmodule


/*
32-bit Adder/Subtracter

RCA - simple to implement and fast enough
*/
module adder64(data_a, data_b, sum, sub);
	input[63:0] data_a, data_b;
	input sub;
	output[63:0] sum;

	wire[64:0] cins;

	// first cin will be either 0 (add) or 1 (sub)
	assign cins[0] = sub;

	// lay down 32 adders RCA style
	genvar i;
   	generate
   	for(i=0; i < 64; i = i + 1) begin: outer
			full_adder fa(.a(data_a[i]), .b(data_b[i]), .cin(cins[i]), .s(sum[i]), .cout(cins[i+1]));
   	end
   	endgenerate

endmodule // adder64


/*
32-bit Adder/Subtracter

RCA - simple to implement and fast enough
*/
module adder32(data_a, data_b, sum, sub);
	input[31:0] data_a, data_b;
	input sub;
	output[31:0] sum;

	wire[32:0] cins;

	// first cin will be either 0 (add) or 1 (sub)
	assign cins[0] = sub;

	// lay down 32 adders RCA style
	genvar i;
   	generate
   	for(i=0; i < 32; i = i + 1) begin: outer
			full_adder fa(.a(data_a[i]), .b(data_b[i]), .cin(cins[i]), .s(sum[i]), .cout(cins[i+1]));
   	end
   	endgenerate

endmodule // adder32


/*
	dflipflop implementation, created by ECE350 TA Staff
*/
module dflipflop(d, clk, clrn, prn, ena, q);
    input d, clk, ena, clrn, prn;
    wire clr;
    wire pr;

    output q;
    reg q;

    assign clr = ~clrn;
    assign pr = ~prn;

    initial
    begin
        q = 1'b0;
    end

    always @(posedge clk or posedge clr) begin
        if (q == 1'bx) begin
            q = 1'b0;
        end else if (clr) begin
            q <= 1'b0;
        end else if (ena) begin
            q <= d;
        end
    end
endmodule






