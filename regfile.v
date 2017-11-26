module regfile (
    clock,
    ctrl_writeEnable,
    ctrl_reset, ctrl_writeReg,
    ctrl_readRegA, ctrl_readRegB, data_writeReg,
    data_readRegA, data_readRegB
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;

   output [31:0] data_readRegA, data_readRegB;

   /* YOUR CODE HERE */
   
   // enstantiate wires and register
   reg pr = 0; 				// pr is 0 to not preset values
   wire [31:0] decoded_in, in_enable, out_enableA, out_enableB; 	// 32-bit wires that contain the decoded values 


   // Drive the encoder outputs to the decoded_in, out_enableA, and out_enableB wires.
   decoder decode_ctrl_writeReg(.in(ctrl_writeReg), .out(decoded_in));
   decoder decode_ctrl_readRegA(.in(ctrl_readRegA), .out(out_enableA));
   decoder decode_ctrl_readRegB(.in(ctrl_readRegB), .out(out_enableB));

   // drive to in_enable by ANDing decoded_in[] with ctrl_writeEnable
   genvar i;
   generate
   for(i=0; i < 32; i = i + 1) begin: outer
   	// create and gate for each bit of decoded_in with ctrl_writeEnable
		and my_and (in_enable[i], decoded_in[i], ctrl_writeEnable);
   end
   endgenerate



   // Create all 32 registers
   single_register reg0(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[0]), .out_enableA(out_enableA[0]), .out_enableB(out_enableB[0]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg1(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[1]), .out_enableA(out_enableA[1]), .out_enableB(out_enableB[1]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg2(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[2]), .out_enableA(out_enableA[2]), .out_enableB(out_enableB[2]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg3(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[3]), .out_enableA(out_enableA[3]), .out_enableB(out_enableB[3]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg4(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[4]), .out_enableA(out_enableA[4]), .out_enableB(out_enableB[4]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg5(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[5]), .out_enableA(out_enableA[5]), .out_enableB(out_enableB[5]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg6(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[6]), .out_enableA(out_enableA[6]), .out_enableB(out_enableB[6]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg7(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[7]), .out_enableA(out_enableA[7]), .out_enableB(out_enableB[7]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg8(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[8]), .out_enableA(out_enableA[8]), .out_enableB(out_enableB[8]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg9(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[9]), .out_enableA(out_enableA[9]), .out_enableB(out_enableB[9]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg10(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[10]), .out_enableA(out_enableA[10]), .out_enableB(out_enableB[10]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg11(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[11]), .out_enableA(out_enableA[11]), .out_enableB(out_enableB[11]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg12(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[12]), .out_enableA(out_enableA[12]), .out_enableB(out_enableB[12]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg13(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[13]), .out_enableA(out_enableA[13]), .out_enableB(out_enableB[13]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg14(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[14]), .out_enableA(out_enableA[14]), .out_enableB(out_enableB[14]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg15(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[15]), .out_enableA(out_enableA[15]), .out_enableB(out_enableB[15]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg16(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[16]), .out_enableA(out_enableA[16]), .out_enableB(out_enableB[16]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg17(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[17]), .out_enableA(out_enableA[17]), .out_enableB(out_enableB[17]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg18(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[18]), .out_enableA(out_enableA[18]), .out_enableB(out_enableB[18]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg19(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[19]), .out_enableA(out_enableA[19]), .out_enableB(out_enableB[19]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg20(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[20]), .out_enableA(out_enableA[20]), .out_enableB(out_enableB[20]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg21(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[21]), .out_enableA(out_enableA[21]), .out_enableB(out_enableB[21]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg22(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[22]), .out_enableA(out_enableA[22]), .out_enableB(out_enableB[22]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg23(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[23]), .out_enableA(out_enableA[23]), .out_enableB(out_enableB[23]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg24(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[24]), .out_enableA(out_enableA[24]), .out_enableB(out_enableB[24]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg25(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[25]), .out_enableA(out_enableA[25]), .out_enableB(out_enableB[25]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg26(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[26]), .out_enableA(out_enableA[26]), .out_enableB(out_enableB[26]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg27(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[27]), .out_enableA(out_enableA[27]), .out_enableB(out_enableB[27]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg28(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[28]), .out_enableA(out_enableA[28]), .out_enableB(out_enableB[28]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg29(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[29]), .out_enableA(out_enableA[29]), .out_enableB(out_enableB[29]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg30(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[30]), .out_enableA(out_enableA[30]), .out_enableB(out_enableB[30]), .outA(data_readRegA), .outB(data_readRegB));
   single_register reg31(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .pr(pr), .in_enable(in_enable[31]), .out_enableA(out_enableA[31]), .out_enableB(out_enableB[31]), .outA(data_readRegA), .outB(data_readRegB));



endmodule




/* 
	DFF_tri

	creates a dff with output going to a tri-buffer
	d[1]: value to be written
	clk[1]: clock
	clr[1]: reset signal (1 = reset, since logic is inverted)
	pr[1]: preset value (1 = preset, since logic is inverted. Set to 0)
	in_enable[1]: bit allowing dff to be written
	out_enableA[1]: bit allowing tri-state buffer to write to bus A
	out_enableB[1]: bit allowing tri-state buffer to write to bus B
	outA[1]: output for ctrl_readRegA, which is either value q (if out_enable high) or Z (out_enable low)
	outB[1]: output for ctrl_readRegB, which is either value q (if out_enable high) or Z (out_enable low)
 */
module edwin_dff_tri (d, clk, clr, pr, in_enable, out_enableA, out_enableB, outA, outB);
	input d, clk, clr, pr, in_enable, out_enableA, out_enableB;
	output outA, outB;
	wire q;
	wire clrn, prn;
	
	not notclr(clrn, clr);
	not notpr(prn, pr);
	
	d_flip_flop_reg a_dff(.d(d), .clk(clk), .clrn(clrn), .prn(prn), .ena(in_enable), .q(q));

	// assign outA and outB based on out_enableA and out_enableB
	assign outA = out_enableA ? q : 1'bz;
	assign outB = out_enableB ? q : 1'bz;
endmodule
	


	
/*
	Single register (32-bits)

	Contains 32 DFF_tri modules
	in_enable: 1 = allow you to write to DFFs
	out_enableA: 1 = allow you to read from DFF (q) and write to bus A
	out_enableB: 1 = allow you to read from DFF (q) and write to bus B
	d: 32-bits specifying signal to be written
	clk: 1-bit clock signal
	pr/clr: 1-bit preset or clear
	outA: 32-bits specifying output A
	outB: 32-bits specifying output B
 */
module single_register (d, clk, clr, pr, in_enable, out_enableA, out_enableB, outA, outB);
	input[31:0] d;
	input clk, clr, pr, in_enable, out_enableA, out_enableB;
	output[31:0] outA, outB;
	
	genvar i;
	generate
	for(i=0; i < 32; i = i + 1) begin: outer
	// enstantiate a DFF_tri every time I loop (therefore create 32 of them)
		edwin_dff_tri dfftri(.d(d[i]), .clk(clk), .clr(clr), .pr(pr), .in_enable(in_enable), .out_enableA(out_enableA), .out_enableB(out_enableB), .outA(outA[i]), .outB(outB[i]));
	end
	endgenerate
endmodule
	



/*
	5-->32-bit decoder. 
	in[5]: input signal
	out[32]: output will have one bit high, remainder low, which signifies the decoded position.  
 */
module decoder (in, out);
	input[4:0] in;
	output[31:0] out;
	wire n0, n1, n2, n3, n4;
	
	not not0(n0, in[0]);
	not not1(n1, in[1]);
	not not2(n2, in[2]);
	not not3(n3, in[3]);
	not not4(n4, in[4]);
	
	// define decoder statements for all possible input combinations
	and and0(out[0], 	n4, 	n3, 	n2, 	n1, 	n0);
	and and1(out[1], 	n4, 	n3, 	n2, 	n1, 	in[0]);
	and and2(out[2], 	n4, 	n3, 	n2, 	in[1], 	n0);
	and and3(out[3], 	n4, 	n3, 	n2, 	in[1], 	in[0]);
	and and4(out[4], 	n4, 	n3, 	in[2], 	n1, 	n0);
	and and5(out[5], 	n4, 	n3, 	in[2], 	n1, 	in[0]);
	and and6(out[6], 	n4, 	n3, 	in[2], 	in[1], 	n0);
	and and7(out[7], 	n4, 	n3, 	in[2], 	in[1], 	in[0]);
	and and8(out[8], 	n4, 	in[3], 	n2, 	n1, 	n0);
	and and9(out[9], 	n4, 	in[3], 	n2, 	n1, 	in[0]);
	and and10(out[10], 	n4, 	in[3], 	n2, 	in[1], 	n0);
	and and11(out[11], 	n4, 	in[3], 	n2, 	in[1], 	in[0]);
	and and12(out[12], 	n4, 	in[3], 	in[2], 	n1, 	n0);
	and and13(out[13], 	n4, 	in[3], 	in[2], 	n1, 	in[0]);
	and and14(out[14], 	n4, 	in[3], 	in[2], 	in[1], 	n0);
	and and15(out[15], 	n4, 	in[3], 	in[2], 	in[1], 	in[0]);
	and and16(out[16], 	in[4], 	n3, 	n2, 	n1, 	n0);
	and and17(out[17], 	in[4], 	n3, 	n2, 	n1, 	in[0]);
	and and18(out[18], 	in[4], 	n3, 	n2, 	in[1], 	n0);
	and and19(out[19], 	in[4], 	n3, 	n2, 	in[1], 	in[0]);
	and and20(out[20], 	in[4], 	n3, 	in[2], 	n1, 	n0);
	and and21(out[21], 	in[4], 	n3, 	in[2], 	n1, 	in[0]);
	and and22(out[22], 	in[4], 	n3, 	in[2], 	in[1], 	n0);
	and and23(out[23], 	in[4], 	n3, 	in[2], 	in[1], 	in[0]);
	and and24(out[24], 	in[4], 	in[3], 	n2, 	n1, 	n0);
	and and25(out[25], 	in[4], 	in[3], 	n2, 	n1, 	in[0]);
	and and26(out[26], 	in[4], 	in[3], 	n2, 	in[1], 	n0);
	and and27(out[27], 	in[4], 	in[3], 	n2, 	in[1], 	in[0]);
	and and28(out[28], 	in[4], 	in[3], 	in[2], 	n1, 	n0);
	and and29(out[29], 	in[4], 	in[3], 	in[2], 	n1, 	in[0]);
	and and30(out[30], 	in[4], 	in[3], 	in[2], 	in[1], 	n0);
	and and31(out[31], 	in[4], 	in[3], 	in[2], 	in[1], 	in[0]);

endmodule


/*
	d_flip_flop_reg implementation, created by ECE350 TA Staff
*/
module d_flip_flop_reg(d, clk, clrn, prn, ena, q);
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