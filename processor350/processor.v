/**
 * The processor takes in two inputs and returns two outputs
 *
 * Inputs
 * clock: this is the clock for your processor at 50 MHz
 * reset: we should be able to assert a reset to start your pc from 0 (sync or
 * async is fine)
 *
 * Outputs
 * dmem_data_in: this should connect to the wire that feeds in data to your dmem
 * dmem_address: this should be the address of data that you write data to
 *
 * Notes
 * You will need to figure out how to instantiate two memory elements, called
 * "syncram," in Quartus: one for imem and one for dmem. Each should take in a
 * 12-bit address and allow for storing a 32-bit value at each address. Each
 * should have a single clock.
 *
 * Each memory element should have a corresponding .mif file that initializes
 * the memory element to certain value on start up. These should be named
 * imem.mif and dmem.mif respectively.
 *
 * imem
 * Inputs:  12-bit address, 1-bit clock enable, and a clock
 * Outputs: 32-bit instruction
 *
 * dmem
 * Inputs:  12-bit address, 1-bit clock, 32-bit data, 1-bit write enable
 * Outputs: 32-bit data at the given address
 *
 * Additional modules
 * dffe - D-Flip Flop
 * single_register_pro - 32 bit register
 * 
 * 
 */
module processor(clock, reset, dmem_data_in, dmem_address
	  );
	 

	 
    input clock, reset;

	//output [31:0] data_op_A, data_op_B_final_after_setx;
    //output [4:0]  ALU_op_code;
    //output x_n_eq, x_LT, x_ovf;
	 
    output [31:0] dmem_data_in;//, x_PC, x_alu_out, x_RS1, x_RS2; //, data_op_B_final_after_setx, data_op_A;
    output [11:0] dmem_address;

    // Figure out how to generate a Quartus syncram component and commit the
    // generated verilog file. Make sure you configure it correctly!

    

    /* YOUR CODE STARTS HERE */

	assign dmem_data_in = m_data_in;
	assign dmem_address = m_data[11:0];


    // *** stall wires *** //
	 
	 
	 
    wire mult_div_stall, data_haz_stall, jr_stall;

    // mult_div_stall = (mult/div not done) AND (MULT instruction in X OR DIV instruction in X)
	 
	 
	 
    wire x_mult, x_divd, x_ALU;
    assign x_mult = /*ALU OP == 00110 */ !x_inst[6] & !x_inst[5] & x_inst[4] & x_inst[3] & !x_inst[2];
    assign x_divd = /*ALU OP == 00111 */ !x_inst[6] & !x_inst[5] & x_inst[4] & x_inst[3] &  x_inst[2];
    assign x_ALU  = /*ALU OP == 00000 */ !x_inst[31] & !x_inst[30] & !x_inst[29] & !x_inst[28] & !x_inst[27];
    assign mult_div_stall = (!data_resultRDY & ( x_mult | x_divd ) & x_ALU) | /* add the lw stall for reduced coding complexity*/ lw_sw_stall;

    // data_haz_stall = (D_inst.OP == LOAD) && ( (F_inst.RS == D_inst.RD) || (F_inst.RT == D_inst.RD) ) && (F_inst.OP != STORE)
	 
	 
	 
    wire d_load, f_not_store, rs_equal_rd, rt_equal_rd;
    assign d_load = /*OP == 01000 */ !d_inst[31] &  d_inst[30] & !d_inst[29] & !d_inst[28] & !d_inst[27];
    assign f_not_store = /*OP != 00111 */ !(!d_inst[31] & !d_inst[30] &  d_inst[29] &  d_inst[28] &  d_inst[27]);
    compare5 rs_rd(f_inst[21:17], d_inst[26:22], rs_equal_rd); // (F_inst.RS == D_inst.RD)
    compare5 rt_rd(f_inst[16:12], d_inst[26:22], rt_equal_rd); // (F_inst.RT == D_inst.RD)
    assign data_haz_stall = (d_load) & ( (rs_equal_rd) | (rt_equal_rd) ) & (f_not_store);

    // jr_stall = (D_inst.op == jr) or (X_inst.op == jr)
	
	 
    wire d_jr;
    // x_jr is already done below
    assign d_jr  = /* op == 00100 */ !d_inst[31] & !d_inst[30] &  d_inst[29] & !d_inst[28] & !d_inst[27];
    assign jr_stall = d_jr | x_jr;






	 ///*REMOVE*/ output[31:0] next_PC, f_PC, f_PC_plus4, f_inst, j_PC, j_PC_hold, f_T_extended, d_PC;
	 ///*REMOVE*/ output d_jr, PC_latch_enable, f_jump, f_j, f_jal, x_jr, take_branch, m_op_bne, m_op_blt, nop_notfinst, fd_reset, fd_stall, mult_div_stall, data_haz_stall, jr_stall, x_mult, x_divd, x_ALU, d_load, f_not_store, rs_equal_rd, rt_equal_rd;

    // *** FETCH STAGE *** //
	 
    wire[31:0] next_PC, f_PC, f_PC_plus4, f_inst, m_branch_dest_after_bex, 
        j_PC, j_PC_hold, f_T_extended;

    // PC_latch is the program counter
	 wire PC_latch_enable;
	 assign PC_latch_enable = !(mult_div_stall | data_haz_stall);
    single_register_pro PC_latch(.d(next_PC), .clk(clock), .clr(reset), .write_enable(PC_latch_enable), .q(f_PC));
    // ALU to add 4 bytes to the f_PC value
    adder32_pro next_PC_ALU(.data_a(f_PC), .data_b(32'd1), .sum(f_PC_plus4), .sub(1'b0));
    // choose between PC = jump value (T), x_RS1 (jr), or f_PC_plus4 based on jump logic
    // f_jump is high if F_inst.op == j or jal, or if X_inst == jr
	 
	 
    wire f_jump, f_j, f_jal, x_jr, f_jr; 
    assign f_j   = /* op == 00001 */ !f_inst[31] & !f_inst[30] & !f_inst[29] & !f_inst[28] &  f_inst[27];
    assign f_jal = /* op == 00011 */ !f_inst[31] & !f_inst[30] & !f_inst[29] &  f_inst[28] &  f_inst[27];
    assign x_jr  = /* op == 00100 */ !x_inst[31] & !x_inst[30] &  x_inst[29] & !x_inst[28] & !x_inst[27];
	 assign f_jr  = /* op == 00100 */ !f_inst[31] & !f_inst[30] &  f_inst[29] & !f_inst[28] & !f_inst[27];
    assign f_jump = (
        /* F_inst.op == j */    (f_j)   |
        /* F_inst.op == jal */  (f_jal) |
        /* X_inst.op == jr */   (x_jr) 
        );
    assign f_T_extended[31:27] = 5'd0; assign f_T_extended[26:0] = f_inst[26:0]; // unsigned extend f_inst.T
	 
    assign j_PC_hold = (f_j | f_jal) ? /* F_inst.T extended to 32 */ f_T_extended : data_op_B;
    assign j_PC      = (f_jump) ? j_PC_hold : f_PC_plus4 ;

    // ALU: add 4 to m_PC
    adder32_pro m_PC_add4(.data_a(m_PC), .data_b(32'd1), .sum(m_PC_plus4), .sub(1'b0));
    // ALU: add sign extended m_inst_imm to m_PC_plus4
    sign_extender m_imm_extender(m_inst[16:0], m_imm_xt);
    adder32_pro m_PC_plus4_add_imm(.data_a(m_PC_plus4), .data_b(m_imm_xt), .sum(m_branch_dest), .sub(1'b0));

    // MUX between branch value and jump/next PC
	 
    wire take_branch, m_op_bne, m_op_blt;
    assign m_op_bne = /* op == 00010 */ !m_inst[31] & !m_inst[30] & !m_inst[29] &  m_inst[28] & !m_inst[27];
    assign m_op_blt = /* op == 00110 */ !m_inst[31] & !m_inst[30] &  m_inst[29] &  m_inst[28] & !m_inst[27];
    assign take_branch = (
        /*M_inst.op == bne && m_neq*/
        m_op_bne & m_neq
        |
        /*OR m_inst.op == blt && m_lt <-- the operands are backwards for blt, so instead make it greater than or equal to*/
        m_op_blt & (!m_lt & m_neq)
		  |
		  /*OR m_op_bex && m_neq*/
		  m_op_bex & m_neq
        );

	 assign m_branch_dest_after_bex = m_op_bex ? m_bex_32 : m_branch_dest;
    assign next_PC = /*true to take branch val*/ take_branch ? m_branch_dest_after_bex : j_PC ;    

    // Instruction Memory
    imemm myimem(
        .address    (/* 12-bit wire */ f_PC[11:0]),       // address of data
        .clken      (/* 1-bit signal */ 1'b1),            // clock enable (optional)
        .clock      (!clock),                             // may need to invert the clock
        .q          (/* 32-bit instruction */ f_inst) // the raw instruction
    );
    // MUX between nop and f_inst based on (take_branch) or (data_haz_stall) or (jr_stall)
	 
	 
    wire nop_notfinst;
    assign nop_notfinst = take_branch | data_haz_stall | jr_stall;
    assign d_inst_in = nop_notfinst ? 32'd0 : f_inst;
	 assign d_PC_in   = nop_notfinst ? 32'd0 : f_PC;

    // F/D Latch
    wire fd_reset, fd_stall;
    or fdr(fd_reset, reset, take_branch);   // reset signal = reset OR take_branch
    assign fd_stall = mult_div_stall;       // stall signal = mult_div_stall (MUST INVERT INTO ENABLE)
    single_register_pro fd_PC_latch(.d(d_PC_in), .clk(clock), .clr(fd_reset), 
        .pr(1'b0), .write_enable(!fd_stall), .q(d_PC)); // f_PC latch into d_PC
    single_register_pro fd_inst_latch(.d(d_inst_in), .clk(clock), .clr(fd_reset), 
        .pr(1'b0), .write_enable(!fd_stall), .q(d_inst)); // d_inst_in into d_inst









    // *** DECODE STAGE *** //
	 
	 // output[31:0] d_inst, d_PC, d_inst_in, d_rs1_data, d_rs2_data, w_rd_data, w_inst, w_PC_plus4;
	 // output w_inst_writesback, m_inst_writesback, d_br_or_sw, w_jal;
	 // output[4:0] w_rd, d_rs, d_rs2, w_rd_after_exception, d_rs2_hold;
	 
    wire[31:0] d_inst, d_PC, d_inst_in, d_PC_in;

    // wires that will control regfile
    wire[31:0] d_rs1_data, d_rs2_data, w_rd_data, w_rd_data_after_exception;

    wire w_inst_writesback, m_inst_writesback, d_op_bex, d_op_setx;
    wire[4:0] w_rd, d_rs, d_rs2, w_rd_after_exception, d_rs2_hold;

    // w_inst_writesback = w is an instruction that writes back
    assign w_inst_writesback = /*any alu op 00000*/   (( (!w_inst[31] & !w_inst[30] & !w_inst[29] & !w_inst[28] & !w_inst[27]) 	|
                               /*addi       00101*/   	(!w_inst[31] & !w_inst[30] &  w_inst[29] & !w_inst[28] &  w_inst[27]) 	|
                               /*lw         01000*/   	(!w_inst[31] &  w_inst[30] & !w_inst[29] & !w_inst[28] & !w_inst[27]) )	&
										 /*rd != 	  00000*/ 	!(!w_inst[26] & !w_inst[25] & !w_inst[24] & !w_inst[23] & !w_inst[22])) 	|
										 /*jal        00011*/	(!w_inst[31] & !w_inst[30] & !w_inst[29] &  w_inst[28] &  w_inst[27]) ;                               
    
	 assign m_inst_writesback = /*any alu op 00000*/   (( (!m_inst[31] & !m_inst[30] & !m_inst[29] & !m_inst[28] & !m_inst[27]) 	|
                               /*addi       00101*/   	(!m_inst[31] & !m_inst[30] &  m_inst[29] & !m_inst[28] &  m_inst[27]) 	|
                               /*lw         01000*/   	(!m_inst[31] &  m_inst[30] & !m_inst[29] & !m_inst[28] & !m_inst[27]) )	&
										 /*rd != 	  00000*/ 	!(!m_inst[26] & !m_inst[25] & !m_inst[24] & !m_inst[23] & !m_inst[22])) 	|
										 /*jal        00011*/	(!m_inst[31] & !m_inst[30] & !m_inst[29] &  m_inst[28] &  m_inst[27]) ;
										 
	 // w_rd = w_inst.rd OR ($31) if W.inst == jal (OP = 00011)
    assign w_rd  = (!w_inst[31] & !w_inst[30] & !w_inst[29] &  w_inst[28] &  w_inst[27]) ? 5'd31 : w_inst[26:22];
    assign w_rd_after_exception = w_exc ? 5'd30 : w_rd;
	 

	 
	 assign d_op_bex  = /*bex       10110*/   ( d_inst[31] & !d_inst[30] &  d_inst[29] &  d_inst[28] & !d_inst[27]);
	 assign d_op_setx = /*setx      10101*/   ( d_inst[31] & !d_inst[30] &  d_inst[29] & !d_inst[28] &  d_inst[27]);
	 // d_rs is always rs of d_inst, unless we call bex - then it is $r30
    assign d_rs  = d_op_bex ? 5'd30 : d_inst[21:17];

    // d_rs2 is MUX between rt (0) and rd (1)
    wire d_br_or_sw;
    assign d_br_or_sw = /*branch    00-10*/   (!d_inst[31] & !d_inst[30]               &  d_inst[28] & !d_inst[27]) |
								/*jr        00100*/   (!d_inst[31] & !d_inst[30] &  d_inst[29] & !d_inst[28] & !d_inst[27]) |
                        /*sw        00111*/   (!d_inst[31] & !d_inst[30] &  d_inst[29] &  d_inst[28] &  d_inst[27]);                               
    assign d_rs2_hold = d_br_or_sw ? d_inst[26:22] : d_inst[16:12];
	 // d_rs2 is $r0 if we call bex
	 assign d_rs2 = d_op_bex ? 5'd0 : d_rs2_hold;
	 

    // w_rd_data = w_data OR w_PC + 4 if w_inst.op == jal
    wire w_jal;
	 wire[31:0] w_PC_plus4;
    assign w_jal = /* op == 00011 */ !w_inst[31] & !w_inst[30] & !w_inst[29] &  w_inst[28] &  w_inst[27];
    adder32_pro next_PC_jal(.data_a(w_PC), .data_b(32'd1), .sum(w_PC_plus4), .sub(1'b0));
    assign w_rd_data = w_jal ? w_PC_plus4 : w_data;
	 assign w_rd_data_after_exception = w_exc ? w_exception_ID : w_rd_data;
	 
    regfile papa_reggy(
        .clock(!clock),
        .ctrl_writeEnable(w_inst_writesback),
        .ctrl_reset(reset), 
        .ctrl_writeReg(w_rd_after_exception),
        .ctrl_readRegA(d_rs), 
        .ctrl_readRegB(d_rs2), 
        .data_writeReg(w_rd_data_after_exception),
        .data_readRegA(d_rs1_data), 
        .data_readRegB(d_rs2_data));

    // D/X Latch
	 wire[31:0] d_inst_latch, d_T_extended, d_inst_latch_final;
    wire dx_reset, dx_stall;
    assign dx_reset = reset | take_branch;
    assign dx_stall = mult_div_stall;
	 
	 // d_op_bex makes x_inst think it's doing sub $r0, $r30, $r0
	 assign d_inst_latch  	   = d_op_bex  ? 32'b00000000001111000000000000000100 : d_inst;
	 assign d_inst_latch_final = d_op_setx ? 32'b00101111100000000000000000000000 : d_inst_latch;
	 
	 
    single_register_pro dx_PC_latch(.d(d_PC), .clk(clock), .clr(dx_reset), 
        .pr(1'b0), .write_enable(!dx_stall), .q(x_PC));     // latch PC 
    single_register_pro dx_inst_latch(.d(d_inst_latch_final), .clk(clock), .clr(dx_reset), 
        .pr(1'b0), .write_enable(!dx_stall), .q(x_inst));   // latch inst
    single_register_pro dx_rs1_latch(.d(d_rs1_data), .clk(clock), .clr(dx_reset), 
        .pr(1'b0), .write_enable(!dx_stall), .q(x_RS1));     // latch d_rs1_data
    single_register_pro dx_rs2_latch(.d(d_rs2_data), .clk(clock), .clr(dx_reset), 
        .pr(1'b0), .write_enable(!dx_stall), .q(x_RS2)); // latch d_rs2_data

	 dffe dx_bex_latch(.d(d_op_bex), .clk(clock), .clrn(!dx_reset), .ena(!dx_stall), .q(x_op_bex));
	 dffe dx_setx_latch(.d(d_op_setx), .clk(clock), .clrn(!dx_reset), .ena(!dx_stall), .q(x_op_setx));
	 
	 assign d_T_extended[31:27] = 5'd0; assign d_T_extended[26:0] = d_inst[26:0]; // unsigned extend d_inst.T

	 // latch BEX value to X stage
	 single_register_pro dx_bex_T_latch(.d(d_T_extended), .clk(clock), .clr(dx_reset), 
        .pr(1'b0), .write_enable(!dx_stall), .q(x_bex_32)); // latch d_rs2_data


		  
		  
		  
		  
		  



    // *** EXECUTE STAGE *** //
	 

	 // /* REMOVE */ output[31:0] x_RS1, x_RS2, x_inst, x_PC, /*x_imm_xt,*/ data_op_A, data_op_A_hold, data_op_B, data_op_B_hold, data_op_B_final_after_setx, /* x_alu_out, */ x_data_final, x_multdiv_out, x_data_mul_div /*, x_inst_or_nop*/;
	 // /* REMOVE */ output m_op_bex, m_neq, m_s2_exc_writeback, x_exc, m_exc, w_exc, x_s1_equals_31, w_s1_jal_writeback, m_s1_jal_writeback, x_s1_equals_30, w_s1_exc_writeback, m_s1_exc_writeback, x_jal, x_s1_m_equal, x_s1_w_equal, x_br_or_sw, m_br_or_sw, w_br_or_sw, x_s2_m_equal, x_s2_w_equal, x_i_type, x_LT, x_ovf, x_n_eq, x_data_exception, x_mult_or_div, ctrl_MULT, ctrl_DIV, x_s2_equals_31, w_jal_writeback, m_jal_writeback, m_jal, w_s2_exc_writeback, x_s2_equals_30;

	  ///* REMOVE */ output[4:0] x_rs2_holder, m_inst_rs2_holder, w_inst_rs2_holder;
	  ///* REMOVE */ output[2:0] x_exc_ID, m_exc_ID, w_exc_ID;
	 
	 
    wire[31:0] x_RS1, x_RS2, x_inst, x_PC, x_imm_xt, x_bex_32;

    // MUX between writeback values into operands
    wire[31:0] data_op_A, data_op_A_hold, data_op_B, data_op_B_hold, data_op_B_final, w_ex_or_jal_s1;
    wire x_s1_m_equal, x_s1_w_equal, x_op_bex, x_op_setx;

    compare5 cxms1(x_inst[21:17], m_inst[26:22], x_s1_m_equal);
    compare5 cxws1(x_inst[21:17], w_inst[26:22], x_s1_w_equal);
	 
	 
	 // if JAL, must also write value back if x_rs2 is 31
	 // write from w instruction
	 wire[31:0] w_s1_data_or_jal_PC, m_s1_data_or_jal_PC, m_ex_or_jal_s1;
	 wire x_s1_equals_31, w_s1_jal_writeback, m_s1_jal_writeback, x_s1_equals_30, w_s1_exc_writeback, m_s1_exc_writeback;
	 compare5 cxms1jal(x_inst[21:17], 5'd31, x_s1_equals_31); // check if x_rs1 equals 31 (JAL destination)
	 assign w_s1_jal_writeback = (x_s1_equals_31) & (w_jal);	// if x_rs2 is 31 && w_inst is a JAL
	 assign m_s1_jal_writeback = (x_s1_equals_31) & (m_jal);
	 
	 compare5 cxms1_30(x_inst[21:17], 5'd30, x_s1_equals_30); // check if x_rs1 equals 30 (Exception destination)
	 assign w_s1_exc_writeback = x_s1_equals_30 & w_exc;
	 assign m_s1_exc_writeback = x_s1_equals_30 & m_exc;
	 
	 
	 // w_data_or_jal_PC is input to first MUX (TAKING THE place of w_data)
	 assign w_ex_or_jal_s1      = w_s1_jal_writeback ? w_PC_plus4 : w_exception_ID;
	 assign w_s1_data_or_jal_PC = (w_s1_jal_writeback | w_s1_exc_writeback) ? w_ex_or_jal_s1 : w_data;
	 assign m_ex_or_jal_s1      = m_s1_jal_writeback ? m_PC_plus4 : m_exception_ID;
	 assign m_s1_data_or_jal_PC = (m_s1_jal_writeback | m_s1_exc_writeback) ? m_ex_or_jal_s1 : m_data;
	 
    // if w should be written back, take it

    assign data_op_A_hold = ((x_s1_w_equal & w_inst_writesback) | w_s1_jal_writeback) | w_s1_exc_writeback ? w_s1_data_or_jal_PC : x_RS1;
    // if m should be written back, take it (prioritize becuase newer inst)
    assign data_op_A      = ((x_s1_m_equal & m_inst_writesback) | m_s1_jal_writeback) | m_s1_exc_writeback ? m_s1_data_or_jal_PC : data_op_A_hold;


    // x_rs2_holder is MUX between rt (0) and rd (1)
    wire x_br_or_sw, m_br_or_sw, w_br_or_sw;
    wire[4:0] x_rs2_holder, m_inst_rs2_holder, w_inst_rs2_holder;
    assign x_br_or_sw = /*branch    00-10*/   (!x_inst[31] & !x_inst[30]               &  x_inst[28] & !x_inst[27]) |
						/*jr        00100*/   (!x_inst[31] & !x_inst[30] &  x_inst[29] & !x_inst[28] & !x_inst[27]) |
                        /*sw        00111*/   (!x_inst[31] & !x_inst[30] &  x_inst[29] &  x_inst[28] &  x_inst[27]);                               
    assign x_rs2_holder = x_br_or_sw ? x_inst[26:22] : x_inst[16:12];
	 
	assign m_inst_rs2_holder = m_inst[26:22];
	assign w_inst_rs2_holder = w_inst[26:22];
	 
    wire x_s2_m_equal, x_s2_w_equal;
    compare5 cxms2(x_rs2_holder, m_inst_rs2_holder, x_s2_m_equal);
    compare5 cxws2(x_rs2_holder, w_inst_rs2_holder, x_s2_w_equal);
	 	 
	 // if JAL, must also write value back if x_rs2 is 31
	 // write from w instruction
	 wire[31:0] w_data_or_jal_PC, m_data_or_jal_PC, w_ex_or_jal, m_ex_or_jal, data_op_B_final_after_setx;
	 wire x_s2_equals_31, w_jal_writeback, m_jal_writeback, m_jal, w_s2_exc_writeback, x_s2_equals_30, m_s2_exc_writeback;	

	 compare5 cxms2jal(x_rs2_holder, 5'd31, x_s2_equals_31); // check if x_rs2 equals 31 (JAL destination)
	 assign w_jal_writeback = (x_s2_equals_31) & (w_jal);	// if x_rs2 is 31 && w_inst is a JAL
	 
	 assign m_jal = /* op == 00011 */ !m_inst[31] & !m_inst[30] & !m_inst[29] &  m_inst[28] &  m_inst[27];
	 assign m_jal_writeback = (x_s2_equals_31) & (m_jal);
	 
	 compare5 cxms2_30(x_rs2_holder, 5'd30, x_s2_equals_30); // check if x_rs2 equals 30 (Exception destination)
	 assign w_s2_exc_writeback = x_s2_equals_30 & w_exc;
	 
	 assign m_s2_exc_writeback = x_s2_equals_30 & m_exc;
	 
	 // w_data_or_jal_PC is input to first MUX (TAKING THE place of w_data)
	 assign w_ex_or_jal      = w_jal_writeback ? w_PC_plus4 : w_exception_ID;
	 assign w_data_or_jal_PC = (w_jal_writeback | w_s2_exc_writeback) ? w_ex_or_jal : w_data;
	 assign m_ex_or_jal      = m_jal_writeback ? m_PC_plus4 : m_exception_ID;
	 assign m_data_or_jal_PC = (m_jal_writeback | m_s2_exc_writeback) ? m_ex_or_jal : m_data;
	 
    // if w should be written back, take it
    assign data_op_B_hold = (x_s2_w_equal & w_inst_writesback) | (w_jal_writeback) | w_s2_exc_writeback ? w_data_or_jal_PC : x_RS2;
    // if m should be written back, take it (prioritize becuase newer inst)
    assign data_op_B      = (x_s2_m_equal & m_inst_writesback) | (m_jal_writeback) | m_s2_exc_writeback ? m_data_or_jal_PC : data_op_B_hold;


	 
	 
    // extend x_immediate
    sign_extender immxt(x_inst[16:0], x_imm_xt);
    // MUX between x_imm_xt and data_op_B
    wire x_i_type;
    assign x_i_type = (
        (/* addi op == 00101 */ !x_inst[31] & !x_inst[30] &  x_inst[29] & !x_inst[28] &  x_inst[27]) |
        (/* sw   op == 00111 */ !x_inst[31] & !x_inst[30] &  x_inst[29] &  x_inst[28] &  x_inst[27]) |
        (/* lw   op == 01000 */ !x_inst[31] &  x_inst[30] & !x_inst[29] & !x_inst[28] & !x_inst[27]) 
        );
    assign data_op_B_final = (x_i_type) ? x_imm_xt : data_op_B;
	 assign data_op_B_final_after_setx = x_op_setx ? x_bex_32 : data_op_B_final;

    wire[31:0] x_alu_out, x_multdiv_out;
    wire x_LT, x_ovf, x_n_eq, x_data_exception;

    // x_mult_or_div = if x_inst signals mult/div op
    wire x_mult_or_div, data_resultRDY;
    assign x_mult_or_div = ( x_mult | x_divd ) & x_ALU;

    // ctrl_DIV and ctrl_MULT signals
    wire ctrl_MULT, ctrl_DIV;
    ctrl_muldiv_signal mult_signal(
        .clock(clock), 
        .reset(reset), 
        .data_resultRDY(data_resultRDY), 
        .x_multdiv_inst( x_mult & x_ALU ), 
        .ctrl_MULTDIV(ctrl_MULT)
        );

    ctrl_muldiv_signal div_signal(
        .clock(clock), 
        .reset(reset), 
        .data_resultRDY(data_resultRDY), 
        .x_multdiv_inst( x_divd & x_ALU ), 
        .ctrl_MULTDIV(ctrl_DIV)
        );

	wire x_op_bne, x_op_blt, x_branch;
	assign x_op_bne = /* op == 00010 */ !x_inst[31] & !x_inst[30] & !x_inst[29] &  x_inst[28] & !x_inst[27];
    assign x_op_blt = /* op == 00110 */ !x_inst[31] & !x_inst[30] &  x_inst[29] &  x_inst[28] & !x_inst[27];
		  
	wire[4:0] ALU_op_code, ALU_op_code_hold;
	assign x_branch = x_op_bne | x_op_blt;
	assign ALU_op_code_hold = x_i_type ? 5'b00000 : x_inst[6:2];
	assign ALU_op_code = x_branch ? 5'b00001 : ALU_op_code_hold;
	
    alu alu_friend(
        .data_operandA(data_op_A), 
        .data_operandB(data_op_B_final_after_setx), 
        .ctrl_ALUopcode(ALU_op_code), 
        .ctrl_shiftamt(x_inst[11:7]), 
        .data_result(x_alu_out), 
        .isNotEqual(x_n_eq), 
        .isLessThan(x_LT), 
        .overflow(x_ovf)
        );


    multdiv mult_div_unit(
        .data_operandA(data_op_A), 
        .data_operandB(data_op_B_final_after_setx), 
        .ctrl_MULT(ctrl_MULT), 
        .ctrl_DIV(ctrl_DIV), 
        .clock(clock), 
        .data_result(x_multdiv_out), 
        .data_exception(x_data_exception), 
        .data_resultRDY(data_resultRDY)
        );

    // choose between mult_div and ALU output
    wire[31:0] x_data_final, x_data_mul_div, x_inst_or_nop;
	wire x_jal;
	assign x_jal = /* op == 00011 */ !x_inst[31] & !x_inst[30] & !x_inst[29] &  x_inst[28] &  x_inst[27];
	 
    assign x_data_final = (x_mult_or_div) ? x_multdiv_out : x_alu_out;

	 
	 // EXCEPTION HANDLING
	 wire[2:0] x_exc_ID, a_or_s_exc, m_or_d_exc, m_or_a_exc;
	 wire x_exc, x_add, x_sub, x_addi; // use x_divd and x_mult
	 assign x_addi = /* addi op == 00101 */ !x_inst[31] & !x_inst[30] &  x_inst[29] & !x_inst[28] &  x_inst[27];
	 assign x_add  = /* ALU  OP == 00000 */ !x_inst[6]  & !x_inst[5]  & !x_inst[4]  & !x_inst[3]  & !x_inst[2];
	 assign x_sub  = /* ALU  OP == 00001 */ !x_inst[6]  & !x_inst[5]  & !x_inst[4]  & !x_inst[3]  &  x_inst[2];
	 
	 assign a_or_s_exc = x_sub ? 3'd3 : 3'd1;									// add or sub
	 assign m_or_d_exc = x_mult ? 3'd4 : 3'd5;								// mul or div
	 assign m_or_a_exc = (x_divd|x_mult) ? m_or_d_exc : a_or_s_exc;	// muldiv or alu
	 assign x_exc_ID   = x_addi ? 3'd2 : m_or_a_exc;						// final
	 
	 // any exception at all triggers x_exc (but must also be an inst that has exception capability)
	 assign x_exc = ((x_data_exception) & (x_divd|x_mult)) | ((x_ovf) & (x_add|x_sub|x_addi));

	
	
	 
	 
    // X/M Latch
    // enable = 1'b1. reset = reset
    assign x_inst_or_nop = mult_div_stall ? 32'd0 : x_inst;
	 wire xm_stall;
	 assign xm_stall = mult_div_stall;
	 
    single_register_pro xm_inst_latch(.d(x_inst_or_nop), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!xm_stall), .q(m_inst));     // latch x_inst_or_nop into m_inst
    single_register_pro xm_rd_latch(.d(x_RS2), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!xm_stall), .q(m_RD));     // latch x_rd into m_rd
    single_register_pro xm_data_latch(.d(x_data_final), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!xm_stall), .q(m_data));     // latch x_data_final into m_data
    single_register_pro xm_PC_latch(.d(x_PC), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!xm_stall), .q(m_PC));     // latch x_PC into m_PC
    dffe x_ovf_dffe(.d(x_ovf), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_ovf)); // latch x_ovf through to m_ovf
    dffe x_neq_dffe(.d(x_n_eq), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_neq)); // latch x_n_eq through to m_neq
    dffe x_lt_dffe(.d(x_LT), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_lt)); // latch x_LT through to m_lt
		
		// latch the exceptions through
		dffe x_exc_latch(.d(x_exc), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_exc)); // latch x_exc through to m_exc
		// latch m_exc_ID through to m_exc_ID
		dffe x_excID0_latch(.d(x_exc_ID[0]), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_exc_ID[0])); 
		dffe x_excID1_latch(.d(x_exc_ID[1]), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_exc_ID[1])); 
		dffe x_excID2_latch(.d(x_exc_ID[2]), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_exc_ID[2])); 


	   dffe xm_bex_latch(.d(x_op_bex), .clk(clock), .clrn(!reset), .ena(!xm_stall), .q(m_op_bex));
	 
		// latch BEX value to M stage
		single_register_pro xm_bex_T_latch(.d(x_bex_32), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!xm_stall), .q(m_bex_32)); // latch d_rs2_data






    // *** MEMORY STAGE *** //
	 
	 ///* REMOVE */ output[31:0] m_RD, m_inst, m_PC, m_PC_plus4, m_data, m_data_in, m_data_final, m_imm_xt, m_branch_dest, mem_out, w_inst, w_PC, w_data;
	 ///* REMOVE */ output m_ovf, m_neq, m_lt, write_rd_back_mw, m_inst_sw, m_inst_lw;
	 

    wire[31:0] m_bex_32, m_exception_ID, m_RD, m_inst, m_PC, m_PC_plus4, m_data, m_data_in, m_data_final, m_imm_xt, m_branch_dest, mem_out;
    wire m_ovf, m_neq, m_lt, m_exc, m_op_bex;

	 wire[2:0] m_exc_ID;
	 
	 sign_extender_3_32 s_ext_m_exception_ID(m_exc_ID, m_exception_ID);

    wire write_rd_back_mw, m_inst_sw, m_inst_lw, write_rd_back_mw_not_r0;
    compare5 wm_rd(w_inst[26:22], m_inst[26:22], write_rd_back_mw);
	 assign write_rd_back_mw_not_r0 = (write_rd_back_mw) & !(!w_inst[26] & !w_inst[25] & !w_inst[24] & !w_inst[23] & !w_inst[22]);
	 
    assign m_inst_sw = /* op == 00111 */ !m_inst[31] & !m_inst[30] &  m_inst[29] &  m_inst[28] &  m_inst[27];
    assign m_inst_lw = /* op == 01000 */ !m_inst[31] &  m_inst[30] & !m_inst[29] & !m_inst[28] & !m_inst[27];

    // choose between writeback and m_RD
    assign m_data_in = write_rd_back_mw_not_r0 ? w_data : m_RD;

    dmem papa_mem(
        .address    (/* 12-bit wire */ m_data[11:0]),       // address of data
        .clock      (!clock),                  // may need to invert the clock
        .data       (/* 32-bit data in */ m_data_in),    // data you want to write
        .wren       (/* 1-bit signal */ m_inst_sw),      // write enable
        .q          (/* 32-bit data out */ mem_out)    // data from dmem
    );

    assign m_data_final = m_inst_lw ? mem_out : m_data;
	 wire mw_stall, lw_sw_stall;
	 assign mw_stall = mult_div_stall;
	 
	 dff_sync_clr mem_stall_signal(.d(m_inst_sw | m_inst_lw), .clk(!clock), .clr(lw_sw_stall), .ena(1'b1), .q(lw_sw_stall));
	 
    // M/W Latch
    single_register_pro mw_data_latch(.d(m_data_final), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!mw_stall), .q(w_data));     // latch m_data_final into w_data
    single_register_pro mw_inst_latch(.d(m_inst), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!mw_stall), .q(w_inst));     // latch m_inst into w_inst
    single_register_pro mw_PC_latch(.d(m_PC), .clk(clock), .clr(reset), 
        .pr(1'b0), .write_enable(!mw_stall), .q(w_PC));     // latch m_PC into w_PC

		// latch the exceptions through
		dffe m_exc_latch(.d(m_exc), .clk(clock), .clrn(!reset), .ena(!mw_stall), .q(w_exc)); // latch x_exc through to m_exc
		// latch m_exc_ID through to m_exc_ID
		dffe m_excID0_latch(.d(m_exc_ID[0]), .clk(clock), .clrn(!reset), .ena(!mw_stall), .q(w_exc_ID[0])); 
		dffe m_excID1_latch(.d(m_exc_ID[1]), .clk(clock), .clrn(!reset), .ena(!mw_stall), .q(w_exc_ID[1])); 
		dffe m_excID2_latch(.d(m_exc_ID[2]), .clk(clock), .clrn(!reset), .ena(!mw_stall), .q(w_exc_ID[2])); 





    // *** WRITEBACK STAGE *** //
    wire[31:0] w_inst, w_PC, w_data, w_exception_ID;
	 wire w_exc;
	 wire[2:0] w_exc_ID;
	 
	 sign_extender_3_32 s_ext_exception_ID(w_exc_ID, w_exception_ID);


endmodule



// use negative clock
module stall_counter1(clock, inst, stall_signal);
	input clock;
	input[31:0] inst;
	output stall_signal;
	
	reg stall_signal;

	wire lw_or_sw;
	assign lw_or_sw = /*00111*/ (!inst[31] & !inst[30] &  inst[29] &  inst[28] &  inst[27]) |
							/*01000*/ (!inst[31] &  inst[30] & !inst[29] & !inst[28] & !inst[27]);
	
	always@(posedge clock) begin
		if(lw_or_sw) begin
			stall_signal = 1;
		end
		
	casex({lw_or_sw, stall_signal})
		1'd1: stall_signal = 0;
		1'd0: stall_signal = 0;
		default: stall_signal = 0;
	endcase
	end
endmodule



/*
Sets ctrl_MULTDIV high for one cycle when a multiply or divide instruction (x_multdiv_inst) comes into the stage.
Upon completion of mult/div, the system resets and can be set for execute again.
*/
module ctrl_muldiv_signal(clock, reset, data_resultRDY, x_multdiv_inst, ctrl_MULTDIV);
    input clock;
    input reset;
    input data_resultRDY;
    input x_multdiv_inst;
    output ctrl_MULTDIV;

    wire x_multdiv_inst_latched, ctrl_MUX;

    // latch mult instruction on neg clock edge
    dffe x_multdiv_inst_dffe(.d(x_multdiv_inst), .clk(!clock), .clrn(!reset), .ena(1'b1), .q(x_multdiv_inst_latched)); 
    // MUX between 0 and x_multdiv_inst_latched
    assign ctrl_MULTDIV = (ctrl_MUX) ? 1'b0 : x_multdiv_inst_latched;
    // latch ctrl_mult signal back into ctrl_MUX
    wire ctrl_WE, data_resultRDY_latched;
    assign ctrl_WE = ctrl_MULTDIV | data_resultRDY_latched;
    dffe ctrl_MUX_dffe(.d(ctrl_MULTDIV), .clk(!clock), .clrn(!reset), .ena(ctrl_WE), .q(ctrl_MUX)); 
    // latch data_resultRDY into data_resultRDY_latched
    dffe ctrl_drr_dffe(.d(data_resultRDY), .clk(!clock), .clrn(!reset), .ena(1'b1), .q(data_resultRDY_latched)); 

endmodule



/*
5 bit bitwise comparator
*/
module compare5(inA, inB, equal);
    input[4:0] inA, inB;
    output equal;

	wire[4:0] xorAB, not_xorAB, or_inA, or_inB;
	wire equal_hold;

    genvar i;
    generate
        for (i = 0; i < 5; i = i + 1) begin: loop1
            xor x1(xorAB[i], inA[i], inB[i]);
            not n1(not_xorAB[i], xorAB[i]);
        end
	 endgenerate
    // and all the wires of not(xor)
	assign or_inA = | inA;
	assign or_inB = | inB;
    	assign equal_hold = & not_xorAB;
	assign equal = equal_hold & or_inA & or_inB;
		
endmodule


/*
17 to 32 signed extender
*/
module sign_extender(in17, out32);
    input[16:0] in17;
    output[31:0] out32;

    assign out32[16:0] = in17;
    genvar i;
    generate
        for (i = 17; i < 32; i = i + 1) begin: loop1
            assign out32[i] = in17[16]; // assign highest bit to rest of 32 bit output
        end
	endgenerate
endmodule


/*
3 to 32 signed extender
*/
module sign_extender_3_32(in3, out32);
    input[2:0] in3;
    output[31:0] out32;

    assign out32[2:0] = in3;
    assign out32[31:3] = 29'd0;
endmodule


  
/*
32-bit Adder/Subtracter

RCA - simple to implement and fast enough
*/
module adder32_pro(data_a, data_b, sum, sub);
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

endmodule // adder32_pro
    
	 
	 
	 

module single_register_pro (d, clk, clr, pr, write_enable, q);
	input[31:0] d;
	input clk, clr, pr, write_enable;
	output[31:0] q;
	
	genvar i;
	generate
	for(i=0; i < 32; i = i + 1) begin: outer
		dffe ctrl_drr_dffe(.d(d[i]), .clk(clk), .clrn(!clr), .ena(write_enable), .q(q[i])); 
	end
	endgenerate
endmodule

/*
	dflipflop implementation, created by ECE350 TA Staff
*/
module dff_sync_clr(d, clk, clr, prn, ena, q);
    input d, clk, ena, clr, prn;
    wire clr;
    wire pr;

    output q;
    reg q;

    assign pr = ~prn;

    initial
    begin
        q = 1'b0;
    end

    always @(posedge clk) begin
        if (q == 1'bx) begin
            q = 1'b0;
        end else if (clr) begin
            q <= 1'b0;
        end else if (ena) begin
            q <= d;
        end
    end
endmodule
