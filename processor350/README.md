# 350 Processor

## Construction of Processor 

### PC Latch
PC must be latched throughout the entire pipeline so that it can be easily accessed by instructions that require branches.
- bne: if branch should be taken, PC = PC + 4 + N
- blt: same.

### Fetch
PC stored in register.
- ALU adds 4 bytes to PC.
- MUX between PC+4 and Jump Address.
- Jump address takes one of the three following values:
	- jal: T[26:0] - immediately 
	- j: T[26:0] - immediately
	- jr: $rd[26:22] - must wait until receives value from D stage.
		- set PC back to the previous value, and have $rd's value written back into PC.

Assume branch not taken, therefore execute then determine correct step.
- Continue with next step. (PC + 4)
- When execute completes, check if we should have branched, then:
	- Flush F, D, X in pipeline.
	- Set PC equal to (X-latched PC + 4 + N)

### Stall Logic
Stall means a few things for the processor:
- PC remains at the same value.
- F/D Latch is disabled.
- nop sent into D/X latch.

Stall Logic
*Remember that $0 should be ignored when considering data hazards, because $0 cannot be written*
Stalls occur for mult/div, RAW, and jr instructions
- mult/div: stall while opcode == 00000 and ALU op == 00110/00111
	- stall until data_result_RDY goes high from this operand
	- Stall = AND between the !data_result_RDY and a signal that indicates this is a mult/div
	- Stall: PC, F/D, D/X
- PC+1 instruction is LOAD, and it's $rd is == current instruction's $rs or $rt, but the PC instruction is not a store.
	- **(D/X.IR.OP == LOAD) && ( (F/D.IR.RS == D/X.IR.RD) || (F/D.IR.RT == D/X.IR.RD) ) && (F/D.IR.OP != STORE)**
	- **(D_inst.OP == LOAD) && ( (F_inst.RS == D_inst.RD) || (F_inst.RT == D_inst.RD) ) && (F_inst.OP != STORE)**
	- load value ready in the WB stage, therefore you must kill one cycle to ensure WB value is ready for the X stage. If the next instruction is a STORE, then you can writeback to M stage without a delay.
	- Stall: PC (and put a nop through F/D)
- jr: stall for two cycles
	- if (D_inst.op == jr) or (X_inst.op == jr)
	- Stall: PC (and put nop through F/D)

### Reset Logic
Resets occur during either a hard reset or a pipeline flush.
- "reset" - clear everything. Whatever signal for flush should be OR'ed with reset.
- flush
	- Reset signal to F/D, D/X, and send nop through F/D


### Instruction Breakdown 
R
- [31:27] Opcode
- [26:22] $rd
- [21:17] $rs
- [16:12] $rt
- [11:7] shamt
- [6:2] ALU op
- [1:0] Zeroes

I
- [31:27] Opcode
- [26:22] $rd
- [21:17] $rs
- [16:0] N
	- N must be sign extended

JI
- [31:27] Opcode
- [26:0] T

JII
- [31:27] Opcode
- [26:22] $rd
- [21:0] Zeroes

### Decode
Stall: MUX the instruction register to be nop.
	- nop will simply add 0 to 0 and store in 0, therefore fine to send these through without worrying about data values.
	- see stall logic for details

Register File Ports
- WE: Determined by WB stage or jal
	- jal: WE enabled to write PC + 4 into $r31
	- WB: Whatever value has been completed will be written back
- RD: Determined by WB stage or jal
	- jal: $r31
	- WB: IR value of $rd for the instruction latched into WB stage
- RS1: Always RS
- RS2: Determined by D stage 
	- Branch instructions: $rd
	- All others: $rt
- RD val: Determined by WB stage or jal
	- jal: PC + 4
	- WB: final value
- Clock: global clock
- Reset: global reset

### Execute 
ALU Inputs
- Operand A
	- MUX between 3 values:
		- 2: $rs (no writeback)
		- 1: X/M writeback (if D/X.IR.RS1 == X/M.IR.RD)
		- 0: M/W writeback (if D/X.IR.RS1 == M/W.IR.RD)
- Operand B
	- MUX between 3 values:
		- 2: $rs (no writeback)
		- 1: X/M writeback (if D/X.IR.RS2 == X/M.IR.RD)
		- 0: M/W writeback (if D/X.IR.RS2 == M/W.IR.RD)
	- MUX between this value and sign-extended immediate:
		- 1: s-xt immediate - if I-type instruction:
			- addi, sw, lw, bne, blt
		- 0: previous MUX otherwise
- OpCode
	- [6:2] of R type instruction

### Memory
LW and SW are the only operations that require this stage.
- Latch value of $rd (output of Operand B first MUX) through into the M stage.
	- MUX between this value and the WB value (if X/M.IR.RD == M/W.IR.RD)
- Latch X final value through X/M latch.

Data Memory Ports
- WREN: Only if SW is the current instruction.
- Address: directly from X stage.
- Clock: (!clock)
- Data: MUX between latched $rd value and the WB value (if X/M.IR.RD == M/W.IR.RD)
- Q: latched through M/W port.

### Writeback
- MUX between LW and any other operations
	- LW: Take Data from memory
	- otherwise: take ALU value (written straight through Memory stage)

## Debugging

### addi
`addi_tests.mif` - tests exclusively `addi` functionality. Processor correctly executes computations. Includes RAW hazards, which correctly stall for program execution. 

### alu operations
`alu_tests.txt` and `alu_tests.mif` - test functionality for `add`, `sub`, `and`, `or`, `sll`, and `sra`. These functions were tested extensively with RAW hazards. 
- Added requirement that writeback only occurs if the instruction type that will actually write back.
- Added requirement that writeback does not occur if $rd is $r0.

### mul and div
- stalling the X/M and M/W latches during mult/div. This makes sure values are written back into the mult/div.

### load and store
Testing load and store latencies 
- Load requires 1 cycle of stall (1 cycle + 1/2 cycle in order to get value. This means 2 cycles total, hence 1 cycle of stall).
- Store requires 1 cycle of stall as well. 

Changes that were made:
- Add a stall for 1 cycle to the entire pipeline.
- This stall includes the M/W latch, in case w_inst is holding the address or value to be stored.
- Done using a dff with synchronous clear. 

`lw_sw_tests.txt` works under a timing simulation.

### Branch Instructions
`branch_tests.txt` holds tests 
- All work.

### Jump Instructions
- `jr` doesn't seem to be working properly.
- Mult/Div instructions after jr instructions are not being properly flushed (might need to add a reset to mult/div).
	- TODO: test this (send mul/div immediately following jr)

- Made sure to latch nop into PC in addition to inst.
- change `x_RS1` to `data_op_A` becuase this gives the written-back value.
- Having trouble getting the correct value written into j_PC. Seems like the value retreived from execute stage is incorrect, meaning there might be an issue with decode.
	- Was reading from $rs, when should have been writing from $rd. Updating to reflect this change (looks similar to sw/lw logic on RS2).
	- `x_br_or_sw` and `m_br_or_sw` now also go high if instruction is jr
	- `j_PC` input is now `data_op_B`
	- Removed `jr_stall` as a stall cause for PC.

- `jal` needs fixing. Currently seems like value is not being written back into `$r31`.
	- Wasn't correctly writing the immediate through the pipeline. 
	- MUX between `x_data_mul_div` and `x_imm_xt` = `x_data_final` based on if `x_inst` is `jal`
	- ^ This approach was incorrect
	- w_PC_plus_4 was only 1 bit long, therefore was truncating the value. Changed definition to `wire[31:0]`.
	- Was not properly writing back into pipeline. Now, MUX between values in case the value is 31.


### Adding Exceptions
If any data exception occurs, need to change the destination register from whatever we had --> $rstatus
- add overflow: $r30 = 1
- addi overflow: $r30 = 2
- sub overflow: $r30 = 3
- mul overflow: $r30 = 4
- div overflow: $r30 = 5

All occur within execute stage, therefore should make:
- 1-bit wire: OR all of the exceptions
- 3-bit wire with number of exception
	- choose between the instruction op_codes
	- MUX using the instruction op_codes to determine which value should be put into the exception code.

Add latches to X/M, M/W:
- 1-bit for exception_status
- 3-bits for exception_number

During writeback stage:
- Sign extend 3-bit wire to 32 during the writeback stage (this reduces the size latches we have to use from 32 --> 3)
- If any exception occurs, then change write address to r30 and make the write data the w_exception_ID

Forward exception signal back (if m_exc and x_rs1 is $r30, then need to write back)
- replace w_PC_plus4 with another MUX between w_exception_ID and w_PC_plus4
- repeat for m_PC_plus4

### Exception Instructions
`bex T` - 10110
- for this instruction, data_op_A needs to be $r30, data_op_final needs to be $r0
	- DONE d_rs = m_op_bex ? $r30 : previous value;
	- DONE d_rs2 = m_op_bex ? $r0 : previous value;
- DONE create m_op_bex
- execute stage: 
	- DONElatch an instruction in that makes x_inst think that it's add $r0, $r30, $r0 (00000000001111000000000000000000) 
		- MUX between ^ that and normal instruciton based on m_op_bex
	- DONE latch signal to m called m_op_bex (and into x)
	- DONE sign extend, then latch immediate value into m_bex_32 (and into x)
- DONE take_branch logic needs addition: m_op_bex & m_neq
- DONE m_branch_dest needs to be MUXed again. 
	- DONE m_branch_dest_after_bex = (m_op_bex) ? m_T_sign_xt : m_branch_dest;
	- sign extend 27 --> 32 (logical) (line 140 has this)

`setx T` - 10101
- same as addi where destination is hard-coded to $r30 and immediate is 27 sign-extended to 32

- DONE define d_op_setx
- DONE latch signal through to X called x_setx
- DONE use 32 bit value latched through from bex (x_bex_32)
- DONE make d_inst: addi $r30, $r0, 0 (00101111100000000000000000000000)
- data_op_B_final needs to be muxed with x_bex_32 if x_op_setx



# 550 Processor

## New Instructions
4 new instructions
### lab_in $rd, $rs, N
- Function
	- Initialize the LAB counter to prepare for a loop of length $rs + N, incremented each step by $rd.
	- Clear the incrementer value (set it to 0)
	- Set "LOOP" register to 1
	- Clear the LAB
- OP: 11100
- I-type 
	- $rd[26:22]: size of incrementer (if $r0, then default set to 1)
	- $rs[21:17]: base size of loop 
	- N[16:0]: amount to add to loop

### blab N
- Function
	- Check if the incrementer value is less than or equal to the max increment value.
		- We can set this as a register, and simply check (branches can be resolved in F stage, unless dependencies exist).
		- Create "LOOP" register
	- If LOOP is high, then next PC = PC + 1 + N
	- If LOOP is low, then...
		- Next PC = PC + 1
		- Allow all values in the LAB to be written back to the real register file (stall processor and pad with nops until all values are out)
		- WB_values = AND(valid_bits of LAB)
	- After checking the value, then set the incrementer value += (incrementer_step_size)
- OP: 11101
- J-type
	- N[26:0]: (offset of current PC)
		- need to create a new signed extender (the original N architecture does unsigned extension)

### lw_lab $rd, $rs, N
- Function
	- Load values into memory at the address in LAB at that register
- OP: 11110
- I-type
	- $rd[26:22]: actual register where data will be loaded
	- $rs[21:17]: register index (the one that will specify which index of the LAB we're using to access this value)
	- N[16:0]: immediate value indicating base location of the memory accessed
		- the memory address accessed will be = N + incrementer value 

### sw_lab $rd, $rs, N
- Function
	- Store values into memory at the address in LAB at that register
- OP: 11111
- I-type
	- $rd[26:22]: actual register containing data to be written to memory
	- $rs[21:17]: register index (the one that will specify which index of the LAB we're using to access this value)
	- N[16:0]: immediate value indicating base location of the memory accessed
		- the memory address accessed will be = N + incrementer value 

## LAB Interface
















