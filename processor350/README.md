# 550 Processor

## New Instructions
4 new instructions
### init_lab $rd, $rs, $rt
- Function
	- Initialize the LAB counter to prepare for a loop of length $rs, incremented each step by $rd.
	- Reset the offset value (set it to 0)
	- Set "LOOP" register to 1
- OP: 11100
- R-type 
	- $rd[26:22]: size of incrementer
	- $rs[21:17]: size of loop 
	- $rt[16:12]: Index of CAM to access

### b_lab $rd, N
- Function
	- If $rd(LOOP) is high, then... 
		- Next PC = PC + 1 + N
		- Increment $rd(LAB) offset value
		- WB new $rd(LAB) value to $rd
	- If $rd(LOOP) is low, then...
		- Next PC = PC + 1
		- RESET all values at $rd(LOOP)
- OP: 11101
- JII-type
	- $rd[26:22]: Index of CAM to access
	- N[21:0]: (offset of current PC)
		- need to create a new signed extender (the original N architecture does unsigned extension)


### lw_lab $rd, $rs, $rt
- Function
	- Load value at M[$rs + $rt(LAB)] into $rd.
- OP: 11110
- R-type
	- $rd[26:22]: Actual register where data will be loaded.
	- $rs[21:17]: Base address of loop memory access.
	- $rt[16:12]: Index of CAM.

### sw_lab $rd, $rs, N
- Function
	- Store value in $rd into M[$rs + $rt(LAB)].
- OP: 11110
- R-type
	- $rd[26:22]: Actual register where data is located.
	- $rs[21:17]: Base address of loop memory access.
	- $rt[16:12]: Index of CAM.

## LAB Interface
The Load Address Buffer will provide the following interface.

- input[4:0] `register_index`
	- Register index of the CAM to be accessed.

- input[31:0] `threshold_value`
	- Threshold value to compare loop against (set by lab_init).

- input[31:0] `incrementer_value`
	- Value to increment the offset by (set by lab_init).

- input `initialize_WE`
	- Write enable for setting the incrementer_value. When high, the incrementer_value will need to be set to whatever input is given to it (initialize_WE = inst.lib_init).

- input `increase_address`
	- Goes high for one signal, when b_lab is called and taken. Increases the stored address by incrementer_value

- input `clock`
	- Clock to control the signal. Positive Edge clocked.

- input `reset`
	- Reset all values (do when b_lab is NOT taken)

- output[31:0] `address_out`
	- Address used to access memory. This address will be added to base memory from the lw_lab and sw_lab instructions.

- output `LOOP`
	- This output is HIGH if we need to take the branch.
	- This output is LOW if we should NOT take the branch.



## Implementing LAB

### init_lab $rd, $rs, N
Decode
- read the values from register file
	- $rd[26:22]: size of incrementer
	- $rs[21:17]: size of loop 
Execute
- N/A
LAB Interface
- `register_index` = $rt[16:12]
- `threshold_value` = $rs value
- `incrementer_value` = $rd value
- `initialize_WE` = TRUE
- `increase_address` = LOW
- `reset` = LOW

### lw_lab $rd, $rs, $rt
Decode
- read the values from register file
	- $rs[21:17]: Base address of loop memory access.
Execute
- ALU inputs: $rs_value + LAB.`address_out`
LAB Interface
- `register_index` = $rt[16:12]
- `threshold_value` = N/A
- `incrementer_value` = N/A
- `initialize_WE` = LOW
- `increase_address` = LOW
- `reset` = LOW
Memory
- Same as usual (assuming we MUX the address in during Execute)
Writeback
- Write memory access value into $rd[26:22]

### sw_lab $rd, $rs, $rt
Decode
- read the values from register file
	- $rd[26:22]: Actual register where data is located.
	- $rs[21:17]: Base address of loop memory access.
Execute
- ALU inputs: $rs_value + LAB.`address_out`
LAB Interface
- `register_index` = $rt[16:12]
- `threshold_value` = N/A
- `incrementer_value` = N/A
- `initialize_WE` = LOW
- `increase_address` = LOW
- `reset` = LOW
Memory
- Write value into memory at computed address.

### b_lab $rd, N
Fetch
- sign extend N from 22 to 32 (signed)
- resolve branch
	- if(LOOP) 
		- Next PC = PC + 1 + N
	- if(!LOOP)
		- Next PC = PC + 1
- Feed result of LOOP into rest of pipeline
	- set lowest bit fed into F/D latch = LOOP
Decode
- if LOOP (lowest bit of instruction and inst.op == b_lab), then instruction sent into the next loop will be:
	- addi $rd, $r0, address_out of LAB
LAB Interface
- `register_index` = $rd[26:22]
- `threshold_value` = N/A
- `incrementer_value` = N/A
- `initialize_WE` = LOW
- `increase_address` = HIGH
- `reset` = TRUE when b_lab reaches Decode stage


### Final LAB Interface Values
- `register_index`
	- if (b_lab): $rd[26:22]
	- if !(b_lab): $rt[16:12] 
- `threshold_value` = $rs value
- `incrementer_value` = $rd value
- `initialize_WE` = init_lab = (xinst[31:27] == 11100)
- `increase_address` = b_lab = (xinst[31:27] == 11101)
- `reset` = dedode.b_lab and (!LOOP) = (d_inst[31:27] == 11101) && ($rt.(LOOP) is LOW) = do_not_b_lab
- `clock` = clock



### Track Clock Cycles
- Create Register that will be incremented each clock cycle.


