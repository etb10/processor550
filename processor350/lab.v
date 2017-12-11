module lab(
	input[4:0] register_index,
	input[31:0] threshold_value,
	input[31:0] incrementer_value,
	input initialize_WE,
	input increase_address,
	input clock,
	input reset,
	output[31:0] address_out,
	output loop
	);

// wire declarations
reg[31:0] offset_reg, increment_reg, threshold_reg;
reg loop_reg;

// set value of offset and stored registers as necessary
always@(posedge clock)
begin
	if(reset) // if reset, then set values back to 0
	begin
		offset_reg <= 0;
		threshold_reg <= 0;
		increment_reg <= 0;
	end
	else if(initialize_WE) // if we're initializing
	begin
		offset_reg <= 0;	// initialize offset value
		threshold_reg <= threshold_value;	// set threshold to input
		increment_reg <= incrementer_value; // set incrementer to input
	end
	else if(increase_address) // if we should increase address
	begin
		offset_reg <= offset_reg + increment_reg; // increase offset register
		// threshold_reg constant
		// increment_reg constant
	end
end
// convert register to a wire
assign address_out = offset_reg;

// calculate loop value
// if the offset_reg is less than threshold_reg, then output HIGH
// if the offset_reg is more or equal to threshold_reg, then output LOW
always@(posedge clock)
begin
	if(reset)
		loop_reg <= 0;
	else if(initialize_WE)
		loop_reg <= 1;
	else 
	begin
		if(offset_reg < threshold_reg)
			loop_reg <= 1;
		else
			loop_reg <= 0;
	end
end
// convert register to a wire
assign loop = loop_reg;

endmodule // lab

