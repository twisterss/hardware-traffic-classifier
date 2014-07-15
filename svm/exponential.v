//////////////////////////////////////////////////////////////////////////////////
// Compute an exponential.
// Work with a ROM. May use shifts to decrease the size of the ROM by
// shifting the received input (1 shift => 1 comparison, 1 add, 1 mult.)
// Compute continually: y = exp(x) at posedge with 6 periods late.
// Set new_result to 1 during 1 period when a new result is out.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

`include "../config/proj_env.v"
`include "config/svm_parameters.v"

// Do not load the module if not required to avoid compilation errors
`ifdef SVM_KERNEL_TYPE_RBF

`include "config/exp_memory.v"

module exponential
(
	input signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x,
	
	input clk,
	input reset,
	input data_valid,
	
	output reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] y,
	output reg new_result
);

	// Registered inputs for each pipeline stage
	reg enable_comp;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x_reg_comp;
	
	reg enable_sel;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x_reg_sel;
	
	reg enable_addr;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x_reg_addr;
	
	reg enable_read;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x_reg_read;
	reg [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_select_reg_read;
	
	reg enable_mul;
	reg [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_select_reg_mul;
	
	reg enable_out;
	reg [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_select_reg_out;
	
	// Shift variables
	reg [`EXP_SHIFT_COUNT-1:0] shift_comparisons;
	wire [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_comparisons_sum;
	wire [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_select_out;
	reg [`EXP_SHIFT_INDEX_WIDTH-1:0] shift_select;
	reg signed [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x_shifted [0:`EXP_SHIFT_COUNT];
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted [0:`EXP_SHIFT_COUNT];
	wire [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted_reduced [0:`EXP_SHIFT_COUNT];
	wire [2 * (`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC) - 1:0] mem_out_shifted_tmp [0:`EXP_SHIFT_COUNT];
	
	// Memory variables
	reg [`EXP_MEM_INDEX_WIDTH-1:0] address;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] memory [0:`EXP_MEM_COUNT - 1];
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out;
	
	// Initialize the memory
	initial begin
		$readmemh({`PROJ_CONFIG_DIR, "exp_memory.list"}, memory);
	end
	
	// Control of the input of each stage of the pipeline
	always @(posedge clk) begin
		// Prepare for comp
		x_reg_comp <= x;
		enable_comp <= reset ? 0 : data_valid;
		// Prepare for sel
		x_reg_sel <= x_reg_comp;
		enable_sel <= reset ? 0 : enable_comp;
		// Prepare for addr
		x_reg_addr <= x_reg_sel;
		enable_addr <= reset ? 0 : enable_sel;
		// Prepare for read
		x_reg_read <= x_reg_addr;
		enable_read <= reset ? 0 : enable_addr;
		shift_select_reg_read <= shift_select;
		// Prepare for mul
		enable_mul <= reset ? 0 : enable_read;
		shift_select_reg_mul <= shift_select_reg_read;
		// Prepare for out
		enable_out <= reset ? 0 : enable_mul;
		shift_select_reg_out <= shift_select_reg_mul;
	end
	
	// Management of the shift to apply to the memory
	generate
		genvar i;
		for (i = 0; i <= `EXP_SHIFT_COUNT; i = i + 1) begin : shift_selection
			assign mem_out_shifted_tmp[i] = (mem_out * `EXP_SHIFT_FACTORS_i);
			assign mem_out_shifted_reduced[i] = (i == `EXP_SHIFT_COUNT) ? mem_out : mem_out_shifted_tmp[i][2 * (`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC) - 1:`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC];
			// comp step: comparaison + shift precomputation
			always @(posedge clk) begin
				// comp step: comparison
				if (i != `EXP_SHIFT_COUNT) begin
					if (enable_comp)
						shift_comparisons[i] <= x_reg_comp < `EXP_SHIFT_LIMITS_i;
					else 
						shift_comparisons[i] <= shift_comparisons[i];
				end
				// sel step: input shift precomputation
				if (enable_sel)
					x_shifted[i] <= (i == `EXP_SHIFT_COUNT) ? x_reg_sel : (x_reg_sel + `EXP_SHIFT_VALUES_i);
				else
					x_shifted[i] <= x_shifted[i];
				// mul step: output shift computation
				if (enable_mul)
					mem_out_shifted[i] <= mem_out_shifted_reduced[i];
				else
					mem_out_shifted[i] <= mem_out_shifted[i];
			end
		end
	endgenerate
	
	// Compute the selected shift index by summing all shift_comparisons (sel step)
	generate
		if (`EXP_SHIFT_COUNT > 0) begin : shift_add_all
			multi_sum #(
				.VALUE_WIDTH(1),
				.VALUE_COUNT(`EXP_SHIFT_COUNT),
				.SUM_WIDTH(`EXP_SHIFT_INDEX_WIDTH)
			) select_adder (
				.values(shift_comparisons), 
				.sum(shift_comparisons_sum)
			);
			assign shift_select_out = `EXP_SHIFT_COUNT - shift_comparisons_sum;
		end else begin : shift_add_none
			assign shift_select_out = 0;
		end
	endgenerate
		
	// Address computation
	always @(posedge clk) begin
		if (enable_sel)
			shift_select <= shift_select_out;
		else
			shift_select <= shift_select;
		if (enable_addr)
			address <= (x_shifted[shift_select] >= (`EXP_MEM_COUNT + `EXP_MEM_MIN_VALUE)) ? (`EXP_MEM_COUNT - 1) : ((x_shifted[shift_select] < `EXP_MEM_MIN_VALUE) ? 0 : (x_shifted[shift_select] - `EXP_MEM_MIN_VALUE));
		else
			address <= address;
	end
	
	// Read synchronously the memory	(read step)
	always @(posedge clk) begin
		if (!enable_read)
			mem_out <= mem_out;
		else
			mem_out <= memory[address];
	end
	
	// Output control(out)
	always @(posedge clk) begin
		if (reset) begin
			y <= 0;
			new_result <= 0;
		end else if (enable_out) begin
			y <= mem_out_shifted[shift_select_reg_out];
			new_result <= 1;
		end else begin
			y <= y;
			new_result <= 0;
		end
	end
endmodule

`endif