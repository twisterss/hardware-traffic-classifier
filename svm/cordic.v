//////////////////////////////////////////////////////////////////////////////////
// Compute y*2^-x using the CORDIC algorithm.
// This is implemented as a pipeline, receives at each clock cycle,
// the result is available after `SVM_CORDIC_STEPS clock cycles.
// x and y  and result are unsigned with a fixed width of 0,`SVM_CORDIC_WIDTH.
// data_valid_nxt is set to 1 one clock cycle before result is set to valid data
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

`include "config/svm_parameters.v"

// Do not load the module if not required to avoid compilation errors
`ifdef SVM_KERNEL_TYPE_CORDIC

module cordic
(
	input [`SVM_CORDIC_WIDTH - 1:0] x,
	input [`SVM_CORDIC_WIDTH - 1:0] y,
	input enable,

	output [`SVM_CORDIC_WIDTH - 1:0] result,
	output data_valid_nxt,
	
	input clk,
	input reset
);

	// Variables for each step
	reg [`SVM_CORDIC_INTER_WIDTH-1:0] e_value [0:`SVM_CORDIC_STEPS];
	reg [`SVM_CORDIC_INTER_WIDTH-1:0] b_value [0:`SVM_CORDIC_STEPS];
	reg step_en [0:`SVM_CORDIC_STEPS];

	// Initialization
	always @(*) begin
		e_value[0] = {x, {(`SVM_CORDIC_INTER_WIDTH - `SVM_CORDIC_WIDTH){1'b0}}};
		b_value[0] = {y, {(`SVM_CORDIC_INTER_WIDTH - `SVM_CORDIC_WIDTH){1'b0}}};
		step_en[0] = reset ? 0 : enable;
	end

	// Computation
	generate
		genvar step_id;
		for (step_id = 0; step_id < `SVM_CORDIC_STEPS; step_id = step_id + 1) begin: cordic_step
			// One CORDIC step
			always @(posedge clk) begin
				if (e_value[step_id] >= `SVM_CORDIC_INCR_LOG_step) begin
					e_value[step_id+1] <= e_value[step_id] - `SVM_CORDIC_INCR_LOG_step;
					b_value[step_id+1] <= b_value[step_id] - (b_value[step_id] >> `SVM_CORDIC_INCR_step);
				end else begin
					e_value[step_id+1] <= e_value[step_id];
					b_value[step_id+1] <= b_value[step_id];
				end
				step_en[step_id+1] <= step_en[step_id];
			end
		end
	endgenerate

	// Return the result
	assign result = b_value[`SVM_CORDIC_STEPS][`SVM_CORDIC_INTER_WIDTH - 1:`SVM_CORDIC_INTER_WIDTH - `SVM_CORDIC_WIDTH];
	assign data_valid_nxt = step_en[`SVM_CORDIC_STEPS-1];
endmodule

`endif