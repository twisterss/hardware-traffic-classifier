// Exponential test module
// Read inputs and expected outputs in memories, and compare
// to the output of the tested module.

`timescale 1ns / 1ps

`include "../config/exp_memory.v"
`define EXP_TEST_COUNT 1000

module exponential_test_v;

	// Inputs
	wire [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] x;
	reg clk;
	reg reset;
	reg data_valid;

	// Outputs
	wire [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] out;
	wire new_result;
	
	// Memories
	reg [`EXP_INPUT_WIDTH_INT + `EXP_INPUT_WIDTH_FRAC - 1:0] exp_input [0:`EXP_TEST_COUNT - 1];
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] exp_output [0:`EXP_TEST_COUNT - 1];

	// Variables
	reg [10:0] counter;
	wire [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted1;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted2;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted3;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted4;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted5;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted6;
	reg [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] mem_out_shifted;
	wire [`EXP_OUTPUT_WIDTH_INT + `EXP_OUTPUT_WIDTH_FRAC - 1:0] out_ref;
	wire error;

	// Instantiate the Unit Under Test (UUT)
	exponential uut (
		.x(x), 
		.y(out),
		.new_result(new_result),
		.clk(clk), 
		.reset(reset),
		.data_valid(data_valid)
	);
	
	// Initialize the memory
	initial begin
		$readmemh("exp_memory.test.input.list", exp_input);
		$readmemh("exp_memory.test.output.list", exp_output);
	end

	// Initialize Inputs
	initial begin
		clk = 1;
		reset = 1;
		data_valid = 0;
		counter = 0;
		#20
		reset = 0;
		data_valid = 1;
	end
   
	// Clock
	always begin
	  #5 clk = ~clk;
	end
	
	// Counter
	always @(posedge clk) begin
		if (~reset) begin
			counter<= (counter + 1) % `EXP_TEST_COUNT;
		end
		mem_out_shifted1 <= reset ? 0 : mem_out;
		mem_out_shifted2 <= reset ? 0 : mem_out_shifted1;
		mem_out_shifted3 <= reset ? 0 : mem_out_shifted2;
		mem_out_shifted4 <= reset ? 0 : mem_out_shifted3;
		mem_out_shifted5 <= reset ? 0 : mem_out_shifted4;
		mem_out_shifted6 <= reset ? 0 : mem_out_shifted5;
		mem_out_shifted <= reset ? 0 : mem_out_shifted6;
	end
      
	// Test signals
	assign x = exp_input[counter];
	assign mem_out = exp_output[counter];
	assign out_ref = mem_out_shifted;
	assign error = out != out_ref;
endmodule

