// Test of the square module

`timescale 1ns / 1ps

`define VALUE_WIDTH 12
`define VALUE_MAX 1500

module square_test;

	// Inputs
	reg signed [`VALUE_WIDTH-1:0] value;
	reg clk;
	reg reset;
	reg data_valid;

	// Outputs
	wire [(`VALUE_WIDTH-1) * 2 - 1:0] square;
	wire new_result;
	
	// Internal variables
	reg [(`VALUE_WIDTH-1) * 2 - 1:0] expected_square;
	reg [(`VALUE_WIDTH-1) * 2 - 1:0] expected_square1;
	reg [(`VALUE_WIDTH-1) * 2 - 1:0] expected_square2;
	wire error;
	

	// Instantiate the Unit Under Test (UUT)
	square #(
		.VALUE_WIDTH(`VALUE_WIDTH),
		.VALUE_MAX(`VALUE_MAX)
	) uut ( 
		.value(value), 
		.square(square), 
		.new_result(new_result),
		.clk(clk), 
		.reset(reset), 
		.data_valid(data_valid)
	);

	// Initialize Inputs
	initial begin
		value = -`VALUE_MAX;
		clk = 1;
		reset = 1;
		data_valid = 1;
		expected_square = 0;
		expected_square1 = 0;
		expected_square2 = 0;
		#20
		reset = 0;
	end
   
	// Clock
	always begin
	  #5 clk = ~clk;
	end
	
	// Counter
	always @(posedge clk) begin
		if (~reset) begin
			if (value + 9 <= `VALUE_MAX)
				value <= value + 9;
			else
				value <= -`VALUE_MAX;
			expected_square2 <= value * value;
			expected_square1 <= expected_square2;
			expected_square <= expected_square1;
		end
	end
	
	// Error checking
	assign error = square != expected_square;
      
endmodule

