`timescale 1ns / 1ps

// Test of the simple hash

`include "../config/hash_parameters.v"

module tb_simple_hash;

	// Inputs
	reg [`HASH_WORD_WIDTH-1:0] data_in;

	// Outputs
	wire [`HASH_RESULT_WIDTH-1:0] data_out;

	// Instantiate the Unit Under Test (UUT)
	simple_hash uut (
		.data_in(data_in), 
		.data_out(data_out)
	);

	initial begin
        data_in = 0;
		#20;
        data_in = 1;
        #20;
        data_in = 12;
        #20;
        data_in = 45;
        #20;
        data_in = 75;
        #20;
        data_in = 836727523;
	end
      
endmodule

