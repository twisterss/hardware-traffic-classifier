`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Testbench for the CORDIC module
////////////////////////////////////////////////////////////////////////////////

`include "../config/svm_parameters.v"

// Do not load the module if not required to avoid compilation errors
`ifdef SVM_KERNEL_TYPE_CORDIC

module tb_cordic;

	// Inputs
	reg [`SVM_CORDIC_WIDTH - 1:0] x;
	reg [`SVM_CORDIC_WIDTH - 1:0] y;
	reg enable;
	reg clk;
	reg reset;

	// Outputs
	wire [`SVM_CORDIC_WIDTH - 1:0] result;
	wire data_valid_nxt;

	// Instantiate the Unit Under Test (UUT)
	cordic uut (
		.x(x), 
		.y(y), 
		.enable(enable), 
		.result(result), 
		.data_valid_nxt(data_valid_nxt), 
		.clk(clk), 
		.reset(reset)
	);
    
    // Clock simulation
    always begin
        #5 clk = !clk;
    end

	initial begin
		// Initialize Inputs
		x = 0;
		y = 0;
		enable = 0;
		clk = 1;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Stimulus
        reset = 0;
        
        #10
        
        x = 454545;
        y = 1234567;
        enable = 1;
        
        #10
        
        x = 1245;
        y = 86954;
        enable = 1;
        
        #10
        
        x = 475;
        y = 778589;
        enable = 1;
        
        #10
        
        x = 444855;
        y = 452;
        enable = 1;
        
        #10
        
        x = 147;
        y = 124;
        enable = 1;
        
        #10
        
        x = 0;
        y = 0;
        enable = 0;
	end
      
endmodule

`endif

