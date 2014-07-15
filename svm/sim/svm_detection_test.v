`timescale 1ns / 1ps

`include "../config/svm_parameters.v"
`include "../../config/proj_env.v"
`define CLASS_TEST_COUNT 1000
`define CLASS_TEST_WIDTH 16

module svm_detection_test_v;

	// Inputs
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x;
	reg data_valid;
	reg clk;
	reg reset;

	// Outputs
	wire [`SVM_CLASS_WIDTH - 1:0] class;
	wire ready;
	wire new_result;
	
	reg error;
	
	// Instantiate the Unit Under Test (UUT)
	svm_detection uut (
		.x(x), 
		.data_valid(data_valid),
		.clk(clk), 
		.reset(reset),
		.class(class), 
		.ready(ready),
		.new_result(new_result)
	);
	
	// Initialize the memory with test vectors and classes
	reg [`SVM_CLASS_WIDTH-1:0] ref_classes [0:`CLASS_TEST_COUNT-1];
	initial begin
		$readmemh({`PROJ_CONFIG_DIR, "svm_parameters.test.classes.list"}, ref_classes);
	end
	generate
		genvar i;
		for (i = 0; i < `SVM_PARAM_COUNT; i = i + 1) begin: ref
			reg [`SVM_PARAM_WIDTH-1:0] vectors [0:`CLASS_TEST_COUNT-1];
			initial begin: init_param 
				reg [1:8 * 100] memory_name;
				$sformat(memory_name, "svm_parameters.test.vectors.%1d.list", i);
				$readmemh({`PROJ_CONFIG_DIR, memory_name}, vectors);
			end
		end
	endgenerate

	initial begin
		// Initialize Inputs
		clk = 1;
		reset = 1;

		#2000;
		
		reset = 0;
	end
	
	// Reference ROM management
	reg [`CLASS_TEST_WIDTH-1:0] input_counter;
	reg [`CLASS_TEST_WIDTH-1:0] output_counter;
	wire input_enable;
	wire output_enable;
	reg [`SVM_CLASS_WIDTH-1:0] output_ref;
	always @(posedge clk) begin
		if (output_enable)
			output_ref <= ref_classes[output_counter];
	end
	generate
		for (i = 0; i < `SVM_PARAM_COUNT; i = i + 1) begin: params_read
			always @(posedge clk) begin
				if (input_enable)
					x[(i+1) * `SVM_PARAM_WIDTH - 1 : i * `SVM_PARAM_WIDTH] <= ref[i].vectors[input_counter];
			end
		end
	endgenerate
	
	// Control
	assign input_enable = 1;
	assign output_enable = 1;
	
	always @(posedge clk) begin
		if (reset) begin
			input_counter <= 0;
			output_counter <= 0;
			error <= 0;
		end else begin
			if (ready && !data_valid) begin
				input_counter <= (input_counter + 1) % `CLASS_TEST_COUNT;
				data_valid <= 1;
			end else begin
				data_valid <= 0;
			end
			if (new_result) begin
				output_counter <= (output_counter + 1) % `CLASS_TEST_COUNT;
				error <= error || (output_ref != class);
			end
			
		end
	end
   
	always begin
	  #5 clk = ~clk;
	end
      
endmodule

