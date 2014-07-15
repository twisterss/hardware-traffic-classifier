// SVM kernel test module

`timescale 1ns / 1ps

`include "../config/svm_parameters.v"

module svm_kernel_test;

	// Inputs
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] x;
	reg [`SVM_PARAM_WIDTH * `SVM_PARAM_COUNT - 1:0] sv;
	reg [`SVM_CLASS_WIDTH - 1:0] sv_class;
	reg [(`SVM_COEF_WIDTH_INT + `SVM_COEF_WIDTH_FRAC) * (`SVM_CLASS_COUNT - 1) - 1:0] coef;
	reg [`SVM_COEF_SIGN_WIDTH * (`SVM_CLASS_COUNT - 1) - 1:0] coef_sign;
	reg new_computation;
	reg clk;
	reg reset;
	reg data_valid;

	// Outputs
	wire [(`SVM_DISTANCE_WIDTH_INT + `SVM_DISTANCE_WIDTH_FRAC) * (`SVM_DECISION_COUNT) - 1:0] distance;

	// Instantiate the Unit Under Test (UUT)
    `ifdef SVM_KERNEL_TYPE_RBF
        // RBF kernel computation
        svm_kernel_rbf uut (
            .x(x), 
            .sv(sv), 
            .sv_class(sv_class), 
            .coef_ln(coef), 
            .coef_sign(coef_sign),
            .new_computation(new_computation), 
            .data_valid(data_valid),
            .clk(clk), 
            .reset(reset), 
            .distance(distance)
        );
    `endif
    `ifdef SVM_KERNEL_TYPE_CORDIC
        // CORDIC kernel computation
        svm_kernel_cordic uut (
            .x(x), 
            .sv(sv), 
            .sv_class(sv_class), 
            .coef_scaled(coef), 
            .coef_sign(coef_sign),
            .new_computation(new_computation), 
            .data_valid(data_valid),
            .clk(clk), 
            .reset(reset), 
            .distance(distance)
        );
    `endif
   
	// Clock
	always begin
	  #5 clk = ~clk;
	end

	// Initialize Inputs
	initial begin
		x = 0;
		sv = 0;
		sv_class = 0;
		coef = 0;
		coef_sign = 0;
		clk = 1;
		reset = 1;
		data_valid = 0;
		new_computation = 0;

		// Wait for global reset to finish
		#2000;
		reset = 0;

		// Wait before enable
		#100;
        
		// Constant value
		data_valid = 1;
		x = {`SVM_PARAM_WIDTH'd45, `SVM_PARAM_WIDTH'd1200, `SVM_PARAM_WIDTH'd128};
		sv = {`SVM_PARAM_WIDTH'd50, `SVM_PARAM_WIDTH'd1180, `SVM_PARAM_WIDTH'd120};
		sv_class = 3'd0;
		coef = {-14'sd25, 14'sd0, 14'sd18, 14'sd1258};
		coef_sign = {1'b1, 1'b0, 1'b1, 1'b1};
		
		// Other constant value
		#10
		x = {`SVM_PARAM_WIDTH'd1020, `SVM_PARAM_WIDTH'd182, `SVM_PARAM_WIDTH'd0};
		sv = {`SVM_PARAM_WIDTH'd950, `SVM_PARAM_WIDTH'd350, `SVM_PARAM_WIDTH'd120};
		sv_class = 3'd2;
		coef = {14'sd4587, 14'sd12, -14'sd5269, 14'sd0};
		coef_sign = {1'b1, 1'b1, 1'b1, 1'b0};
		
		// Disable
		#10
		data_valid = 0;
		x = 0;
		sv = 0;
		sv_class = 0;
		coef = 0;
	end
      
endmodule

