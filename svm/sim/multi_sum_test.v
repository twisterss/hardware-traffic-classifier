// Test the multiple sum module with some values

`timescale 1ns / 1ps

module multi_sum_test;

	// Inputs
	reg [2:0] values_tab [0:3];
	wire [11:0] values;

	// Outputs
	wire [4:0] sum;
	
	// Internal
	wire error;

	// Instantiate the Unit Under Test (UUT)
	multi_sum #(
		.VALUE_WIDTH(3),
		.VALUE_COUNT(4),
		.SUM_WIDTH(5)
	) uut (
		.values(values), 
		.sum(sum)
	);

	// Inputs
	initial begin
		values_tab[0] = 0;
		values_tab[1] = 0;
		values_tab[2] = 0;
		values_tab[3] = 0;
		#10;
		values_tab[0] = 2;
		values_tab[1] = 0;
		values_tab[2] = 2;
		values_tab[3] = 0;
		#10;
		values_tab[0] = 2;
		values_tab[1] = 2;
		values_tab[2] = 2;
		values_tab[3] = 2;
		#10;
		values_tab[0] = 7;
		values_tab[1] = 7;
		values_tab[2] = 7;
		values_tab[3] = 0;
		#10;
		values_tab[0] = 7;
		values_tab[1] = 6;
		values_tab[2] = 5;
		values_tab[3] = 4;
	end
	
	// Values packing
	assign values = {values_tab[3], values_tab[2], values_tab[1], values_tab[0]};
      
	// Error?
	assign error = sum != (values_tab[3] + values_tab[2] + values_tab[1] + values_tab[0]);
endmodule

